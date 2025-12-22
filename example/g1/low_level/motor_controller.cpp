#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/time.h>
#include <map>
#include <array>
#include <ctime>
#include <iomanip>
#include <atomic>
#include <chrono>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include "xixilowcmd/xixilowcmd/msg/low_cmd.hpp"
#include "xixilowcmd/xixilowcmd/msg/motor_cmd.hpp"

// Optional: Keep DDS for backwards compatibility if needed
#ifdef USE_DDS
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#endif

// ROS includes (for backwards compatibility)
#ifdef USE_ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include "MotorFeedback.h"
#endif

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_STATE_TOPIC = "rt/lowstate";
static const std::string ROS2_CMD_TOPIC = "/lowcmd";

// Using directives
const int G1_NUM_MOTOR = 30;  // Updated for xixilowcmd format (30 motors)

// CRC32 calculation function (matches the original)
inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else
                CRC32 <<= 1;
            if (data & xbit) CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
};

// TCP configuration for motor command forwarding
struct TcpConfig {
    std::string remote_ip = "127.0.0.1";  // Local IP by default
    int remote_port = 8888;                // TCP port for motor commands
    int socket_fd = -1;
    bool initialized = false;
};

// Motor command structure (matches STM32 protocol)
struct MotorCommandCan {
    uint16_t motor_id;
    float pos;      // Target position (rad)
    float vel;      // Target velocity (rad/s)
    float kp;       // Position gain
    float kd;       // Velocity gain
    float torq;     // Feedforward torque (N·m)
};

// Motor ID mapping function
// Motors 1-15 -> CAN IDs 0x01-0x0F
// Motors 16-30 -> CAN IDs 0x10-0x1F
// Motor ID 0 is not used (reserved)
int get_can_id_for_motor(int motor_id) {
    if (motor_id >= 1 && motor_id <= 15) {
        return motor_id;  // 1-15 -> 0x01-0x0F
    } else if (motor_id >= 16 && motor_id <= 30) {
        return motor_id;  // 16-30 -> 0x10-0x1F
    }
    return 0;  // Invalid or out of range
}

class ROS2_to_TCP_Bridge : public rclcpp::Node {
private:
    TcpConfig tcp_config_;

    // ROS2 components
    rclcpp::Subscription<xixilowcmd::msg::LowCmd>::SharedPtr lowcmd_subscriber_;

#ifdef USE_DDS
    // Optional: Keep DDS for backwards compatibility
    ChannelSubscriberPtr<LowCmd_> lowcmd_subscriber_dds_;
    ChannelPublisherPtr<LowState_> lowstate_publisher_;
#endif

    // Debug control flags
    bool verbose_logging_ = false;  // Set to true to enable detailed logging

#ifdef USE_ROS
    // ROS components
    ros::NodeHandle* ros_nh_ = nullptr;
    ros::Publisher motor_feedback_pub_;
    std::string ros_topic_name_ = "/motor_feedback";
    bool enable_ros_publish_ = false;
#endif

    // Motor feedback storage
    struct MotorFeedbackData {
        int motor_id;
        int can_id;
        float position;
        float velocity;
        float torque;
        float temperature_mos;
        float temperature_coil;
        int mode;
        bool error;
        int error_code;
        bool valid;
    };
    std::array<MotorFeedbackData, 4> motor_feedback_data_;  // For 4 motors

    // Store latest motor commands
    std::array<MotorCommandCan, G1_NUM_MOTOR> motor_commands_;

    // 500Hz定时发送相关
    std::thread can_send_thread_;
    std::atomic<bool> should_stop_{false};

    // 插值数据存储
    struct MotorCommandHistory {
        MotorCommandCan current;
        MotorCommandCan previous;
        bool has_previous = false;
        std::chrono::high_resolution_clock::time_point current_timestamp;
        std::chrono::high_resolution_clock::time_point previous_timestamp;
    };
    std::array<MotorCommandHistory, G1_NUM_MOTOR> command_history_;

public:
    ROS2_to_TCP_Bridge(bool verbose = false) : Node("ros2_to_tcp_bridge") {
        verbose_logging_ = verbose;

        // Initialize motor commands to zero
        for (int i = 0; i < G1_NUM_MOTOR; i++) {
            motor_commands_[i] = {static_cast<uint16_t>(i), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        }

        std::cout << "ROS2-to-TCP Bridge Initializing..." << std::endl;
        if (verbose_logging_) {
            std::cout << "  Verbose logging: ENABLED" << std::endl;
        } else {
            std::cout << "  Verbose logging: DISABLED (use -v to enable)" << std::endl;
        }
    }

    ~ROS2_to_TCP_Bridge() {
        // 停止500Hz发送线程
        should_stop_ = true;
        if (can_send_thread_.joinable()) {
            can_send_thread_.join();
        }

        if (tcp_config_.initialized && tcp_config_.socket_fd >= 0) {
            close(tcp_config_.socket_fd);
        }

#ifdef USE_ROS
        if (ros_nh_) {
            delete ros_nh_;
        }
#endif
    }

    bool InitializeTCP(const std::string& tcp_endpoint) {
        // Parse endpoint (format: "ip:port")
        size_t colon_pos = tcp_endpoint.find(':');
        if (colon_pos != std::string::npos) {
            tcp_config_.remote_ip = tcp_endpoint.substr(0, colon_pos);
            tcp_config_.remote_port = std::stoi(tcp_endpoint.substr(colon_pos + 1));
        }

        std::cout << "Initializing TCP connection to " << tcp_config_.remote_ip
                  << ":" << tcp_config_.remote_port << std::endl;

        // Create TCP socket
        tcp_config_.socket_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (tcp_config_.socket_fd < 0) {
            perror("TCP socket creation failed");
            return false;
        }

        // Set socket options for better performance
        int opt = 1;
        setsockopt(tcp_config_.socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        // Configure server address
        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(tcp_config_.remote_port);

        if (inet_pton(AF_INET, tcp_config_.remote_ip.c_str(), &server_addr.sin_addr) <= 0) {
            std::cerr << "Invalid IP address: " << tcp_config_.remote_ip << std::endl;
            close(tcp_config_.socket_fd);
            return false;
        }

        // Connect to TCP server
        std::cout << "Connecting to TCP server..." << std::endl;
        if (connect(tcp_config_.socket_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            perror("TCP connection failed");
            std::cerr << "Make sure the TCP server is running at " << tcp_config_.remote_ip
                      << ":" << tcp_config_.remote_port << std::endl;
            close(tcp_config_.socket_fd);
            return false;
        }

        tcp_config_.initialized = true;
        std::cout << "✓ TCP connection established successfully" << std::endl;
        std::cout << "📡 Current TCP configuration:" << std::endl;
        std::cout << "   - Remote IP: " << tcp_config_.remote_ip << std::endl;
        std::cout << "   - Remote Port: " << tcp_config_.remote_port << std::endl;
        std::cout << "   - Socket FD: " << tcp_config_.socket_fd << std::endl;

        // Test connection by sending a test message
        const char* test_msg = "TCP_INIT";
        if (send(tcp_config_.socket_fd, test_msg, strlen(test_msg), 0) > 0) {
            std::cout << "✓ TCP connection test: Successfully sent test message" << std::endl;
        } else {
            std::cout << "! TCP connection test: Failed to send test message" << std::endl;
        }

        return true;
    }

    void InitializeROS2() {
        std::cout << "🔗 Initializing ROS2 subscriber on topic: " << ROS2_CMD_TOPIC << std::endl;

        // Create ROS2 subscriber
        lowcmd_subscriber_ = this->create_subscription<xixilowcmd::msg::LowCmd>(
            ROS2_CMD_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            std::bind(&ROS2_to_TCP_Bridge::LowCmdHandler, this, std::placeholders::_1)
        );

        std::cout << "✓ ROS2 LowCmd subscriber created and initialized" << std::endl;
        std::cout << "🎯 ROS2 initialization complete! Waiting for messages..." << std::endl;
    }

#ifdef USE_DDS
    void InitializeDDS(const std::string& network_interface) {
        std::cout << "🔗 Initializing DDS on interface: " << network_interface << std::endl;

        // Initialize DDS channel
        ChannelFactory::Instance()->Init(0, network_interface);
        std::cout << "✓ DDS ChannelFactory initialized" << std::endl;

        // Subscribe to lowcmd topic
        lowcmd_subscriber_dds_.reset(new ChannelSubscriber<LowCmd_>(HG_CMD_TOPIC));
        std::cout << "✓ DDS LowCmd subscriber created" << std::endl;

        lowcmd_subscriber_dds_->InitChannel(std::bind(&ROS2_to_TCP_Bridge::LowCmdHandlerDDS, this, std::placeholders::_1), 1);
        std::cout << "✓ DDS LowCmd subscriber initialized on topic: " << HG_CMD_TOPIC << std::endl;

        // Create publisher for lowstate (to send feedback)
        lowstate_publisher_.reset(new ChannelPublisher<LowState_>(HG_STATE_TOPIC));
        lowstate_publisher_->InitChannel();
        std::cout << "✓ DDS LowState publisher initialized on topic: " << HG_STATE_TOPIC << std::endl;

        std::cout << "🎯 DDS initialization complete! Waiting for messages..." << std::endl;
    }
#endif

    void LowCmdHandler(const xixilowcmd::msg::LowCmd::SharedPtr msg) {
        static int ros2_message_count = 0;
        static auto first_message_time = std::chrono::high_resolution_clock::now();
        ros2_message_count++;

        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - first_message_time).count() / 1000.0;

        // 🎯 打印每次收到的 ROS2 /lowcmd 消息
        std::cout << "\n" << std::string(80, '=') << std::endl;
        std::cout << "📨 ROS2 LowCmd 消息接收 #" << ros2_message_count
                  << " (时间: " << std::fixed << std::setprecision(3) << elapsed << "s)" << std::endl;
        std::cout << std::string(80, '-') << std::endl;

        // 打印基本消息信息
        std::cout << "📋 基本信息:" << std::endl;
        std::cout << "   Topic: " << ROS2_CMD_TOPIC << std::endl;
        std::cout << "   电机数量: " << msg->motor_cmd.size() << std::endl;

        // 计算消息频率
        static auto last_message_time = current_time;
        if (ros2_message_count > 1) {
            auto message_interval = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_message_time).count();
            double frequency = 1000000.0 / message_interval;
            std::cout << "   消息频率: " << std::fixed << std::setprecision(1) << frequency << " Hz" << std::endl;
        }
        last_message_time = current_time;

        int motors_with_commands = 0;
        // Extract motor commands from ROS2 message
        for (const auto& motor_cmd : msg->motor_cmd) {
            int motor_id = motor_cmd.id;

            if (motor_id >= 0 && motor_id < G1_NUM_MOTOR) {
                motor_commands_[motor_id].motor_id = motor_id;
                motor_commands_[motor_id].pos = motor_cmd.q;
                motor_commands_[motor_id].vel = motor_cmd.dq;
                motor_commands_[motor_id].kp = motor_cmd.kp;
                motor_commands_[motor_id].kd = motor_cmd.kd;
                motor_commands_[motor_id].torq = motor_cmd.tau;

                // Check if this motor has a valid ID mapping
                int can_id = get_can_id_for_motor(motor_id);
                if (can_id > 0) {
                    motor_commands_[motor_id].motor_id = can_id;

                    // 更新命令历史用于插值
                    auto& history = command_history_[motor_id];

                    // 保存上一个命令
                    if (history.has_previous) {
                        history.previous = history.current;
                        history.previous_timestamp = history.current_timestamp;
                    } else {
                        // 第一个命令，创建一个拷贝作为上一个命令
                        history.previous = motor_commands_[motor_id];
                        history.previous_timestamp = current_time;
                        history.has_previous = true;
                    }

                    // 更新当前命令
                    history.current = motor_commands_[motor_id];
                    history.current_timestamp = current_time;

                    motors_with_commands++;
                }
            }
        }

        // 📊 打印电机命令详情
        std::cout << "\n🔧 电机命令详情:" << std::endl;
        std::cout << "   总电机数: " << G1_NUM_MOTOR << " (消息中: " << msg->motor_cmd.size() << ")" << std::endl;

        // 创建关节名称映射
        std::map<int, std::string> joint_names = {
            {0, "LeftHipPitch"}, {1, "LeftHipRoll"}, {2, "LeftHipYaw"}, {3, "LeftKnee"},
            {4, "LeftAnklePitch"}, {5, "LeftAnkleRoll"},
            {6, "RightHipPitch"}, {7, "RightHipRoll"}, {8, "RightHipYaw"}, {9, "RightKnee"},
            {10, "RightAnklePitch"}, {11, "RightAnkleRoll"}
        };

        // 打印所有非零命令
        std::cout << "   非零电机命令:" << std::endl;
        for (const auto& motor_cmd : msg->motor_cmd) {
            int motor_id = motor_cmd.id;

            if (motor_cmd.mode != 0 || fabs(motor_cmd.q) > 0.001 ||
                fabs(motor_cmd.dq) > 0.001 || fabs(motor_cmd.tau) > 0.001 ||
                motor_cmd.kp > 0.1 || motor_cmd.kd > 0.1) {

                std::string joint_name = (joint_names.find(motor_id) != joint_names.end()) ?
                                        joint_names[motor_id] : ("Motor" + std::to_string(motor_id));

                int can_id = get_can_id_for_motor(motor_id);
                std::string can_info = (can_id > 0) ?
                                       (", CAN ID: 0x" + std::to_string(can_id)) : "";

                std::cout << "     " << std::setw(16) << std::left << joint_name
                          << " (ID:" << std::setw(2) << motor_id << can_info << ")" << std::endl;
                std::cout << "         模式:" << std::setw(2) << static_cast<int>(motor_cmd.mode)
                          << " 位置:" << std::setw(8) << std::fixed << std::setprecision(3) << motor_cmd.q
                          << " 速度:" << std::setw(8) << motor_cmd.dq
                          << "  力矩:" << std::setw(8) << motor_cmd.tau << std::endl;
                std::cout << "         增益: Kp=" << std::setw(6) << motor_cmd.kp
                          << " Kd=" << std::setw(6) << motor_cmd.kd << std::endl;
            }
        }

        // 特别显示脚踝关节（即使为零）
        std::cout << "\n   🎯 重点关注的脚踝关节:" << std::endl;
        for (int ankle_id : {4, 5, 10, 11}) {
            for (const auto& motor_cmd : msg->motor_cmd) {
                if (motor_cmd.id == ankle_id) {
                    std::string joint_name = (joint_names.find(ankle_id) != joint_names.end()) ?
                                            joint_names[ankle_id] : ("Motor" + std::to_string(ankle_id));

                    int can_id = get_can_id_for_motor(ankle_id);
                    std::string can_info = (can_id > 0) ?
                                           (", CAN ID: 0x" + std::to_string(can_id)) : "";

                    std::cout << "     " << std::setw(16) << std::left << joint_name
                              << " (ID:" << std::setw(2) << ankle_id << can_info << ")" << std::endl;
                    std::cout << "         位置:" << std::setw(8) << std::fixed << std::setprecision(3) << motor_cmd.q
                              << "  速度:" << std::setw(8) << motor_cmd.dq
                              << "  Kp:" << std::setw(6) << motor_cmd.kp
                              << "  Kd:" << std::setw(6) << motor_cmd.kd << std::endl;
                    break;
                }
            }
        }

        std::cout << "\n📊 统计信息:" << std::endl;
        std::cout << "   有CAN映射的电机数: " << motors_with_commands << "/4" << std::endl;

        // 原有的调试输出（减少频率）
        if (ros2_message_count % 50 == 0) {
            std::cout << "📥 ROS2 Messages: " << ros2_message_count
                     << " | Motors updated: " << motors_with_commands << std::endl;

            // 显示插值状态（显示前8个电机和脚踝关节）
            std::cout << "🔄 Interpolation status: ";
            int display_count = 0;
            for (int i = 1; i <= 30 && display_count < 8; i++) {
                int can_id = get_can_id_for_motor(i);
                if (can_id > 0) {
                    auto& history = command_history_[i];
                    std::cout << "M" << i << "[" << std::hex << can_id << std::dec << "] ";
                    if (history.has_previous) {
                        auto time_diff = std::chrono::duration<double>(history.current_timestamp - history.previous_timestamp).count();
                        std::cout << time_diff*1000 << "ms ";
                    } else {
                        std::cout << "init ";
                    }
                    display_count++;
                }
            }
            // 特别显示脚踝关节
            for (int ankle_id : {4, 5, 10, 11}) {
                if (display_count >= 8) break;
                int can_id = get_can_id_for_motor(ankle_id);
                if (can_id > 0) {
                    auto& history = command_history_[ankle_id];
                    std::cout << "A" << ankle_id << "[" << std::hex << can_id << std::dec << "] ";
                    if (history.has_previous) {
                        auto time_diff = std::chrono::duration<double>(history.current_timestamp - history.previous_timestamp).count();
                        std::cout << time_diff*1000 << "ms ";
                    } else {
                        std::cout << "init ";
                    }
                    display_count++;
                }
            }
            std::cout << std::endl;
        }
    }

    void SendMotorCommandTCP(const MotorCommandCan& cmd) {
        if (!tcp_config_.initialized) {
            std::cerr << "TCP not initialized, skipping command" << std::endl;
            return;
        }

        // Create TCP message packet for motor command
        // Format: [HEADER(4 bytes) | MOTOR_ID(2 bytes) | POS(4 bytes) | VEL(4 bytes) | Kp(4 bytes) | Kd(4 bytes) | TORQ(4 bytes) | CRC32(4 bytes)]
        struct MotorCommandPacket {
            uint32_t header;
            uint16_t motor_id;
            float pos;
            float vel;
            float kp;
            float kd;
            float torq;
            uint32_t crc32;
        } __attribute__((packed));

        MotorCommandPacket packet;
        packet.header = 0xDEADBEEF;  // Magic number for motor command
        packet.motor_id = cmd.motor_id;
        packet.pos = cmd.pos;
        packet.vel = cmd.vel;
        packet.kp = cmd.kp;
        packet.kd = cmd.kd;
        packet.torq = cmd.torq;

        // Calculate CRC32 for data integrity
        packet.crc32 = Crc32Core((uint32_t*)&packet, sizeof(packet) / sizeof(uint32_t) - 1);

        // Send TCP packet
        ssize_t bytes_sent = send(tcp_config_.socket_fd, &packet, sizeof(packet), MSG_NOSIGNAL);
        if (bytes_sent != sizeof(packet)) {
            std::cerr << "✗ Error sending TCP command for motor 0x" << std::hex << cmd.motor_id << std::dec
                     << " (sent " << bytes_sent << " bytes, expected " << sizeof(packet) << ")" << std::endl;
            if (bytes_sent < 0) {
                perror("TCP send error");
            }
        } else {
            // Reduced logging - only log errors and startup
            static int counter = 0;
            counter++;

            // Only log every 100th command to reduce spam
            if (counter % 100 == 0 || verbose_logging_) {
                std::cout << "✓ Sent " << counter << " TCP commands (Motor 0x" << std::hex
                         << cmd.motor_id << std::dec << ": pos=" << std::fixed << std::setprecision(3)
                         << cmd.pos << ")" << std::endl;
            }
        }
    }

    void PublishSimulatedState() {
        // In ROS2 version, we don't need to publish state feedback
        // This function can be used for debugging if needed
        // For now, we'll just log the current motor commands
        if (verbose_logging_) {
            static int counter = 0;
            if (++counter % 100 == 0) {
                std::cout << "📊 Motor commands update #" << counter << std::endl;
                // 显示前10个有有效映射的电机
                int display_count = 0;
                for (int i = 1; i <= 30 && display_count < 10; i++) {
                    int can_id = get_can_id_for_motor(i);
                    if (can_id > 0) {
                        auto& cmd = motor_commands_[i];
                        std::cout << "  Motor " << i << " (CAN 0x" << std::hex << can_id << std::dec
                                  << "): pos=" << std::fixed << std::setprecision(3) << cmd.pos
                                  << " vel=" << cmd.vel << " kp=" << cmd.kp << " kd=" << cmd.kd << std::endl;
                        display_count++;
                    }
                }
            }
        }
    }

#ifdef USE_ROS
    void InitializeROS(int argc, char** argv) {
        // Initialize ROS
        ros::init(argc, argv, "dds_can_bridge", ros::init_options::NoSigintHandler);
        ros_nh_ = new ros::NodeHandle("~");

        // Get ROS topic name from parameter server
        ros_nh_->param<std::string>("motor_feedback_topic", ros_topic_name_, "/motor_feedback");
        ros_nh_->param<bool>("enable_ros_publish", enable_ros_publish_, true);

        // Create publisher
        motor_feedback_pub_ = ros_nh_->advertise<MotorFeedback>(ros_topic_name_, 10);

        // Initialize motor feedback data
        for (int i = 0; i < 4; i++) {
            motor_feedback_data_[i].valid = false;
            motor_feedback_data_[i].motor_id = i + 1;
        }

        ROS_INFO("ROS initialized, publishing to topic: %s", ros_topic_name_.c_str());
    }

    // Parse CAN feedback data (MIT motor protocol)
    MotorFeedbackData ParseMotorFeedback(int can_id, const uint8_t* data, int dlc) {
        MotorFeedbackData feedback;
        feedback.can_id = can_id;
        feedback.valid = false;

        if (dlc < 8) {
            return feedback;  // Invalid frame length
        }

        // Parse motor ID and state
        feedback.motor_id = data[0] & 0x0F;
        feedback.error_code = (data[0] >> 4) & 0x0F;
        feedback.error = (feedback.error_code != 0);
        feedback.mode = feedback.error ? 2 : 1;  // 2=error, 1=enabled, 0=disabled

        // Parse position (16-bit signed, little-endian)
        int16_t pos_int = (int16_t)((data[1] << 8) | data[0]);
        feedback.position = pos_int * 12.5f / 32767.0f;  // Scale to ±12.5 rad

        // Parse velocity (12-bit signed)
        int16_t vel_int = (int16_t)(((data[3] & 0x0F) << 8) | data[2]);
        feedback.velocity = vel_int * 30.0f / 2047.0f;  // Scale to ±30 rad/s

        // Parse torque (12-bit signed)
        int16_t torque_int = (int16_t)(((data[4] & 0x0F) << 8) | data[5]);
        feedback.torque = torque_int * 10.0f / 2047.0f;  // Scale to ±10 Nm

        // Parse temperature
        feedback.temperature_mos = (float)data[6];
        feedback.temperature_coil = (float)data[7];

        feedback.valid = true;
        return feedback;
    }

    void PublishMotorFeedbackROS(int motor_index, const MotorFeedbackData& feedback) {
        if (!enable_ros_publish_ || !ros_nh_ || motor_feedback_pub_.getNumSubscribers() == 0) {
            return;
        }

        MotorFeedback msg;

        // Header
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "can_frame";

        // Motor identification
        msg.motor_id = feedback.motor_id;
        msg.can_id = feedback.can_id;

        // Motor state
        msg.mode = feedback.mode;
        msg.error = feedback.error;
        msg.error_code = feedback.error_code;

        // Motor measurements
        msg.position = feedback.position;
        msg.velocity = feedback.velocity;
        msg.torque = feedback.torque;
        msg.current = 0.0f;  // Not available in this protocol

        // Gains (set to 0 for feedback)
        msg.kp = 0.0f;
        msg.kd = 0.0f;

        // Temperature
        msg.temperature_mos = feedback.temperature_mos;
        msg.temperature_coil = feedback.temperature_coil;

        // Publish message
        motor_feedback_pub_.publish(msg);
    }
#endif

    // 线性插值函数
    MotorCommandCan interpolateCommand(const MotorCommandCan& prev, const MotorCommandCan& curr,
                                       double prev_time, double curr_time, double target_time) {
        MotorCommandCan result = curr;

        if (curr_time > prev_time) {
            double t = (target_time - prev_time) / (curr_time - prev_time);
            t = std::max(0.0, std::min(1.0, t));  // 限制在[0,1]范围内

            result.pos = prev.pos + t * (curr.pos - prev.pos);
            result.vel = prev.vel + t * (curr.vel - prev.vel);
            result.kp = prev.kp + t * (curr.kp - prev.kp);
            result.kd = prev.kd + t * (curr.kd - prev.kd);
            result.torq = prev.torq + t * (curr.torq - prev.torq);
        }

        return result;
    }

    // 500Hz TCP发送线程
    void TCPSendThread() {
        std::cout << "🔄 500Hz TCP发送线程启动" << std::endl;

        const auto interval = std::chrono::milliseconds(2); // 500Hz = 2ms
        auto next_send_time = std::chrono::high_resolution_clock::now();

        while (!should_stop_) {
            auto now = std::chrono::high_resolution_clock::now();

            // 检查是否到了发送时间
            if (now >= next_send_time) {
                // 为每个有有效映射的电机发送插值命令
                for (int motor_id = 1; motor_id <= 30; motor_id++) {
                    int can_id = get_can_id_for_motor(motor_id);
                    if (can_id > 0) {
                        auto& history = command_history_[motor_id];
                        auto current_time = std::chrono::duration<double>(now.time_since_epoch()).count();

                        MotorCommandCan cmd_to_send;

                        if (history.has_previous) {
                            // 使用线性插值
                            double prev_time = std::chrono::duration<double>(history.previous_timestamp.time_since_epoch()).count();
                            double curr_time = std::chrono::duration<double>(history.current_timestamp.time_since_epoch()).count();

                            cmd_to_send = interpolateCommand(history.previous, history.current,
                                                           prev_time, curr_time, current_time);
                        } else {
                            // 没有历史数据，使用当前命令
                            cmd_to_send = history.current;
                        }

                        cmd_to_send.motor_id = can_id;
                        SendMotorCommandTCP(cmd_to_send);
                    }
                }

                // 计算下一次发送时间
                next_send_time += interval;

                // 如果已经落后太多，跳过一些周期
                if (now > next_send_time + interval) {
                    next_send_time = now + interval;
                }
            }

            // 短暂休眠以避免占用CPU
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        std::cout << "🛑 500Hz TCP发送线程停止" << std::endl;
    }

    void Run() {
        std::cout << "\nROS2-to-TCP Bridge Running..." << std::endl;
        std::cout << "⚡ 500Hz TCP发送已启用 (带线性插值)" << std::endl;
        std::cout << "Waiting for ROS2 commands on topic: " << ROS2_CMD_TOPIC << std::endl;
        std::cout << "Sending TCP commands to: " << tcp_config_.remote_ip << ":" << tcp_config_.remote_port << std::endl;
        std::cout << "Press Ctrl+C to stop" << std::endl << std::endl;

        // 启动500Hz TCP发送线程
        can_send_thread_ = std::thread(&ROS2_to_TCP_Bridge::TCPSendThread, this);

        // 启动TCP监控线程
        std::cout << "🚀 Starting TCP monitor thread..." << std::endl;
        std::thread tcp_monitor_thread(&ROS2_to_TCP_Bridge::MonitorTCP, this);
        tcp_monitor_thread.detach();
        std::cout << "✓ TCP monitor thread started (detached)" << std::endl;

        // 使用 ROS2 spin 来处理消息，同时在单独的线程中执行 PublishSimulatedState
        std::thread state_thread([this]() {
            while (rclcpp::ok()) {
                PublishSimulatedState();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100Hz for state updates
            }
        });
        state_thread.detach();

        rclcpp::spin(shared_from_this());
    }

    void MonitorTCP() {
        if (!tcp_config_.initialized) {
            return;
        }

        std::cout << "📡 TCP monitoring started for connection to " << tcp_config_.remote_ip
                  << ":" << tcp_config_.remote_port << std::endl;
        std::cout << "   Listening for motor feedback from TCP server" << std::endl;

        char buffer[1024];
        int tcp_rx_count = 0;

        while (tcp_config_.initialized) {
            // Try to read from TCP socket with timeout
            fd_set read_fds;
            struct timeval timeout;
            FD_ZERO(&read_fds);
            FD_SET(tcp_config_.socket_fd, &read_fds);

            timeout.tv_sec = 1;  // 1 second timeout
            timeout.tv_usec = 0;

            int select_result = select(tcp_config_.socket_fd + 1, &read_fds, NULL, NULL, &timeout);

            if (select_result > 0 && FD_ISSET(tcp_config_.socket_fd, &read_fds)) {
                ssize_t bytes_received = recv(tcp_config_.socket_fd, buffer, sizeof(buffer) - 1, 0);

                if (bytes_received > 0) {
                    tcp_rx_count++;
                    buffer[bytes_received] = '\0';  // Null-terminate for safety

                    if (verbose_logging_) {
                        std::cout << "📨 Received TCP message #" << tcp_rx_count
                                 << " | Size: " << bytes_received << " bytes" << std::endl;

                        // Show first part of message for debugging
                        std::cout << "    Content: ";
                        int show_bytes = std::min((int)bytes_received, 64);
                        for (int i = 0; i < show_bytes; i++) {
                            if (isprint(buffer[i])) {
                                std::cout << buffer[i];
                            } else {
                                std::cout << "\\x" << std::hex << std::setw(2) << std::setfill('0')
                                         << (int)(unsigned char)buffer[i] << std::dec;
                            }
                        }
                        if (bytes_received > show_bytes) {
                            std::cout << "...";
                        }
                        std::cout << std::dec << std::setfill(' ') << std::endl;
                    } else {
                        // Simple logging - just count
                        static int feedback_count = 0;
                        if (++feedback_count % 50 == 0) {
                            std::cout << "✓ Received " << feedback_count << " TCP messages" << std::endl;
                        }
                    }
                }
                else if (bytes_received == 0) {
                    // Connection closed by server
                    std::cout << "⚠️  TCP server closed connection" << std::endl;
                    tcp_config_.initialized = false;
                    close(tcp_config_.socket_fd);
                    tcp_config_.socket_fd = -1;
                    break;
                }
                else {
                    // Error occurred
                    perror("TCP recv error");
                    tcp_config_.initialized = false;
                    close(tcp_config_.socket_fd);
                    tcp_config_.socket_fd = -1;
                    break;
                }
            }
            else if (select_result == 0) {
                // Timeout occurred - periodically show we're still listening
                static int timeout_count = 0;
                if (++timeout_count % 10 == 0) {
                    std::cout << "⏱️  Still listening for TCP messages... (timeout #" << timeout_count << ")" << std::endl;
                }
            }
            else {
                // Error occurred
                perror("TCP select error");
                tcp_config_.initialized = false;
                break;
            }
        }

        std::cout << "🛑 TCP monitoring stopped" << std::endl;
    }
};

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: motor_controller <tcp_endpoint> [-v]" << std::endl;
        std::cout << "Example: ./motor_controller 127.0.0.1:8888          # Connect to local TCP server" << std::endl;
        std::cout << "         ./motor_controller 192.168.1.100:8888 -v   # Connect to remote with verbose" << std::endl;
        std::cout << "\nOptions:" << std::endl;
        std::cout << "  -v     Enable verbose logging (show all messages)" << std::endl;
        std::cout << "\nTCP endpoint format: <ip_address>:<port>" << std::endl;
        std::cout << "  Default IP: 127.0.0.1 (localhost)" << std::endl;
        std::cout << "  Default Port: 8888" << std::endl;
        return 1;
    }

    std::string tcp_endpoint = argv[1];

    // Check for command line flags
    bool verbose = false;

    for (int i = 2; i < argc; i++) {
        if (std::string(argv[i]) == "-v") {
            verbose = true;
        }
    }

    std::cout << "========================================" << std::endl;
    std::cout << "ROS2-to-TCP Motor Controller Bridge" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "ROS2 Topic: " << ROS2_CMD_TOPIC << std::endl;
    std::cout << "TCP Endpoint: " << tcp_endpoint << std::endl;
    std::cout << "Verbose Mode: " << (verbose ? "ON" : "OFF") << std::endl;
    std::cout << "Data Format: xixilowcmd/LowCmd" << std::endl;
    std::cout << "" << std::endl;

    // Initialize ROS2
    rclcpp::init(argc, argv);

    try {
        // Create and initialize bridge as shared pointer for rclcpp::spin
        auto bridge = std::make_shared<ROS2_to_TCP_Bridge>(verbose);

        // Initialize TCP connection first
        if (!bridge->InitializeTCP(tcp_endpoint)) {
            std::cerr << "Failed to initialize TCP connection to: " << tcp_endpoint << std::endl;
            std::cerr << "Make sure:" << std::endl;
            std::cerr << "  1. TCP server is running at the specified endpoint" << std::endl;
            std::cerr << "  2. IP address and port are correct" << std::endl;
            std::cerr << "  3. Network connectivity is available" << std::endl;
            rclcpp::shutdown();
            return 1;
        }

        // Initialize ROS2 subscriber
        bridge->InitializeROS2();

        std::cout << "Initialization complete!" << std::endl << std::endl;

        // Run the bridge (this will block until Ctrl+C)
        bridge->Run();

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    // Clean shutdown
    rclcpp::shutdown();
    return 0;
}