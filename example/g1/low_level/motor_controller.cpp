#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <map>
#include <array>
#include <ctime>
#include <iomanip>
#include <atomic>
#include <chrono>

// DDS
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

// IDL
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_STATE_TOPIC = "rt/lowstate";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

const int G1_NUM_MOTOR = 29;

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

// CAN interface configuration
struct CanConfig {
    std::string interface = "can0";
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

// Map G1 joint indices to CAN motor IDs
// Matches the actual CAN ID format seen in candump
std::map<int, int> g1_to_can_motor = {
    {4, 0x201},   // LeftAnklePitch -> CAN ID 0x201 (Motor 1)
    {5, 0x202},   // LeftAnkleRoll -> CAN ID 0x202 (Motor 2)
    {10, 0x203},  // RightAnklePitch -> CAN ID 0x203 (Motor 3)
    {11, 0x204}   // RightAnkleRoll -> CAN ID 0x204 (Motor 4)
};

class DDS_to_CAN_Bridge {
private:
    CanConfig can_config_;
    ChannelSubscriberPtr<LowCmd_> lowcmd_subscriber_;
    ChannelPublisherPtr<LowState_> lowstate_publisher_;

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
    DDS_to_CAN_Bridge() {
        // Initialize motor commands to zero
        for (int i = 0; i < G1_NUM_MOTOR; i++) {
            motor_commands_[i] = {0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        }

        std::cout << "DDS-to-CAN Bridge Initializing..." << std::endl;
    }

    ~DDS_to_CAN_Bridge() {
        // 停止500Hz发送线程
        should_stop_ = true;
        if (can_send_thread_.joinable()) {
            can_send_thread_.join();
        }

        if (can_config_.initialized && can_config_.socket_fd >= 0) {
            close(can_config_.socket_fd);
        }
    }

    bool InitializeCAN(const std::string& can_interface) {
        can_config_.interface = can_interface;

        // Check if CAN interface exists and is up
        struct ifreq ifr;
        strcpy(ifr.ifr_name, can_interface.c_str());
        int check_sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (ioctl(check_sock, SIOCGIFFLAGS, &ifr) < 0) {
            std::cerr << "CAN interface " << can_interface << " does not exist" << std::endl;
            close(check_sock);
            return false;
        }
        close(check_sock);

        if (!(ifr.ifr_flags & IFF_UP)) {
            std::cerr << "CAN interface " << can_interface << " is down" << std::endl;
            std::cerr << "Run: sudo ip link set " << can_interface << " up" << std::endl;
            return false;
        }

        std::cout << "CAN interface " << can_interface << " exists and is UP" << std::endl;

        // Create CAN socket
        can_config_.socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_config_.socket_fd < 0) {
            perror("Socket creation failed");
            return false;
        }

        // Set socket receive buffer size for better performance
        int rcvbuf_size = 1024 * 1024; // 1MB
        if (setsockopt(can_config_.socket_fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
            perror("Setting receive buffer size failed");
            close(can_config_.socket_fd);
            return false;
        }

        // Set up CAN interface
        strcpy(ifr.ifr_name, can_interface.c_str());
        if (ioctl(can_config_.socket_fd, SIOCGIFINDEX, &ifr) < 0) {
            perror("SIOCGIFINDEX failed");
            close(can_config_.socket_fd);
            return false;
        }

        // Bind socket to CAN interface
        struct sockaddr_can addr;
        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_config_.socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("Bind failed");
            close(can_config_.socket_fd);
            return false;
        }

        can_config_.initialized = true;
        std::cout << "CAN interface " << can_interface << " initialized successfully" << std::endl;

        // Test CAN by sending a test frame
        struct can_frame test_frame;
        test_frame.can_id = 0x7DF;  // OBD test ID (harmless)
        test_frame.can_dlc = 8;
        memset(test_frame.data, 0x00, 8);

        if (write(can_config_.socket_fd, &test_frame, sizeof(test_frame)) == sizeof(test_frame)) {
            std::cout << "✓ CAN interface test: Successfully sent test frame" << std::endl;
        } else {
            std::cout << "! CAN interface test: Failed to send test frame (may be normal if no physical connection)" << std::endl;
        }

        return true;
    }

    void InitializeDDS(const std::string& network_interface) {
        std::cout << "🔗 Initializing DDS on interface: " << network_interface << std::endl;

        // Initialize DDS channel
        ChannelFactory::Instance()->Init(0, network_interface);
        std::cout << "✓ DDS ChannelFactory initialized" << std::endl;

        // Subscribe to lowcmd topic
        lowcmd_subscriber_.reset(new ChannelSubscriber<LowCmd_>(HG_CMD_TOPIC));
        std::cout << "✓ LowCmd subscriber created" << std::endl;

        lowcmd_subscriber_->InitChannel(std::bind(&DDS_to_CAN_Bridge::LowCmdHandler, this, std::placeholders::_1), 1);
        std::cout << "✓ LowCmd subscriber initialized on topic: " << HG_CMD_TOPIC << std::endl;

        // Create publisher for lowstate (to send feedback)
        lowstate_publisher_.reset(new ChannelPublisher<LowState_>(HG_STATE_TOPIC));
        lowstate_publisher_->InitChannel();
        std::cout << "✓ LowState publisher initialized on topic: " << HG_STATE_TOPIC << std::endl;

        std::cout << "🎯 DDS initialization complete! Waiting for messages..." << std::endl;
    }

    void LowCmdHandler(const void *message) {
        static int dds_message_count = 0;
        dds_message_count++;

        const LowCmd_ *low_cmd = (const LowCmd_ *)message;
        auto current_time = std::chrono::high_resolution_clock::now();

        int motors_with_commands = 0;
        // Extract motor commands from DDS message
        for (int i = 0; i < G1_NUM_MOTOR; i++) {
            if (i < low_cmd->motor_cmd().size()) {
                const auto& motor_cmd = low_cmd->motor_cmd()[i];

                motor_commands_[i].pos = motor_cmd.q();
                motor_commands_[i].vel = motor_cmd.dq();
                motor_commands_[i].kp = motor_cmd.kp();
                motor_commands_[i].kd = motor_cmd.kd();
                motor_commands_[i].torq = motor_cmd.tau();

                // Check if this motor has a CAN mapping
                auto it = g1_to_can_motor.find(i);
                if (it != g1_to_can_motor.end()) {
                    motor_commands_[i].motor_id = it->second;

                    // 更新命令历史用于插值
                    auto& history = command_history_[i];

                    // 保存上一个命令
                    if (history.has_previous) {
                        history.previous = history.current;
                        history.previous_timestamp = history.current_timestamp;
                    } else {
                        // 第一个命令，创建一个拷贝作为上一个命令
                        history.previous = motor_commands_[i];
                        history.previous_timestamp = current_time;
                        history.has_previous = true;
                    }

                    // 更新当前命令
                    history.current = motor_commands_[i];
                    history.current_timestamp = current_time;

                    motors_with_commands++;
                }
            }
        }

        // 减少调试输出频率
        if (dds_message_count % 50 == 0) {
            std::cout << "📥 DDS Messages: " << dds_message_count
                     << " | Motors updated: " << motors_with_commands << std::endl;

            // 显示插值状态
            std::cout << "🔄 Interpolation status: ";
            for (auto const& [g1_joint, can_motor] : g1_to_can_motor) {
                auto& history = command_history_[g1_joint];
                std::cout << "G1[" << g1_joint << "] ";
                if (history.has_previous) {
                    auto time_diff = std::chrono::duration<double>(history.current_timestamp - history.previous_timestamp).count();
                    std::cout << time_diff*1000 << "ms ";
                } else {
                    std::cout << "init ";
                }
            }
            std::cout << std::endl;
        }
    }

    void SendMotorCommandCAN(const MotorCommandCan& cmd) {
        if (!can_config_.initialized) {
            std::cerr << "CAN not initialized, skipping command" << std::endl;
            return;
        }

        // Convert motor command to CAN frame (MIT mode protocol)
        // This matches the STM32 motor controller protocol
        struct can_frame frame;
        frame.can_id = cmd.motor_id;  // Use the actual CAN ID (0x201-0x204)
        frame.can_dlc = 8;  // 8 bytes data

        // Convert float values to motor protocol format
        // Note: This conversion needs to match the STM32 implementation
        int16_t p_int = static_cast<int16_t>(cmd.pos * 32767.0f / 12.5f);  // Scale to motor range
        int16_t v_int = static_cast<int16_t>(cmd.vel * 32767.0f / 30.0f);
        int16_t t_int = static_cast<int16_t>(cmd.torq * 32767.0f / 10.0f);
        int16_t kp_int = static_cast<int16_t>(cmd.kp * 32767.0f / 500.0f);
        int16_t kd_int = static_cast<int16_t>(cmd.kd * 32767.0f / 5.0f);

        // Pack data according to MIT mode protocol (8 bytes)
        frame.data[0] = static_cast<uint8_t>(p_int & 0xFF);
        frame.data[1] = static_cast<uint8_t>((p_int >> 8) & 0xFF);
        frame.data[2] = static_cast<uint8_t>(v_int & 0xFF);
        frame.data[3] = static_cast<uint8_t>((v_int >> 8) & 0xFF);
        frame.data[4] = static_cast<uint8_t>(t_int & 0xFF);
        frame.data[5] = static_cast<uint8_t>((t_int >> 8) & 0xFF);
        frame.data[6] = static_cast<uint8_t>(kp_int & 0xFF);
        frame.data[7] = static_cast<uint8_t>((kp_int >> 8) & 0xFF);

        // Send CAN frame
        ssize_t bytes_sent = write(can_config_.socket_fd, &frame, sizeof(frame));
        if (bytes_sent != sizeof(frame)) {
            std::cerr << "✗ Error sending CAN frame for motor " << cmd.motor_id
                     << " (sent " << bytes_sent << " bytes, expected " << sizeof(frame) << ")" << std::endl;
            if (bytes_sent < 0) {
                perror("CAN write error");
            }
        } else {
            // Enhanced logging with CAN frame details
            static int counter = 0;
            static time_t last_log_time = 0;
            time_t current_time = time(nullptr);

            counter++;

            // Log first command, every 100th command, or every 5 seconds
            if (counter == 1 || counter % 100 == 0 || (current_time - last_log_time) >= 5) {
                std::cout << std::dec << "[" << counter << "] ✓ CAN -> Motor " << cmd.motor_id
                         << " | ID: 0x" << std::hex << frame.can_id << std::dec
                         << " | pos=" << std::fixed << std::setprecision(4) << cmd.pos
                         << " rad (" << p_int << ")"
                         << " | kp=" << cmd.kp
                         << " | kd=" << cmd.kd
                         << " | torq=" << cmd.torq << std::endl;

                // Show raw CAN data for debugging
                std::cout << "    Raw: ";
                for (int i = 0; i < frame.can_dlc; i++) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0')
                             << (int)frame.data[i] << " ";
                }
                std::cout << std::dec << std::setfill(' ') << std::endl;

                last_log_time = current_time;
            }
        }
    }

    void PublishSimulatedState() {
        if (!lowstate_publisher_) return;

        // Create simulated low state (since we can't read actual motor feedback)
        LowState_ low_state;
        low_state.mode_machine() = 1;  // G1 type

        // Initialize motor states with some simulated values
        for (int i = 0; i < G1_NUM_MOTOR; i++) {
            auto& motor_state = low_state.motor_state()[i];
            motor_state.mode() = 0;
            motor_state.q() = motor_commands_[i].pos;  // Echo command as state
            motor_state.dq() = 0.0f;
            motor_state.ddq() = 0.0f;
            motor_state.tau_est() = motor_commands_[i].torq;
            motor_state.temperature()[0] = 25.0f;
            motor_state.temperature()[1] = 25.0f;
            motor_state.motorstate() = 0;  // No error
        }

        // Set some initial positions for realistic simulation
        low_state.motor_state()[4].q() = 0.0;    // LeftAnklePitch
        low_state.motor_state()[5].q() = 0.0;    // LeftAnkleRoll
        low_state.motor_state()[10].q() = 0.0;   // RightAnklePitch
        low_state.motor_state()[11].q() = 0.0;   // RightAnkleRoll

        // Simulate IMU data
        auto& imu = low_state.imu_state();
        imu.rpy()[0] = 0.0f;  // roll
        imu.rpy()[1] = 0.0f;  // pitch
        imu.rpy()[2] = 0.0f;  // yaw
        imu.gyroscope()[0] = 0.0f;
        imu.gyroscope()[1] = 0.0f;
        imu.gyroscope()[2] = 0.0f;
        imu.accelerometer()[0] = 0.0f;
        imu.accelerometer()[1] = 0.0f;
        imu.accelerometer()[2] = 9.81f;

        // Calculate CRC using the same method as the original
        low_state.crc() = Crc32Core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1);

        // Publish state
        lowstate_publisher_->Write(low_state);
    }

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

    // 500Hz CAN发送线程
    void CANSendThread() {
        std::cout << "🔄 500Hz CAN发送线程启动" << std::endl;

        const auto interval = std::chrono::milliseconds(2); // 500Hz = 2ms
        auto next_send_time = std::chrono::high_resolution_clock::now();

        while (!should_stop_) {
            auto now = std::chrono::high_resolution_clock::now();

            // 检查是否到了发送时间
            if (now >= next_send_time) {
                // 为每个有CAN映射的电机发送插值命令
                for (auto const& [g1_joint, can_motor] : g1_to_can_motor) {
                    if (g1_joint < G1_NUM_MOTOR) {
                        auto& history = command_history_[g1_joint];
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

                        cmd_to_send.motor_id = can_motor;
                        SendMotorCommandCAN(cmd_to_send);
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

        std::cout << "🛑 500Hz CAN发送线程停止" << std::endl;
    }

    void Run() {
        std::cout << "\nDDS-to-CAN Bridge Running..." << std::endl;
        std::cout << "⚡ 500Hz CAN发送已启用 (带线性插值)" << std::endl;
        std::cout << "Waiting for DDS commands on topic: " << HG_CMD_TOPIC << std::endl;
        std::cout << "Sending CAN commands on interface: " << can_config_.interface << std::endl;
        std::cout << "Press Ctrl+C to stop" << std::endl << std::endl;

        // 启动500Hz CAN发送线程
        can_send_thread_ = std::thread(&DDS_to_CAN_Bridge::CANSendThread, this);

        // 启动CAN监控线程
        std::thread can_monitor_thread(&DDS_to_CAN_Bridge::MonitorCAN, this);
        can_monitor_thread.detach();

        // Main loop - publish simulated state periodically
        while (true) {
            PublishSimulatedState();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));  // 500Hz
        }
    }

    void MonitorCAN() {
        if (!can_config_.initialized) {
            return;
        }

        std::cout << "📡 CAN monitoring started on " << can_config_.interface << std::endl;
        std::cout << "   Listening for motor feedback (IDs 0x11-0x1F)" << std::endl;

        struct can_frame frame;
        int can_rx_count = 0;

        while (can_config_.initialized) {
            // Try to read CAN frame with timeout
            fd_set read_fds;
            struct timeval timeout;
            FD_ZERO(&read_fds);
            FD_SET(can_config_.socket_fd, &read_fds);

            timeout.tv_sec = 1;  // 1 second timeout
            timeout.tv_usec = 0;

            int select_result = select(can_config_.socket_fd + 1, &read_fds, NULL, NULL, &timeout);

            if (select_result > 0 && FD_ISSET(can_config_.socket_fd, &read_fds)) {
                ssize_t bytes_received = read(can_config_.socket_fd, &frame, sizeof(frame));

                if (bytes_received == sizeof(frame)) {
                    can_rx_count++;

                    // Log motor feedback frames (typical feedback IDs are 0x11-0x1F)
                    if (frame.can_id >= 0x11 && frame.can_id <= 0x1F) {
                        int motor_id = frame.can_id - 0x10;
                        std::cout << "📨 Received feedback: Motor " << motor_id
                                 << " | ID: 0x" << std::hex << frame.can_id << std::dec
                                 << " | Len: " << (int)frame.can_dlc << std::endl;

                        // Show raw data
                        std::cout << "    Data: ";
                        for (int i = 0; i < frame.can_dlc && i < 8; i++) {
                            std::cout << std::hex << std::setw(2) << std::setfill('0')
                                     << (int)frame.data[i] << " ";
                        }
                        std::cout << std::dec << std::setfill(' ') << std::endl;
                    }
                    else {
                        // Log other CAN frames
                        static int other_count = 0;
                        if (++other_count % 10 == 0) {
                            std::cout << "📨 Other CAN frames received: " << other_count << std::endl;
                        }
                    }
                }
            }
            // Timeout - continue loop
        }
    }
};

int main(int argc, char const *argv[]) {
    if (argc < 3) {
        std::cout << "Usage: motor_controller <dds_network_interface> <can_interface>" << std::endl;
        std::cout << "Example: ./motor_controller lo can0" << std::endl;
        std::cout << "         ./motor_controller eth0 can1" << std::endl;
        return 1;
    }

    std::string dds_interface = argv[1];
    std::string can_interface = argv[2];

    std::cout << "========================================" << std::endl;
    std::cout << "DDS-to-CAN Motor Controller Bridge" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "DDS Interface: " << dds_interface << std::endl;
    std::cout << "CAN Interface: " << can_interface << std::endl;
    std::cout << "" << std::endl;

    // Create and initialize bridge
    DDS_to_CAN_Bridge bridge;

    // Initialize CAN first
    if (!bridge.InitializeCAN(can_interface)) {
        std::cerr << "Failed to initialize CAN interface: " << can_interface << std::endl;
        std::cerr << "Make sure:" << std::endl;
        std::cerr << "  1. CAN interface exists: sudo ip link show " << can_interface << std::endl;
        std::cerr << "  2. Interface is up: sudo ip link set " << can_interface << " up" << std::endl;
        std::cerr << "  3. You have permission to access raw sockets" << std::endl;
        return 1;
    }

    // Initialize DDS
    bridge.InitializeDDS(dds_interface);

    std::cout << "Initialization complete!" << std::endl << std::endl;

    try {
        bridge.Run();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}