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
#include <mutex>
#include <condition_variable>
#include <vector>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include "xixilowcmd/xixilowcmd/msg/low_cmd.hpp"
#include "xixilowcmd/xixilowcmd/msg/motor_cmd.hpp"

static const std::string ROS2_CMD_TOPIC = "/lowcmd";
const int G1_NUM_MOTOR = 30;

// CRC32 calculation function
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
    std::string remote_ip = "127.0.0.1";
    int remote_port = 8888;
    int socket_fd = -1;
    bool initialized = false;
};

// 周立功设备配置 (使用UDP)
struct ZhilgongConfig {
    std::string remote_ip = "192.168.1.5";  // 周立功设备IP
    int remote_port = 8002;                // 周立功设备UDP端口
    int socket_fd = -1;
    bool initialized = false;
};

// Motor command structure
struct MotorCommandCan {
    uint16_t motor_id;
    float pos;
    float vel;
    float kp;
    float kd;
    float torq;
};

// ==================== ZLG 网络包格式 (per zlg_desc.txt) ====================

// ZLG CAN Network Packet Format (PC -> CANFDNET)
// Network packet example: 55 00 00 00 00 18 00 00 00 00 00 00 00 00 00 00 07 ff 00 08 00 08 01 02 03 04 05 06 07 08 bd
struct ZlgCanNetPacket {
    uint8_t header;                    // 0x55 - packet header
    uint8_t type[3];                   // 0x00 0x00 0x00 - CAN message type + reserved
    uint8_t length[2];                 // Data length (little-endian)
    uint8_t timestamp[8];              // 8 bytes timestamp (zeros when sending)
    uint8_t can_id[4];                 // CAN Frame ID (little-endian)
    uint8_t frame_info[2];             // Frame info (0x00 0x00 for standard frame without echo)
    uint8_t channel;                   // CAN channel (0x00 = CAN0)
    uint8_t dlc;                       // Data length code (0-8)
    uint8_t data[8];                   // CAN data bytes
    uint8_t checksum;                  // XOR checksum of all bytes except checksum
} __attribute__((packed));

// Helper class for building ZLG CAN network packets
class ZlgPacketBuilder {
public:
    // Build ZLG CAN network packet for transmission
    // per zlg_desc.txt format
    // Example output: 55 00 00 00 00 18 00 00 00 00 00 00 00 00 00 00 07 ff 00 08 00 08 01 02 03 04 05 06 07 08 bd
    static std::vector<uint8_t> BuildCanPacket(uint32_t can_id, const uint8_t* data, uint8_t dlc, uint8_t channel = 0) {
        std::vector<uint8_t> packet;
        packet.reserve(27);  // Total packet size: 27 bytes

        // 1. Header: 0x55
        packet.push_back(0x55);

        // 2. Type: 0x00 0x00 0x00 (CAN message type, reserved)
        packet.push_back(0x00);
        packet.push_back(0x00);
        packet.push_back(0x00);

        // 3. Data length: 0x18 (24 bytes for CAN frame)
        // This is the length of the data field after the length field
        // Includes: timestamp(8) + can_id(4) + frame_info(2) + channel(1) + dlc(1) + data(8) = 24 bytes
        uint16_t data_length = 0x18;
        packet.push_back(data_length & 0xFF);         // Little-endian low byte
        packet.push_back((data_length >> 8) & 0xFF);  // Little-endian high byte

        // 4. Timestamp: 8 bytes (all zeros when sending)
        for (int i = 0; i < 8; i++) {
            packet.push_back(0x00);
        }

        // 5. CAN ID: 4 bytes (little-endian)
        packet.push_back(can_id & 0xFF);
        packet.push_back((can_id >> 8) & 0xFF);
        packet.push_back((can_id >> 16) & 0xFF);
        packet.push_back((can_id >> 24) & 0xFF);

        // 6. Frame info: 0x00 0x00 (standard frame, no echo)
        // bit3=1 enables echo, bit2 is valid for receive direction
        packet.push_back(0x00);
        packet.push_back(0x00);

        // 7. Channel (0x00 = CAN0)
        packet.push_back(channel);

        // 8. DLC (Data Length Code)
        packet.push_back(dlc);

        // 9. Data bytes (up to 8 bytes)
        for (uint8_t i = 0; i < dlc && i < 8; i++) {
            packet.push_back(data[i]);
        }
        // Pad remaining data bytes with zeros if dlc < 8
        for (uint8_t i = dlc; i < 8; i++) {
            packet.push_back(0x00);
        }

        // 10. Calculate XOR checksum (XOR of all bytes after header, i.e., from index 1 to end)
        uint8_t checksum = 0;
        for (size_t i = 1; i < packet.size(); i++) {
            checksum ^= packet[i];
        }
        packet.push_back(checksum);

        return packet;
    }

    // Debug: Print packet in hex format (like zlg_desc.txt examples)
    static void PrintPacket(const std::vector<uint8_t>& packet, const std::string& label = "ZLG Packet") {
        std::cout << label << " [" << packet.size() << " bytes]: ";
        for (size_t i = 0; i < packet.size(); i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(packet[i]) << " ";
        }
        std::cout << std::dec << std::setfill(' ') << std::endl;
    }

    // Parse received packet (for debugging/monitoring)
    static void ParseReceivedPacket(const uint8_t* data, size_t len) {
        if (len < 27 || data[0] != 0x55) {
            std::cout << "Invalid ZLG packet" << std::endl;
            return;
        }

        std::cout << "=== ZLG Received Packet ===" << std::endl;
        std::cout << "Header: 0x" << std::hex << static_cast<int>(data[0]) << std::dec << std::endl;

        // Type
        std::cout << "Type: ";
        for (int i = 1; i <= 3; i++) std::cout << std::hex << static_cast<int>(data[i]) << " ";
        std::cout << std::dec << std::endl;

        // Length
        uint16_t data_len = data[4] | (data[5] << 8);
        std::cout << "Data Length: 0x" << std::hex << data_len << std::dec << std::endl;

        // Timestamp (received packets have valid timestamp)
        std::cout << "Timestamp: ";
        for (int i = 6; i < 14; i++) std::cout << std::hex << static_cast<int>(data[i]) << " ";
        std::cout << std::dec << std::endl;

        // CAN ID
        uint32_t can_id = data[14] | (data[15] << 8) | (data[16] << 16) | (data[17] << 24);
        std::cout << "CAN ID: 0x" << std::hex << can_id << std::dec << std::endl;

        // Frame info
        std::cout << "Frame Info: ";
        for (int i = 18; i < 20; i++) std::cout << std::hex << static_cast<int>(data[i]) << " ";
        std::cout << std::dec << std::endl;

        // Channel
        std::cout << "Channel: " << static_cast<int>(data[20]) << std::endl;

        // DLC
        uint8_t dlc = data[21];
        std::cout << "DLC: " << static_cast<int>(dlc) << std::endl;

        // Data
        std::cout << "Data: ";
        for (int i = 0; i < dlc && i < 8; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(data[22 + i]) << " ";
        }
        std::cout << std::dec << std::setfill(' ') << std::endl;

        // Checksum
        std::cout << "Checksum: 0x" << std::hex << static_cast<int>(data[26]) << std::dec << std::endl;

        // Verify checksum
        uint8_t calc_checksum = 0;
        for (size_t i = 1; i < 26; i++) {
            calc_checksum ^= data[i];
        }
        std::cout << "Checksum Valid: " << (calc_checksum == data[26] ? "YES" : "NO") << std::endl;
        std::cout << "=========================" << std::endl;
    }
};

// Motor ID mapping function
int get_can_id_for_motor(int motor_id) {
    if (motor_id >= 1 && motor_id <= 30) {
        return motor_id;
    }
    return 0;
}

class ROS2_to_TCP_Bridge : public rclcpp::Node {
private:
    TcpConfig tcp_config_;
    ZhilgongConfig zhilgong_config_;

    struct MotorEnableMsg {
        int motor_id;
        uint8_t command;
    };

    rclcpp::Subscription<xixilowcmd::msg::LowCmd>::SharedPtr lowcmd_subscriber_;
    rclcpp::Subscription<xixilowcmd::msg::MotorCmd>::SharedPtr enable_subscriber_;

    bool verbose_logging_ = false;
    bool motor_cmd_enabled_ = false;

    std::array<MotorCommandCan, G1_NUM_MOTOR> motor_commands_;

    std::thread can_send_thread_;
    std::atomic<bool> should_stop_{false};

    std::atomic<bool> enable_command_active_{false};
    std::atomic<bool> can_send_started_{false};
    std::mutex pause_mutex_;
    std::condition_variable pause_cv_;

    struct MotorCommandHistory {
        MotorCommandCan current;
        MotorCommandCan previous;
        bool has_previous = false;
        std::chrono::high_resolution_clock::time_point current_timestamp;
        std::chrono::high_resolution_clock::time_point previous_timestamp;
    };
    std::array<MotorCommandHistory, G1_NUM_MOTOR> command_history_;

public:
    ROS2_to_TCP_Bridge(bool verbose = false, bool motor_cmd_enabled = false) : Node("ros2_to_tcp_bridge") {
        verbose_logging_ = verbose;
        motor_cmd_enabled_ = motor_cmd_enabled;

        for (int i = 0; i < G1_NUM_MOTOR; i++) {
            motor_commands_[i] = {static_cast<uint16_t>(i), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        }

        std::cout << "ROS2-to-TCP Bridge Initializing..." << std::endl;
        std::cout << "  Verbose logging: " << (verbose_logging_ ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "  Motor command: " << (motor_cmd_enabled_ ? "ENABLED" : "DISABLED") << std::endl;
    }

    ~ROS2_to_TCP_Bridge() {
        should_stop_ = true;
        if (can_send_thread_.joinable()) {
            can_send_thread_.join();
        }

        if (tcp_config_.initialized && tcp_config_.socket_fd >= 0) {
            close(tcp_config_.socket_fd);
        }

        CloseZhilgongConnection();
    }

    bool InitializeTCP(const std::string& tcp_endpoint) {
        size_t colon_pos = tcp_endpoint.find(':');
        if (colon_pos != std::string::npos) {
            tcp_config_.remote_ip = tcp_endpoint.substr(0, colon_pos);
            tcp_config_.remote_port = std::stoi(tcp_endpoint.substr(colon_pos + 1));
        }

        std::cout << "Initializing TCP connection to " << tcp_config_.remote_ip
                  << ":" << tcp_config_.remote_port << std::endl;

        tcp_config_.socket_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (tcp_config_.socket_fd < 0) {
            perror("TCP socket creation failed");
            return false;
        }

        int opt = 1;
        setsockopt(tcp_config_.socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(tcp_config_.remote_port);

        if (inet_pton(AF_INET, tcp_config_.remote_ip.c_str(), &server_addr.sin_addr) <= 0) {
            std::cerr << "Invalid IP address: " << tcp_config_.remote_ip << std::endl;
            close(tcp_config_.socket_fd);
            return false;
        }

        std::cout << "Connecting to TCP server..." << std::endl;
        if (connect(tcp_config_.socket_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            perror("TCP connection failed");
            close(tcp_config_.socket_fd);
            return false;
        }

        tcp_config_.initialized = true;
        std::cout << "TCP connection established successfully" << std::endl;
        return true;
    }

    // Initialize ZLG connection using TCP
    bool InitializeZhilgongConnection() {
        if (zhilgong_config_.initialized) {
            return true;
        }

        std::cout << "Initializing ZLG TCP connection to " << zhilgong_config_.remote_ip
                  << ":" << zhilgong_config_.remote_port << std::endl;

        // Create TCP socket
        zhilgong_config_.socket_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (zhilgong_config_.socket_fd < 0) {
            perror("TCP socket creation failed");
            return false;
        }

        int opt = 1;
        setsockopt(zhilgong_config_.socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        // Connect to ZLG device
        struct sockaddr_in zlg_addr;
        memset(&zlg_addr, 0, sizeof(zlg_addr));
        zlg_addr.sin_family = AF_INET;
        zlg_addr.sin_port = htons(zhilgong_config_.remote_port);

        if (inet_pton(AF_INET, zhilgong_config_.remote_ip.c_str(), &zlg_addr.sin_addr) <= 0) {
            std::cerr << "Invalid ZLG IP address: " << zhilgong_config_.remote_ip << std::endl;
            close(zhilgong_config_.socket_fd);
            return false;
        }

        if (connect(zhilgong_config_.socket_fd, (struct sockaddr*)&zlg_addr, sizeof(zlg_addr)) < 0) {
            std::cerr << "ZLG TCP connection failed: " << strerror(errno) << std::endl;
            close(zhilgong_config_.socket_fd);
            return false;
        }

        zhilgong_config_.initialized = true;
        std::cout << "ZLG TCP connection established successfully" << std::endl;
        std::cout << "  ZLG IP: " << zhilgong_config_.remote_ip << std::endl;
        std::cout << "  ZLG Port: " << zhilgong_config_.remote_port << std::endl;
        return true;
    }

    void InitializeROS2() {
        std::cout << "Initializing ROS2 subscriber on topic: " << ROS2_CMD_TOPIC << std::endl;

        lowcmd_subscriber_ = this->create_subscription<xixilowcmd::msg::LowCmd>(
            ROS2_CMD_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            std::bind(&ROS2_to_TCP_Bridge::LowCmdHandler, this, std::placeholders::_1)
        );

        enable_subscriber_ = this->create_subscription<xixilowcmd::msg::MotorCmd>(
            "/motor_enable",
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            std::bind(&ROS2_to_TCP_Bridge::EnableCommandHandler, this, std::placeholders::_1)
        );

        std::cout << "ROS2 subscribers created successfully" << std::endl;
    }

    void LowCmdHandler(const xixilowcmd::msg::LowCmd::SharedPtr msg) {
        auto current_time = std::chrono::high_resolution_clock::now();

        for (const auto& motor_cmd : msg->motor_cmd) {
            int motor_id = motor_cmd.id;

            if (motor_id >= 0 && motor_id < G1_NUM_MOTOR) {
                motor_commands_[motor_id].motor_id = motor_id;
                motor_commands_[motor_id].pos = motor_cmd.q;
                motor_commands_[motor_id].vel = motor_cmd.dq;
                motor_commands_[motor_id].kp = motor_cmd.kp;
                motor_commands_[motor_id].kd = motor_cmd.kd;
                motor_commands_[motor_id].torq = motor_cmd.tau;

                int can_id = get_can_id_for_motor(motor_id);
                if (can_id > 0) {
                    motor_commands_[motor_id].motor_id = can_id;

                    auto& history = command_history_[motor_id];

                    if (history.has_previous) {
                        history.previous = history.current;
                        history.previous_timestamp = history.current_timestamp;
                    } else {
                        history.previous = motor_commands_[motor_id];
                        history.previous_timestamp = current_time;
                        history.has_previous = true;
                    }

                    history.current = motor_commands_[motor_id];
                    history.current_timestamp = current_time;
                }
            }
        }

        if (verbose_logging_) {
            static int count = 0;
            if (++count % 100 == 0) {
                std::cout << "Received " << count << " ROS2 messages" << std::endl;
            }
        }
    }

    void EnableCommandHandler(const xixilowcmd::msg::MotorCmd::SharedPtr msg) {
        std::cout << "\n=== Enable Command Received ===" << std::endl;
        std::cout << "Motor ID: " << static_cast<int>(msg->id) << std::endl;
        std::cout << "Command (q value): " << msg->q << std::endl;

        int motor_id = msg->id;
        int command_value = static_cast<int>(msg->q);

        if (motor_id < 0 || motor_id > 29) {
            std::cerr << "Invalid motor ID: " << motor_id << " (range: 0-29)" << std::endl;
            return;
        }

        if (command_value == 0) {
            std::cout << "Command value is 0, ignoring" << std::endl;
            return;
        }

        {
            std::unique_lock<std::mutex> lock(pause_mutex_);
            enable_command_active_ = true;
        }

        if (command_value == 1) {
            SendEnableCommandToZhilgong(motor_id);
        } else if (command_value == 0) {
            SendDisableCommandToZhilgong(motor_id);
        } else {
            SendCustomEnableCommandToZhilgong(motor_id, static_cast<uint8_t>(command_value));
        }

        {
            std::unique_lock<std::mutex> lock(pause_mutex_);
            enable_command_active_ = false;
            can_send_started_ = true;
        }
        pause_cv_.notify_one();
    }

    // Send enable command using ZLG network packet format
    void SendEnableCommandToZhilgong(int motor_id) {
        if (!InitializeZhilgongConnection()) {
            return;
        }

        int can_id = get_can_id_for_motor(motor_id);
        if (can_id <= 0) {
            std::cerr << "Invalid motor ID: " << motor_id << std::endl;
            return;
        }

        // Enable frame: FF FF FF FF FF FF FF FC
        uint8_t enable_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

        std::cout << "\n>>> Sending ENABLE Command via ZLG Network Packet <<<" << std::endl;
        std::cout << "  Motor ID: " << motor_id << std::endl;
        std::cout << "  CAN ID:   0x" << std::hex << can_id << std::dec << std::endl;
        std::cout << "  Command:  ENABLE (0xFC)" << std::endl;
        std::cout << "  Data:     FF FF FF FF FF FF FF FC" << std::endl;

        SendZlgNetworkPacket(can_id, enable_frame, sizeof(enable_frame));
        std::cout << ">>> Enable command sent <<<" << std::endl;
    }

    // Send disable command using ZLG network packet format
    void SendDisableCommandToZhilgong(int motor_id) {
        if (!InitializeZhilgongConnection()) {
            return;
        }

        int can_id = get_can_id_for_motor(motor_id);
        if (can_id <= 0) {
            std::cerr << "Invalid motor ID: " << motor_id << std::endl;
            return;
        }

        // Disable frame: FF FF FF FF FF FF FF FD
        uint8_t disable_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

        std::cout << "\n>>> Sending DISABLE Command via ZLG Network Packet <<<" << std::endl;
        std::cout << "  Motor ID: " << motor_id << std::endl;
        std::cout << "  CAN ID:   0x" << std::hex << can_id << std::dec << std::endl;
        std::cout << "  Command:  DISABLE (0xFD)" << std::endl;
        std::cout << "  Data:     FF FF FF FF FF FF FF FD" << std::endl;

        SendZlgNetworkPacket(can_id, disable_frame, sizeof(disable_frame));
        std::cout << ">>> Disable command sent <<<" << std::endl;
    }

    // Send custom enable command using ZLG network packet format
    void SendCustomEnableCommandToZhilgong(int motor_id, uint8_t command) {
        if (!InitializeZhilgongConnection()) {
            return;
        }

        int can_id = get_can_id_for_motor(motor_id);
        if (can_id <= 0) {
            std::cerr << "Invalid motor ID: " << motor_id << std::endl;
            return;
        }

        // Custom frame: FF FF FF FF FF FF FF [command]
        uint8_t custom_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command};

        std::cout << "\n>>> Sending CUSTOM Command via ZLG Network Packet <<<" << std::endl;
        std::cout << "  Motor ID: " << motor_id << std::endl;
        std::cout << "  CAN ID:   0x" << std::hex << can_id << std::dec << std::endl;
        std::cout << "  Command:  0x" << std::hex << static_cast<int>(command) << std::dec << std::endl;
        std::cout << "  Data:     FF FF FF FF FF FF FF " << std::hex << static_cast<int>(command) << std::dec << std::endl;

        SendZlgNetworkPacket(can_id, custom_frame, sizeof(custom_frame));
        std::cout << ">>> Custom command sent <<<" << std::endl;
    }

    // Send ZLG network packet format (per zlg_desc.txt) via TCP
    // Example: 55 00 00 00 00 18 00 00 00 00 00 00 00 00 00 00 07 ff 00 08 00 08 01 02 03 04 05 06 07 08 bd
    void SendZlgNetworkPacket(int can_id, const uint8_t* data, size_t data_len) {
        if (!zhilgong_config_.initialized || zhilgong_config_.socket_fd < 0) {
            std::cerr << "ZLG device not connected" << std::endl;
            return;
        }

        // Build ZLG CAN network packet
        std::vector<uint8_t> zlg_packet = ZlgPacketBuilder::BuildCanPacket(
            static_cast<uint32_t>(can_id), data, static_cast<uint8_t>(data_len), 0  // CAN0
        );

        // Print raw packet bytes in hex format (like: 55 00 00 00 00 18 00 ...)
        std::cout << "  Raw ZLG Packet [" << zlg_packet.size() << " bytes]: ";
        for (size_t i = 0; i < zlg_packet.size(); i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(zlg_packet[i]) << " ";
        }
        std::cout << std::dec << std::setfill(' ') << std::endl;

        // Send via TCP to ZLG device (connection already established)
        ssize_t sent = send(zhilgong_config_.socket_fd,
                            zlg_packet.data(),
                            zlg_packet.size(),
                            0);

        if (sent < 0) {
            std::cerr << "Failed to send ZLG packet: " << strerror(errno) << std::endl;
        } else if (sent != static_cast<ssize_t>(zlg_packet.size())) {
            std::cerr << "Partial send: " << sent << "/" << zlg_packet.size() << " bytes" << std::endl;
        } else {
            std::cout << "  Sent " << sent << " bytes to ZLG device" << std::endl;
        }
    }

    void CloseZhilgongConnection() {
        if (zhilgong_config_.initialized && zhilgong_config_.socket_fd >= 0) {
            close(zhilgong_config_.socket_fd);
            zhilgong_config_.socket_fd = -1;
            zhilgong_config_.initialized = false;
            std::cout << "ZLG connection closed" << std::endl;
        }
    }

    void SendMotorCommandTCP(const MotorCommandCan& cmd) {
        if (!tcp_config_.initialized) {
            return;
        }

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
        packet.header = 0xDEADBEEF;
        packet.motor_id = cmd.motor_id;
        packet.pos = cmd.pos;
        packet.vel = cmd.vel;
        packet.kp = cmd.kp;
        packet.kd = cmd.kd;
        packet.torq = cmd.torq;
        packet.crc32 = Crc32Core((uint32_t*)&packet, sizeof(packet) / sizeof(uint32_t) - 1);

        send(tcp_config_.socket_fd, &packet, sizeof(packet), MSG_NOSIGNAL);
    }

    MotorCommandCan interpolateCommand(const MotorCommandCan& prev, const MotorCommandCan& curr,
                                       double prev_time, double curr_time, double target_time) {
        MotorCommandCan result = curr;

        if (curr_time > prev_time) {
            double t = (target_time - prev_time) / (curr_time - prev_time);
            t = std::max(0.0, std::min(1.0, t));

            result.pos = prev.pos + t * (curr.pos - prev.pos);
            result.vel = prev.vel + t * (curr.vel - prev.vel);
            result.kp = prev.kp + t * (curr.kp - prev.kp);
            result.kd = prev.kd + t * (curr.kd - prev.kd);
            result.torq = prev.torq + t * (curr.torq - prev.torq);
        }

        return result;
    }

    void TCPSendThread() {
        std::cout << "500Hz TCP send thread started, waiting for start signal..." << std::endl;

        const auto interval = std::chrono::milliseconds(2);
        auto next_send_time = std::chrono::high_resolution_clock::now();

        while (!should_stop_) {
            std::unique_lock<std::mutex> lock(pause_mutex_);

            pause_cv_.wait(lock, [this] {
                return (can_send_started_ && !enable_command_active_) || should_stop_;
            });

            lock.unlock();

            if (should_stop_) {
                break;
            }

            auto now = std::chrono::high_resolution_clock::now();

            if (now >= next_send_time) {
                if (motor_cmd_enabled_) {
                    for (int motor_id = 1; motor_id <= 30; motor_id++) {
                        int can_id = get_can_id_for_motor(motor_id);
                        if (can_id > 0) {
                            auto& history = command_history_[motor_id];
                            auto current_time = std::chrono::duration<double>(now.time_since_epoch()).count();

                            MotorCommandCan cmd_to_send;

                            if (history.has_previous) {
                                double prev_time = std::chrono::duration<double>(history.previous_timestamp.time_since_epoch()).count();
                                double curr_time = std::chrono::duration<double>(history.current_timestamp.time_since_epoch()).count();

                                cmd_to_send = interpolateCommand(history.previous, history.current,
                                                               prev_time, curr_time, current_time);
                            } else {
                                cmd_to_send = history.current;
                            }

                            cmd_to_send.motor_id = can_id;
                            SendMotorCommandTCP(cmd_to_send);
                        }
                    }
                }

                next_send_time += interval;

                if (now > next_send_time + interval) {
                    next_send_time = now + interval;
                }
            }

            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        std::cout << "500Hz TCP send thread stopped" << std::endl;
    }

    void Run() {
        std::cout << "\nROS2-to-TCP Bridge Running..." << std::endl;
        std::cout << "Motor command: " << (motor_cmd_enabled_ ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "Waiting for ROS2 commands on topic: " << ROS2_CMD_TOPIC << std::endl;
        std::cout << "Press Ctrl+C to stop" << std::endl << std::endl;

        can_send_thread_ = std::thread(&ROS2_to_TCP_Bridge::TCPSendThread, this);

        rclcpp::spin(shared_from_this());
    }
};

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: motor_controller_with_enable <tcp_endpoint> [options]" << std::endl;
        std::cout << "Example: ./motor_controller_with_enable 127.0.0.1:8888" << std::endl;
        std::cout << "         ./motor_controller_with_enable 127.0.0.1:8888 -v" << std::endl;
        std::cout << "         ./motor_controller_with_enable 127.0.0.1:8888 --enable-motor-cmd" << std::endl;
        std::cout << "\nOptions:" << std::endl;
        std::cout << "  -v                 Enable verbose logging" << std::endl;
        std::cout << "  --enable-motor-cmd Enable motor command sending (default: DISABLED)" << std::endl;
        std::cout << "\nZLG Configuration:" << std::endl;
        std::cout << "  Default IP: 192.168.1.5" << std::endl;
        std::cout << "  Default Port: 8002" << std::endl;
        return 1;
    }

    std::string tcp_endpoint = argv[1];

    bool verbose = false;
    bool motor_cmd_enabled = false;

    for (int i = 2; i < argc; i++) {
        if (std::string(argv[i]) == "-v") {
            verbose = true;
        } else if (std::string(argv[i]) == "--enable-motor-cmd") {
            motor_cmd_enabled = true;
        }
    }

    std::cout << "========================================" << std::endl;
    std::cout << "ROS2 Motor Controller with ZLG Enable" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "TCP Endpoint: " << tcp_endpoint << std::endl;
    std::cout << "ZLG Device: " << "192.168.1.5:8002" << std::endl;
    std::cout << "Verbose Mode: " << (verbose ? "ON" : "OFF") << std::endl;
    std::cout << "Motor Command: " << (motor_cmd_enabled ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << "" << std::endl;

    rclcpp::init(argc, argv);

    try {
        auto bridge = std::make_shared<ROS2_to_TCP_Bridge>(verbose, motor_cmd_enabled);

        if (!bridge->InitializeTCP(tcp_endpoint)) {
            std::cerr << "Failed to initialize TCP connection" << std::endl;
            rclcpp::shutdown();
            return 1;
        }

        bridge->InitializeROS2();

        std::cout << "Initialization complete!" << std::endl << std::endl;

        bridge->Run();

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
