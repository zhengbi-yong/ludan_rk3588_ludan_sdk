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

// 周立功设备配置 (使用TCP)
struct ZhilgongConfig {
    std::string remote_ip = "192.168.1.5";  // 周立功设备IP
    int remote_port = 8003;                // 周立功设备TCP端口
    int channel = 3;                       // CAN通道 (0-7, 默认3=CAN3)
    int socket_fd = -1;
    bool initialized = false;

    // CAN-FD 波特率配置
    int arbitration_baud = 1000000;  // 仲裁域波特率 (1M bps)
    int data_baud = 5000000;         // 数据域波特率 (5M bps)
    bool baud_configured = false;    // 波特率是否已配置
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
//
// 字节偏移    字段                      说明
// ----------------------------------------------------------------------
// 0           header                   0x55 包头
// 1-3         type                     0x00 0x00 0x00 (CAN报文类型+保留位)
// 4-5         length                   大端序数据长度 (0x0018 = 24字节)
// 6-13        timestamp                8字节时间戳(微秒)，发送时填0
// 14-17       can_id                   4字节CAN ID (大端序: 0x000007FF)
// 18-19       frame_info               0x0000 (标准帧，无回显)
// 20          channel                  通道号 (0=CAN0, 3=CAN3)
// 21          dlc                      数据长度 (0-8)
// 22-29       data                     数据段 (固定8字节)
// 30          checksum                 XOR校验 (字节1-29的异或)
struct ZlgCanNetPacket {
    uint8_t header;                    // 0x55
    uint8_t type[3];                   // 0x00 0x00 0x00
    uint8_t length[2];                 // 大端序长度
    uint8_t timestamp[8];              // 8字节时间戳
    uint8_t can_id[4];                 // 大端序CAN ID
    uint8_t frame_info[2];             // 0x00 0x00
    uint8_t channel;                   // 通道号
    uint8_t dlc;                       // DLC
    uint8_t data[8];                   // 数据
    uint8_t checksum;                  // XOR校验
} __attribute__((packed));

// Helper class for building ZLG CAN network packets
class ZlgPacketBuilder {
public:
    // 时间戳模式
    enum TimestampMode {
        TIMESTAMP_ZERO = 0,      // 全0（发送时默认）
        TIMESTAMP_CURRENT,       // 使用当前时间
        TIMESTAMP_CUSTOM         // 使用指定的时间戳
    };

    // Build ZLG CAN network packet for transmission
    // per zlg_desc.txt format (严格按照ZLG规范)
    //
    // @param can_id: CAN ID (标准帧11位或扩展帧29位)
    // @param data: CAN数据字节 (8字节)
    // @param dlc: 数据长度 (0-8)
    // @param channel: CAN通道 (0-7, 默认3=CAN3)
    // @param timestamp_mode: 时间戳模式
    // @param custom_timestamp: 自定义时间戳（微秒）
    //
    // Example output (CAN ID=0x01, 使能帧):
    //   55 00 00 00 00 18 00 00 00 00 00 00 00 00 00 00 01 00 00 00 00 08 02 08 ff ff ff ff ff ff ff fc [checksum]
    //   ^^^^^^^^ ^^^^^^^ ^^^^^^^^^^^^^^^^^^^^ ^^^^^^ ^^^^^^ ^^ ^^^^^^^^^^^^^^^^^^^^^^^ ^
    //   头/类型   长度    时间戳                CANID   帧信息 通道 数据段                     校验
    static std::vector<uint8_t> BuildCanPacket(
            uint32_t can_id,
            const uint8_t* data,
            uint8_t dlc,
            uint8_t channel = 3,           // 默认使用CAN3
            TimestampMode timestamp_mode = TIMESTAMP_ZERO,
            uint64_t custom_timestamp = 0) {

        // 限制 DLC 范围
        if (dlc > 8) dlc = 8;
        if (dlc < 0) dlc = 0;

        // 固定数据段8字节（标准CAN）
        const uint8_t DATA_BYTES = 8;

        // 计算长度字段值：timestamp(8) + can_id(4) + frame_info(2) + channel(1) + dlc(1) + data(8) = 24
        const uint16_t DATA_LENGTH = 24;  // 0x0018

        std::vector<uint8_t> packet;
        packet.reserve(31);  // 固定31字节

        // 1. Header: 0x55
        packet.push_back(0x55);

        // 2. Type: 0x00 0x00 0x00 (标准CAN报文类型)
        packet.push_back(0x00);
        packet.push_back(0x00);
        packet.push_back(0x00);

        // 3. Data length: 大端序 0x0018 = 24字节
        packet.push_back((DATA_LENGTH >> 8) & 0xFF);  // 高字节 0x00
        packet.push_back(DATA_LENGTH & 0xFF);         // 低字节 0x18

        // 4. Timestamp: 8字节
        uint64_t timestamp = 0;
        if (timestamp_mode == TIMESTAMP_CURRENT) {
            auto now = std::chrono::high_resolution_clock::now();
            auto duration = now.time_since_epoch();
            timestamp = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
        } else if (timestamp_mode == TIMESTAMP_CUSTOM) {
            timestamp = custom_timestamp;
        }
        // 写入8字节时间戳（大端序）
        for (int i = 7; i >= 0; i--) {
            packet.push_back((timestamp >> (i * 8)) & 0xFF);
        }

        // 5. CAN ID: 4字节（大端序）
        packet.push_back((can_id >> 24) & 0xFF);
        packet.push_back((can_id >> 16) & 0xFF);
        packet.push_back((can_id >> 8) & 0xFF);
        packet.push_back(can_id & 0xFF);

        // 6. Frame info: 0x00 0x00 (标准帧，无回显)
        packet.push_back(0x00);
        packet.push_back(0x00);

        // 7. Channel: 通道号
        packet.push_back(channel);

        // 8. DLC: 数据长度
        packet.push_back(dlc);

        // 9. Data: 固定8字节
        for (uint8_t i = 0; i < DATA_BYTES; i++) {
            if (i < dlc) {
                packet.push_back(data[i]);
            } else {
                packet.push_back(0x00);  // 填充0
            }
        }

        // 10. XOR Checksum: 所有字节的异或（包括包头，从索引0开始）
        uint8_t checksum = 0;
        for (size_t i = 0; i < packet.size(); i++) {
            checksum ^= packet[i];
        }
        packet.push_back(checksum);

        return packet;
    }

    // 便捷方法：使用当前时间戳
    static std::vector<uint8_t> BuildCanPacketWithCurrentTime(
            uint32_t can_id, const uint8_t* data, uint8_t dlc, uint8_t channel = 3) {
        return BuildCanPacket(can_id, data, dlc, channel, TIMESTAMP_CURRENT, 0);
    }

    // 便捷方法：使用自定义时间戳
    static std::vector<uint8_t> BuildCanPacketWithCustomTime(
            uint32_t can_id, const uint8_t* data, uint8_t dlc, uint8_t channel, uint64_t timestamp_us) {
        return BuildCanPacket(can_id, data, dlc, channel, TIMESTAMP_CUSTOM, timestamp_us);
    }

    // 便捷方法：使用零时间戳（默认）
    static std::vector<uint8_t> BuildCanPacketWithZeroTime(
            uint32_t can_id, const uint8_t* data, uint8_t dlc, uint8_t channel = 3) {
        return BuildCanPacket(can_id, data, dlc, channel, TIMESTAMP_ZERO, 0);
    }

    // Convert motor command to CAN data bytes (MIT motor protocol format)
    // 将电机命令转换为 CAN 数据字节 (MIT 电机协议格式)
    //
    // @param cmd: 电机命令结构体 (包含 pos, vel, kp, kd, torq)
    // @param can_data: 输出 8 字节 CAN 数据
    //
    // MIT motor protocol format:
    // Bytes 0-1: Position (float to int16, ±12.5 rad -> ±32767)
    // Bytes 2-3: Velocity (float to int16, ±30 rad/s -> ±32767)
    // Bytes 4-5: Kp (float to int16, 0-500 -> 0-32767)
    // Bytes 6-7: Kd (float to int16, 0-50 -> 0-32767)
    static void MotorCommandToCanData(const MotorCommandCan& cmd, uint8_t* can_data) {
        // Convert position to int16 (scale: ±12.5 rad -> ±32767)
        int16_t pos_int = static_cast<int16_t>(cmd.pos * 32767.0f / 12.5f);
        can_data[0] = pos_int & 0xFF;
        can_data[1] = (pos_int >> 8) & 0xFF;

        // Convert velocity to int16 (scale: ±30 rad/s -> ±32767)
        int16_t vel_int = static_cast<int16_t>(cmd.vel * 32767.0f / 30.0f);
        can_data[2] = vel_int & 0xFF;
        can_data[3] = (vel_int >> 8) & 0xFF;

        // Convert Kp to int16 (scale: 0-500 -> 0-32767)
        int16_t kp_int = static_cast<int16_t>(cmd.kp * 32767.0f / 500.0f);
        can_data[4] = kp_int & 0xFF;
        can_data[5] = (kp_int >> 8) & 0xFF;

        // Convert Kd to int16 (scale: 0-50 -> 0-32767)
        int16_t kd_int = static_cast<int16_t>(cmd.kd * 32767.0f / 50.0f);
        can_data[6] = kd_int & 0xFF;
        can_data[7] = (kd_int >> 8) & 0xFF;
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

// ==================== ZLG CAN-FD 波特率配置 ====================
// ZLG CANFDNET 设备通过 TCP 命令配置 CAN-FD 波特率
// 命令格式: :CANFD{通道}:ABTBaud {仲裁域波特率}
//           :CANFD{通道}:DATABaud {数据域波特率}
//
// 示例:
//   :CANFD3:ABTBaud 1000000    (仲裁域 1M bps)
//   :CANFD3:DATABaud 5000000    (数据域 5M bps)
class ZlgBaudRateConfig {
public:
    // 配置 CAN-FD 波特率
    // @param socket_fd: TCP socket 文件描述符
    // @param channel: CAN 通道号 (0-7)
    // @param arb_baud: 仲裁域波特率 (如 1000000 = 1M bps)
    // @param data_baud: 数据域波特率 (如 5000000 = 5M bps)
    // @return: true 成功, false 失败
    static bool ConfigureBaudRate(int socket_fd, int channel, int arb_baud, int data_baud) {
        if (socket_fd < 0) {
            std::cerr << "Invalid socket fd" << std::endl;
            return false;
        }

        std::cout << "\n=== Configuring CAN-FD Baud Rate ===" << std::endl;
        std::cout << "Channel: CAN" << channel << std::endl;
        std::cout << "Arbitration Baud: " << arb_baud << " bps (" << (arb_baud / 1000000) << "M)" << std::endl;
        std::cout << "Data Baud: " << data_baud << " bps (" << (data_baud / 1000000) << "M)" << std::endl;

        // 构建仲裁域波特率配置命令
        std::string abt_cmd = FormatBaudCommand(channel, "ABTBaud", arb_baud);
        if (!SendConfigCommand(socket_fd, abt_cmd)) {
            std::cerr << "Failed to set arbitration baud rate" << std::endl;
            return false;
        }
        std::cout << "  Arbitration baud rate set to " << arb_baud << " bps" << std::endl;

        // 短暂延迟
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 构建数据域波特率配置命令
        std::string data_cmd = FormatBaudCommand(channel, "DATABaud", data_baud);
        if (!SendConfigCommand(socket_fd, data_cmd)) {
            std::cerr << "Failed to set data baud rate" << std::endl;
            return false;
        }
        std::cout << "  Data baud rate set to " << data_baud << " bps" << std::endl;

        std::cout << "=== CAN-FD Baud Rate Configuration Complete ===" << std::endl << std::endl;
        return true;
    }

private:
    // 格式化波特率配置命令
    // 格式: :CANFD{通道}:ABTBaud {波特率}
    static std::string FormatBaudCommand(int channel, const std::string& baud_type, int baud_rate) {
        char cmd[128];
        snprintf(cmd, sizeof(cmd), ":CANFD%d:%s %d", channel, baud_type.c_str(), baud_rate);
        return std::string(cmd);
    }

    // 发送配置命令
    static bool SendConfigCommand(int socket_fd, const std::string& command) {
        // 添加换行符
        std::string cmd_with_crlf = command + "\r\n";

        std::cout << "  Sending command: " << command << std::endl;

        ssize_t sent = send(socket_fd, cmd_with_crlf.c_str(), cmd_with_crlf.length(), 0);
        if (sent < 0) {
            std::cerr << "  Failed to send command: " << strerror(errno) << std::endl;
            return false;
        }
        if (sent != static_cast<ssize_t>(cmd_with_crlf.length())) {
            std::cerr << "  Partial send: " << sent << "/" << cmd_with_crlf.length() << " bytes" << std::endl;
            return false;
        }

        // 等待响应（ZLG 设备通常会返回 OK 或错误信息）
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        return true;
    }
};

// ZLG 网络包发送工具类 (不依赖 ROS2)
class ZlgNetworkSender {
private:
    ZhilgongConfig config_;
    bool verbose_;

public:
    ZlgNetworkSender(bool verbose = false) : verbose_(verbose) {}

    // 设置配置
    void SetConfig(const std::string& ip, int port, int channel) {
        config_.remote_ip = ip;
        config_.remote_port = port;
        config_.channel = channel;
        std::cout << "ZLG Config set: IP=" << ip << ", Port=" << port << ", Channel=" << channel << std::endl;
    }

    // 设置配置 (含波特率)
    void SetConfig(const std::string& ip, int port, int channel, int arb_baud, int data_baud) {
        config_.remote_ip = ip;
        config_.remote_port = port;
        config_.channel = channel;
        config_.arbitration_baud = arb_baud;
        config_.data_baud = data_baud;
        std::cout << "ZLG Config set: IP=" << ip << ", Port=" << port << ", Channel=" << channel << std::endl;
        std::cout << "  Arbitration Baud: " << arb_baud << " bps, Data Baud: " << data_baud << " bps" << std::endl;
    }

    // 获取配置
    const ZhilgongConfig& GetConfig() const { return config_; }

    // 初始化 TCP 连接
    bool InitializeConnection() {
        if (config_.initialized) {
            return true;
        }

        std::cout << "Initializing ZLG TCP connection to " << config_.remote_ip
                  << ":" << config_.remote_port << std::endl;

        config_.socket_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (config_.socket_fd < 0) {
            perror("TCP socket creation failed");
            return false;
        }

        int opt = 1;
        setsockopt(config_.socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        struct sockaddr_in zlg_addr;
        memset(&zlg_addr, 0, sizeof(zlg_addr));
        zlg_addr.sin_family = AF_INET;
        zlg_addr.sin_port = htons(config_.remote_port);

        if (inet_pton(AF_INET, config_.remote_ip.c_str(), &zlg_addr.sin_addr) <= 0) {
            std::cerr << "Invalid ZLG IP address: " << config_.remote_ip << std::endl;
            close(config_.socket_fd);
            return false;
        }

        if (connect(config_.socket_fd, (struct sockaddr*)&zlg_addr, sizeof(zlg_addr)) < 0) {
            std::cerr << "ZLG TCP connection failed: " << strerror(errno) << std::endl;
            close(config_.socket_fd);
            return false;
        }

        config_.initialized = true;
        std::cout << "ZLG TCP connection established successfully" << std::endl;

        // 配置 CAN-FD 波特率 (仲裁域 1M, 数据域 5M)
        if (!config_.baud_configured) {
            if (ZlgBaudRateConfig::ConfigureBaudRate(config_.socket_fd, config_.channel,
                                                      config_.arbitration_baud, config_.data_baud)) {
                config_.baud_configured = true;
            } else {
                std::cerr << "Warning: Failed to configure baud rate, using default" << std::endl;
            }
        }

        return true;
    }

    // 关闭连接
    void CloseConnection() {
        if (config_.initialized && config_.socket_fd >= 0) {
            close(config_.socket_fd);
            config_.socket_fd = -1;
            config_.initialized = false;
            std::cout << "ZLG connection closed" << std::endl;
        }
    }

    // 发送使能命令
    void SendEnableCommand(int motor_id, int channel) {
        if (!InitializeConnection()) {
            return;
        }

        int can_id = get_can_id_for_motor(motor_id);
        if (can_id <= 0) {
            std::cerr << "Invalid motor ID: " << motor_id << std::endl;
            return;
        }

        uint8_t enable_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

        std::cout << "\n>>> Sending ENABLE Command <<<" << std::endl;
        std::cout << "  Motor ID: " << motor_id << std::endl;
        std::cout << "  CAN ID:   0x" << std::hex << can_id << std::dec << std::endl;
        std::cout << "  Data:     FF FF FF FF FF FF FF FC" << std::endl;

        SendZlgNetworkPacket(can_id, enable_frame, sizeof(enable_frame), channel);
    }

    // 发送失能命令
    void SendDisableCommand(int motor_id, int channel) {
        if (!InitializeConnection()) {
            return;
        }

        int can_id = get_can_id_for_motor(motor_id);
        if (can_id <= 0) {
            std::cerr << "Invalid motor ID: " << motor_id << std::endl;
            return;
        }

        uint8_t disable_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

        std::cout << "\n>>> Sending DISABLE Command <<<" << std::endl;
        std::cout << "  Motor ID: " << motor_id << std::endl;
        std::cout << "  CAN ID:   0x" << std::hex << can_id << std::dec << std::endl;
        std::cout << "  Data:     FF FF FF FF FF FF FF FD" << std::endl;

        SendZlgNetworkPacket(can_id, disable_frame, sizeof(disable_frame), channel);
    }

private:
    // 发送 ZLG 网络包
    void SendZlgNetworkPacket(int can_id, const uint8_t* data, size_t data_len, int channel) {
        if (!config_.initialized || config_.socket_fd < 0) {
            std::cerr << "ZLG device not connected" << std::endl;
            return;
        }

        std::vector<uint8_t> zlg_packet = ZlgPacketBuilder::BuildCanPacket(
            static_cast<uint32_t>(can_id), data, static_cast<uint8_t>(data_len), channel
        );

        std::cout << "  Raw ZLG Packet [" << zlg_packet.size() << " bytes]: ";
        for (size_t i = 0; i < zlg_packet.size(); i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(zlg_packet[i]) << " ";
        }
        std::cout << std::dec << std::setfill(' ') << std::endl;

        ssize_t sent = send(config_.socket_fd, zlg_packet.data(), zlg_packet.size(), 0);

        if (sent < 0) {
            std::cerr << "Failed to send ZLG packet: " << strerror(errno) << std::endl;
        } else if (sent != static_cast<ssize_t>(zlg_packet.size())) {
            std::cerr << "Partial send: " << sent << "/" << zlg_packet.size() << " bytes" << std::endl;
        } else {
            std::cout << "  Sent " << sent << " bytes to ZLG device" << std::endl;
        }
    }

    int get_can_id_for_motor(int motor_id) {
        if (motor_id >= 1 && motor_id <= 30) {
            return motor_id;
        }
        return 0;
    }
};

// ROS2 桥接类 (继承自 rclcpp::Node)
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
    std::mutex motor_commands_mutex_;  // 保护 motor_commands_ 和 command_history_

    std::thread can_send_thread_;
    std::atomic<bool> should_stop_{false};

    std::atomic<bool> enable_command_active_{false};
    std::atomic<bool> can_send_started_{false};
    std::mutex pause_mutex_;
    std::condition_variable pause_cv_;

    // 发送统计
    std::atomic<uint64_t> send_count_{0};
    std::atomic<uint64_t> send_success_count_{0};
    std::atomic<uint64_t> send_error_count_{0};
    std::atomic<uint64_t> ros2_msg_count_{0};

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

    // 设置 ZLG 配置 (从 main 函数调用)
    void SetZlgConfig(const std::string& ip, int port, int channel) {
        zhilgong_config_.remote_ip = ip;
        zhilgong_config_.remote_port = port;
        zhilgong_config_.channel = channel;

        std::cout << "ZLG Config set: IP=" << ip << ", Port=" << port << ", Channel=" << channel << std::endl;
    }

    // 设置 ZLG 配置 (含波特率)
    void SetZlgConfig(const std::string& ip, int port, int channel, int arb_baud, int data_baud) {
        zhilgong_config_.remote_ip = ip;
        zhilgong_config_.remote_port = port;
        zhilgong_config_.channel = channel;
        zhilgong_config_.arbitration_baud = arb_baud;
        zhilgong_config_.data_baud = data_baud;

        std::cout << "ZLG Config set: IP=" << ip << ", Port=" << port << ", Channel=" << channel << std::endl;
        std::cout << "  Arbitration Baud: " << arb_baud << " bps, Data Baud: " << data_baud << " bps" << std::endl;
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

        // 不再覆盖 ZLG 配置，ZLG 配置由 SetZlgConfig 独立设置
        // zhilgong_config_.remote_ip = tcp_config_.remote_ip;    // 已移除
        // zhilgong_config_.remote_port = tcp_config_.remote_port; // 已移除

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
        std::cout << "  ZLG Channel: CAN" << zhilgong_config_.channel << std::endl;

        // 配置 CAN-FD 波特率 (仲裁域 1M, 数据域 5M)
        if (!zhilgong_config_.baud_configured) {
            if (ZlgBaudRateConfig::ConfigureBaudRate(zhilgong_config_.socket_fd, zhilgong_config_.channel,
                                                      zhilgong_config_.arbitration_baud, zhilgong_config_.data_baud)) {
                zhilgong_config_.baud_configured = true;
            } else {
                std::cerr << "Warning: Failed to configure baud rate, using default" << std::endl;
            }
        }

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

        // 增加 ROS2 消息计数
        ros2_msg_count_++;

        // 使用互斥锁保护 motor_commands_ 和 command_history_
        std::lock_guard<std::mutex> lock(motor_commands_mutex_);

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

        std::cout << ">>> 500Hz motor command sending STARTED <<<" << std::endl;
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

        // Build ZLG CAN network packet with configured channel
        std::vector<uint8_t> zlg_packet = ZlgPacketBuilder::BuildCanPacket(
            static_cast<uint32_t>(can_id), data, static_cast<uint8_t>(data_len), zhilgong_config_.channel
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

    // Send motor command using ZLG CAN network packet format
    // 使用 ZLG CAN 网络包格式发送电机命令
    // 使用与 SendZlgNetworkPacket 相同的方式打包发送
    void SendMotorCommandTCP(const MotorCommandCan& cmd) {
        send_count_++;

        if (!zhilgong_config_.initialized) {
            send_error_count_++;
            return;
        }

        // Convert motor command to CAN data bytes (MIT motor protocol)
        // 将电机命令转换为 CAN 数据字节 (MIT 电机协议)
        uint8_t can_data[8];
        ZlgPacketBuilder::MotorCommandToCanData(cmd, can_data);

        // Build ZLG CAN network packet
        // 构建 ZLG CAN 网络包
        uint32_t can_id = cmd.motor_id;  // Use motor_id as CAN ID

        // 打印调试信息（每100次打印一次，避免刷屏）
        static int debug_count = 0;
        if (++debug_count % 100 == 1) {
            std::cout << "[MotorCmd] Motor ID: " << cmd.motor_id
                      << " | CAN ID: 0x" << std::hex << can_id << std::dec
                      << " | Data: ";
            for (int i = 0; i < 8; i++) {
                std::cout << std::hex << std::setw(2) << std::setfill('0')
                          << static_cast<int>(can_data[i]) << " ";
            }
            std::cout << std::dec << std::setfill(' ') << std::endl;
        }

        // 使用与 SendZlgNetworkPacket 完全相同的方式发送
        // 调用统一的 SendZlgNetworkPacket 函数
        SendZlgNetworkPacket(can_id, can_data, 8);
        send_success_count_++;
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
        std::cout << ">>> 500Hz TCP send thread started, waiting for enable command to start sending..." << std::endl;

        const auto interval = std::chrono::milliseconds(2);
        const auto log_interval = std::chrono::seconds(1);  // 每秒打印一次
        auto next_send_time = std::chrono::high_resolution_clock::now();
        auto next_log_time = next_send_time + log_interval;

        uint64_t last_send_count = 0;
        uint64_t last_success_count = 0;
        uint64_t last_error_count = 0;
        uint64_t last_ros2_count = 0;
        bool first_log = true;  // 第一次打印等待状态

        while (!should_stop_) {
            auto now = std::chrono::high_resolution_clock::now();

            // 使用 wait_for 而不是 wait，设置1秒超时以便定期打印日志
            std::unique_lock<std::mutex> lock(pause_mutex_);
            bool should_send = pause_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
                return (can_send_started_ && !enable_command_active_) || should_stop_;
            });
            lock.unlock();

            if (should_stop_) {
                break;
            }

            // 如果可以发送，执行发送逻辑
            if (should_send && motor_cmd_enabled_) {
                if (now >= next_send_time) {
                    // 使用互斥锁保护 command_history_
                    std::lock_guard<std::mutex> lock(motor_commands_mutex_);

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

                    next_send_time += interval;

                    if (now > next_send_time + interval) {
                        next_send_time = now + interval;
                    }
                }
            }

            // 每秒打印一次状态（不管是否在发送）
            if (now >= next_log_time) {
                uint64_t current_send_count = send_count_.load();
                uint64_t current_success_count = send_success_count_.load();
                uint64_t current_error_count = send_error_count_.load();
                uint64_t current_ros2_count = ros2_msg_count_.load();

                uint64_t sends_delta = current_send_count - last_send_count;
                uint64_t success_delta = current_success_count - last_success_count;
                uint64_t error_delta = current_error_count - last_error_count;
                uint64_t ros2_delta = current_ros2_count - last_ros2_count;

                if (!can_send_started_) {
                    // 等待使能命令阶段
                    std::cout << "[WAIT] Waiting for enable command via /motor_enable topic... "
                              << "ROS2_msg: " << current_ros2_count << " (+" << ros2_delta << "/s) | "
                              << "ZLG: " << (zhilgong_config_.initialized ? "Connected" : "Disconnected")
                              << std::endl;
                } else if (first_log) {
                    // 第一次开始发送，打印标题
                    std::cout << "\n========== Motor Command Sending Started ==========" << std::endl;
                    std::cout << "[STAT] "
                              << "ROS2_msg: " << current_ros2_count << " (+" << ros2_delta << "/s) | "
                              << "Send: " << current_send_count << " (+" << sends_delta << "/s) | "
                              << "Success: " << current_success_count << " (+" << success_delta << "/s) | "
                              << "Error: " << current_error_count << " (+" << error_delta << "/s) | "
                              << "ZLG: " << (zhilgong_config_.initialized ? "Connected" : "Disconnected")
                              << std::endl;
                    first_log = false;
                } else {
                    // 正常发送状态
                    std::cout << "[STAT] "
                              << "ROS2_msg: " << current_ros2_count << " (+" << ros2_delta << "/s) | "
                              << "Send: " << current_send_count << " (+" << sends_delta << "/s) | "
                              << "Success: " << current_success_count << " (+" << success_delta << "/s) | "
                              << "Error: " << current_error_count << " (+" << error_delta << "/s) | "
                              << "ZLG: " << (zhilgong_config_.initialized ? "Connected" : "Disconnected")
                              << std::endl;
                }

                last_send_count = current_send_count;
                last_success_count = current_success_count;
                last_error_count = current_error_count;
                last_ros2_count = current_ros2_count;
                next_log_time = now + log_interval;
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
    // 命令行参数结构
    struct {
        std::string zlg_ip = "192.168.1.5";
        int zlg_port = 8003;
        int channel = 2;  // CAN2 (默认改为CAN2)
        bool verbose = false;
        bool motor_cmd_enabled = false;

        // CAN-FD 波特率配置
        int arb_baud = 1000000;  // 仲裁域波特率 (默认 1M bps)
        int data_baud = 5000000;  // 数据域波特率 (默认 5M bps)

        // 直接命令模式
        bool direct_mode = false;
        int direct_motor_id = -1;
        bool direct_enable = false;  // true=enable, false=disable
    } params;

    if (argc < 2) {
        std::cout << "Usage: motor_controller_with_enable [zlg_endpoint] [options]" << std::endl;
        std::cout << "\nPositional Arguments:" << std::endl;
        std::cout << "  zlg_endpoint       ZLG device endpoint (e.g., 192.168.1.5:8002, default: 192.168.1.5:8003)" << std::endl;
        std::cout << "                     If omitted, port is determined by -p option" << std::endl;
        std::cout << "\nOptions:" << std::endl;
        std::cout << "  -p, --port <port>       ZLG device TCP port (default: 8003)" << std::endl;
        std::cout << "  -c, --channel <ch>      CAN channel 0-3 (default: 3)" << std::endl;
        std::cout << "  -v, --verbose           Enable verbose logging" << std::endl;
        std::cout << "  --enable-motor-cmd      Enable motor command sending (default: DISABLED)" << std::endl;
        std::cout << "\nCAN-FD Baud Rate Configuration:" << std::endl;
        std::cout << "  --arb-baud <rate>       Arbitration baud rate in bps (default: 1000000 = 1M)" << std::endl;
        std::cout << "  --data-baud <rate>      Data baud rate in bps (default: 5000000 = 5M)" << std::endl;
        std::cout << "\nDirect Command Mode (no ROS2 required):" << std::endl;
        std::cout << "  --enable <motor_id>     Send enable command to motor (1-30)" << std::endl;
        std::cout << "  --disable <motor_id>    Send disable command to motor (1-30)" << std::endl;
        std::cout << "\nExamples:" << std::endl;
        std::cout << "  # ROS2 mode - CAN2 with motor commands" << std::endl;
        std::cout << "  ./motor_controller_with_enable 192.168.1.5:8002 -c 2 --enable-motor-cmd" << std::endl;
        std::cout << "\n  # ROS2 mode - CAN3 with verbose logging" << std::endl;
        std::cout << "  ./motor_controller_with_enable 192.168.1.5:8003 -c 3 -v --enable-motor-cmd" << std::endl;
        std::cout << "\n  # Custom baud rate (arbitration 1M, data 5M)" << std::endl;
        std::cout << "  ./motor_controller_with_enable 192.168.1.5:8002 -c 2 --arb-baud 1000000 --data-baud 5000000" << std::endl;
        std::cout << "\n  # Direct enable/disable mode" << std::endl;
        std::cout << "  ./motor_controller_with_enable 192.168.1.5:8002 --enable 1" << std::endl;
        std::cout << "  ./motor_controller_with_enable 192.168.1.5:8002 -c 2 --disable 5" << std::endl;
        std::cout << "\n  # Enable all motors (1-10) on CAN2" << std::endl;
        std::cout << "  ./motor_controller_with_enable 192.168.1.5:8002 -c 2 --enable 1-10" << std::endl;
        return 1;
    }

    // 解析第一个位置参数 (ZLG endpoint，格式: IP:PORT 或仅 PORT)
    std::string first_arg = argv[1];
    size_t colon_pos = first_arg.find(':');
    if (colon_pos != std::string::npos) {
        // 格式: IP:PORT
        params.zlg_ip = first_arg.substr(0, colon_pos);
        params.zlg_port = std::stoi(first_arg.substr(colon_pos + 1));
    } else {
        // 尝试解析为端口号
        try {
            params.zlg_port = std::stoi(first_arg);
        } catch (...) {
            // 不是数字，当作 IP 处理（使用默认端口）
            params.zlg_ip = first_arg;
        }
    }

    // 解析可选参数
    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "-v" || arg == "--verbose") {
            params.verbose = true;
        }
        else if (arg == "--enable-motor-cmd") {
            params.motor_cmd_enabled = true;
        }
        else if ((arg == "-p" || arg == "--port") && i + 1 < argc) {
            params.zlg_port = std::atoi(argv[++i]);
        }
        else if ((arg == "-c" || arg == "--channel") && i + 1 < argc) {
            params.channel = std::atoi(argv[++i]);
        }
        else if (arg == "--arb-baud" && i + 1 < argc) {
            params.arb_baud = std::atoi(argv[++i]);
        }
        else if (arg == "--data-baud" && i + 1 < argc) {
            params.data_baud = std::atoi(argv[++i]);
        }
        else if (arg == "--enable" && i + 1 < argc) {
            params.direct_mode = true;
            params.direct_enable = true;

            // 解析 motor_id (支持单个 ID 或范围，如 "1-10")
            std::string motor_str = argv[++i];
            size_t dash_pos = motor_str.find('-');
            if (dash_pos != std::string::npos) {
                // 范围模式: 只保存起始ID，用于后续循环
                params.direct_motor_id = std::atoi(motor_str.substr(0, dash_pos).c_str());
            } else {
                params.direct_motor_id = std::atoi(motor_str.c_str());
            }
        }
        else if (arg == "--disable" && i + 1 < argc) {
            params.direct_mode = true;
            params.direct_enable = false;
            params.direct_motor_id = std::atoi(argv[++i]);
        }
    }

    std::cout << "========================================" << std::endl;
    std::cout << "ROS2 Motor Controller with ZLG Enable" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "ZLG Device: " << params.zlg_ip << ":" << params.zlg_port << std::endl;
    std::cout << "CAN Channel: " << params.channel << " (CAN" << params.channel << ")" << std::endl;
    std::cout << "CAN-FD Arbitration Baud: " << params.arb_baud << " bps (" << (params.arb_baud / 1000000) << "M)" << std::endl;
    std::cout << "CAN-FD Data Baud: " << params.data_baud << " bps (" << (params.data_baud / 1000000) << "M)" << std::endl;
    std::cout << "Mode: " << (params.direct_mode ? "Direct Command" : "ROS2 Bridge") << std::endl;
    std::cout << "Verbose: " << (params.verbose ? "ON" : "OFF") << std::endl;
    std::cout << "========================================" << std::endl;

    // 直接命令模式 (不需要ROS2)
    if (params.direct_mode) {
        std::cout << "\n=== Direct Command Mode ===" << std::endl;

        // 创建 ZLG 发送器 (不依赖 ROS2)
        auto sender = std::make_shared<ZlgNetworkSender>(params.verbose);

        // 设置 ZLG 配置 (含波特率)
        sender->SetConfig(params.zlg_ip, params.zlg_port, params.channel, params.arb_baud, params.data_baud);

        // 处理单个或多个电机
        std::string motor_str = argv[argc - 1];  // 最后一个参数是 motor_id
        size_t dash_pos = motor_str.find('-');

        if (dash_pos != std::string::npos) {
            // 范围模式: 1-10
            int start_id = std::atoi(motor_str.substr(0, dash_pos).c_str());
            int end_id = std::atoi(motor_str.substr(dash_pos + 1).c_str());

            std::cout << "Sending " << (params.direct_enable ? "ENABLE" : "DISABLE")
                      << " to motors " << start_id << " - " << end_id << std::endl;

            for (int motor_id = start_id; motor_id <= end_id; motor_id++) {
                if (params.direct_enable) {
                    sender->SendEnableCommand(motor_id, params.channel);
                } else {
                    sender->SendDisableCommand(motor_id, params.channel);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        } else {
            // 单个电机模式
            std::cout << "Sending " << (params.direct_enable ? "ENABLE" : "DISABLE")
                      << " to motor " << params.direct_motor_id << std::endl;

            if (params.direct_enable) {
                sender->SendEnableCommand(params.direct_motor_id, params.channel);
            } else {
                sender->SendDisableCommand(params.direct_motor_id, params.channel);
            }
        }

        std::cout << "\n=== Command Complete ===" << std::endl;
        return 0;
    }

    // ROS2 桥接模式
    rclcpp::init(argc, argv);

    try {
        auto bridge = std::make_shared<ROS2_to_TCP_Bridge>(params.verbose, params.motor_cmd_enabled);

        // 设置 ZLG 配置 (含波特率)
        bridge->SetZlgConfig(params.zlg_ip, params.zlg_port, params.channel, params.arb_baud, params.data_baud);

        // 初始化 TCP 连接 (用于上位机通信，如果需要)
        // 注意：如果不需要连接到上位机 TCP 服务器，可以跳过
        // if (!bridge->InitializeTCP(params.tcp_endpoint)) {
        //     std::cerr << "Failed to initialize TCP connection" << std::endl;
        //     rclcpp::shutdown();
        //     return 1;
        // }

        // 初始化周立功设备连接 (用于发送 CAN 命令)
        if (!bridge->InitializeZhilgongConnection()) {
            std::cerr << "Failed to initialize ZLG connection" << std::endl;
            rclcpp::shutdown();
            return 1;
        }

        bridge->InitializeROS2();

        std::cout << "\nInitialization complete!" << std::endl;
        std::cout << "Waiting for ROS2 commands..." << std::endl << std::endl;

        bridge->Run();

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
