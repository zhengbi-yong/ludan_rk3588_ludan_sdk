#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iomanip>

// CAN总线错误检查工具

class CanBusChecker {
private:
    int socket_fd_;
    std::string ip_;
    int port_;
    int channel_;

    // 构建ZLG网络包 (用于发送测试消息)
    std::vector<uint8_t> BuildZlgPacket(uint32_t can_id, const uint8_t* data, uint8_t dlc) {
        std::vector<uint8_t> packet;
        const uint16_t DATA_LENGTH = 24;  // 固定24字节数据段
        const uint8_t DATA_BYTES = 8;

        packet.push_back(0x55);  // Header
        packet.push_back(0x00);  // Type: 发送帧
        packet.push_back(0x00);
        packet.push_back(0x00);

        // Data length (大端序)
        packet.push_back((DATA_LENGTH >> 8) & 0xFF);
        packet.push_back(DATA_LENGTH & 0xFF);

        // Timestamp: 全0
        for (int i = 0; i < 8; i++) {
            packet.push_back(0x00);
        }

        // CAN ID (大端序)
        packet.push_back((can_id >> 24) & 0xFF);
        packet.push_back((can_id >> 16) & 0xFF);
        packet.push_back((can_id >> 8) & 0xFF);
        packet.push_back(can_id & 0xFF);

        // Frame info: 0x00 0x00
        packet.push_back(0x00);
        packet.push_back(0x00);

        // Channel
        packet.push_back(channel_);

        // DLC
        if (dlc > 8) dlc = 8;
        packet.push_back(dlc);

        // Data
        for (uint8_t i = 0; i < DATA_BYTES; i++) {
            if (i < dlc) {
                packet.push_back(data[i]);
            } else {
                packet.push_back(0x00);
            }
        }

        // XOR checksum
        uint8_t checksum = 0;
        for (size_t i = 0; i < packet.size(); i++) {
            checksum ^= packet[i];
        }
        packet.push_back(checksum);

        return packet;
    }

    // 发送测试消息
    bool SendTestMessage(uint32_t can_id, const uint8_t* data, uint8_t dlc) {
        std::vector<uint8_t> packet = BuildZlgPacket(can_id, data, dlc);

        std::cout << "[TX] CAN_ID: 0x" << std::hex << can_id << std::dec
                  << " Data: ";
        for (uint8_t i = 0; i < dlc; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(data[i]) << " ";
        }
        std::cout << std::dec << std::setfill(' ') << std::endl;

        ssize_t sent = send(socket_fd_, packet.data(), packet.size(), 0);
        if (sent < 0) {
            std::cerr << "  Send failed: " << strerror(errno) << std::endl;
            return false;
        }
        if (sent != static_cast<ssize_t>(packet.size())) {
            std::cerr << "  Partial send: " << sent << "/" << packet.size() << std::endl;
            return false;
        }

        std::cout << "  Sent " << sent << " bytes" << std::endl;
        return true;
    }

    // 监听接收帧并解析
    void ListenForFrames(int duration_ms) {
        auto start = std::chrono::steady_clock::now();
        uint8_t buffer[4096];

        while (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count() < duration_ms) {

            memset(buffer, 0, sizeof(buffer));
            ssize_t received = recv(socket_fd_, buffer, sizeof(buffer), MSG_DONTWAIT);

            if (received > 0) {
                ParseReceivedFrame(buffer, received);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // 解析接收到的帧
    void ParseReceivedFrame(const uint8_t* data, size_t len) {
        if (len < 31 || data[0] != 0x55) {
            return;
        }

        uint8_t type = data[1];
        uint16_t data_len = data[4] | (data[5] << 8);
        uint32_t can_id = data[14] | (data[15] << 8) | (data[16] << 16) | (data[17] << 24);
        uint8_t channel = data[20];
        uint8_t dlc = data[21];

        if (type == 0x03) {
            // 接收帧
            std::cout << "[RX] CAN_ID: 0x" << std::hex << can_id << std::dec
                      << " CH: " << static_cast<int>(channel)
                      << " DLC: " << static_cast<int>(dlc);

            if (dlc > 0 && len >= 22 + dlc) {
                std::cout << " Data: ";
                for (uint8_t i = 0; i < dlc; i++) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0')
                              << static_cast<int>(data[22 + i]) << " ";
                }
                std::cout << std::dec << std::setfill(' ');
            }
            std::cout << std::endl;

            // 检查是否是错误帧
            if ((can_id & 0x20000000) || (can_id & 0x1C000000)) {
                std::cout << "  *** ERROR FRAME DETECTED ***" << std::endl;
                std::cout << "  CAN_ID: 0x" << std::hex << can_id << std::dec << std::endl;
            }
        }
    }

public:
    CanBusChecker(const std::string& ip, int port, int channel)
        : socket_fd_(-1), ip_(ip), port_(port), channel_(channel) {}

    ~CanBusChecker() {
        if (socket_fd_ >= 0) {
            close(socket_fd_);
        }
    }

    bool Connect() {
        socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ < 0) {
            std::cerr << "Socket creation failed" << std::endl;
            return false;
        }

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port_);

        if (inet_pton(AF_INET, ip_.c_str(), &addr.sin_addr) <= 0) {
            std::cerr << "Invalid IP address" << std::endl;
            close(socket_fd_);
            return false;
        }

        if (connect(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "Connection failed to " << ip_ << ":" << port_ << std::endl;
            close(socket_fd_);
            return false;
        }

        std::cout << "Connected to ZLG device " << ip_ << ":" << port_ << std::endl;
        return true;
    }

    // 运行诊断测试
    void RunDiagnostics() {
        std::cout << "\n========================================" << std::endl;
        std::cout << "CAN Bus Diagnostics for CAN" << channel_ << std::endl;
        std::cout << "========================================\n" << std::endl;

        // 测试1: 发送使能命令
        std::cout << "=== Test 1: Send Enable Command ===" << std::endl;
        uint8_t enable_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
        if (SendTestMessage(1, enable_data, 8)) {
            ListenForFrames(500);
        }

        // 测试2: 发送电机命令
        std::cout << "\n=== Test 2: Send Motor Command ===" << std::endl;
        uint8_t motor_data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (SendTestMessage(1, motor_data, 8)) {
            ListenForFrames(500);
        }

        // 测试3: 发送多个命令
        std::cout << "\n=== Test 3: Send Multiple Commands ===" << std::endl;
        for (int i = 1; i <= 5; i++) {
            SendTestMessage(i, motor_data, 8);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        ListenForFrames(1000);

        // 诊断结果
        std::cout << "\n========================================" << std::endl;
        std::cout << "Diagnosis Results" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "\nRed LED flashing usually indicates:" << std::endl;
        std::cout << "  1. Bus-Off state (error counter >= 256)" << std::endl;
        std::cout << "  2. No ACK received (no other nodes on bus)" << std::endl;
        std::cout << "  3. Bit error / Stuffing error / CRC error" << std::endl;
        std::cout << "\nPossible solutions:" << std::endl;
        std::cout << "  1. Check if CAN bus is properly terminated (120Ω resistors)" << std::endl;
        std::cout << "  2. Check if there are other CAN nodes on the bus" << std::endl;
        std::cout << "  3. Verify baud rate matches all nodes" << std::endl;
        std::cout << "  4. Check wiring (CAN_H, CAN_L, GND)" << std::endl;
        std::cout << "  5. Try sending slower (add delays between messages)" << std::endl;
        std::cout << "\nIf no ACK is received, the CAN bus may be" << std::endl;
        std::cout << "  - Not properly terminated" << std::endl;
        std::cout << "  - Only has the ZLG device (no other nodes)" << std::endl;
        std::cout << "  - Using wrong baud rate" << std::endl;
    }
};

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <ip:port> <channel>" << std::endl;
        std::cout << "Example: " << argv[0] << " 192.168.1.5:8003 2" << std::endl;
        return 1;
    }

    std::string endpoint = argv[1];
    size_t colon_pos = endpoint.find(':');
    if (colon_pos == std::string::npos) {
        std::cerr << "Invalid endpoint format. Use IP:PORT" << std::endl;
        return 1;
    }

    std::string ip = endpoint.substr(0, colon_pos);
    int port = std::stoi(endpoint.substr(colon_pos + 1));
    int channel = std::atoi(argv[2]);

    CanBusChecker checker(ip, port, channel);
    if (!checker.Connect()) {
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    checker.RunDiagnostics();

    return 0;
}
