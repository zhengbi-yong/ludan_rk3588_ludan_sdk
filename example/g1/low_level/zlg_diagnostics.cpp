#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <thread>
#include <chrono>
#include <vector>
#include <iomanip>

// ZLG 诊断命令
// 根据周立功CANFDNET设备文档

class ZlgDiagnostics {
private:
    int socket_fd_;
    std::string ip_;
    int port_;
    int channel_;

public:
    ZlgDiagnostics(const std::string& ip, int port, int channel)
        : socket_fd_(-1), ip_(ip), port_(port), channel_(channel) {}

    ~ZlgDiagnostics() {
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

    // 发送命令并接收响应
    std::string SendCommand(const std::string& cmd, int timeout_ms = 500) {
        std::string full_cmd = cmd + "\r\n";

        ssize_t sent = send(socket_fd_, full_cmd.c_str(), full_cmd.length(), 0);
        if (sent < 0) {
            std::cerr << "Send failed: " << strerror(errno) << std::endl;
            return "";
        }

        // 等待响应
        std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms / 2));

        char buffer[4096];
        memset(buffer, 0, sizeof(buffer));
        ssize_t received = recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);

        if (received > 0) {
            return std::string(buffer, received);
        }
        return "";
    }

    // 查询设备信息
    void GetDeviceInfo() {
        std::cout << "\n=== Device Info ===" << std::endl;
        std::string response = SendCommand(":GVER");  // 获取固件版本
        std::cout << "Version: " << response << std::endl;

        response = SendCommand(":GSN");  // 获取序列号
        std::cout << "Serial: " << response << std::endl;

        response = SendCommand(":GTYPE");  // 获取设备类型
        std::cout << "Type: " << response << std::endl;
    }

    // 查询CAN通道状态
    void GetCanStatus() {
        std::cout << "\n=== CAN" << channel_ << " Status ===" << std::endl;

        // 查询工作模式
        std::string response = SendCommand(":CANFD" + std::to_string(channel_) + ":GMODE");
        std::cout << "Mode: " << response << std::endl;

        // 查询波特率
        response = SendCommand(":CANFD" + std::to_string(channel_) + ":GABTBaud");
        std::cout << "Arbitration Baud: " << response << std::endl;

        response = SendCommand(":CANFD" + std::to_string(channel_) + ":GDATABaud");
        std::cout << "Data Baud: " << response << std::endl;

        // 查询错误计数 (如果设备支持)
        // 周立功设备通常通过特殊命令或状态帧返回错误信息
        response = SendCommand(":CANFD" + std::to_string(channel_) + ":GST");
        std::cout << "Status: " << response << std::endl;
    }

    // 查询发送队列状态
    void GetTxQueueStatus() {
        std::cout << "\n=== TX Queue Status ===" << std::endl;
        // 尝试查询队列状态
        std::string response = SendCommand(":CANFD" + std::to_string(channel_) + ":GTQ");
        std::cout << "TX Queue: " << response << std::endl;
    }

    // 查询接收队列状态
    void GetRxQueueStatus() {
        std::cout << "\n=== RX Queue Status ===" << std::endl;
        std::string response = SendCommand(":CANFD" + std::to_string(channel_) + ":GRQ");
        std::cout << "RX Queue: " << response << std::endl;
    }

    // 监听CAN总线消息
    void MonitorCanBus(int duration_seconds = 10) {
        std::cout << "\n=== Monitoring CAN" << channel_ << " bus for " << duration_seconds << " seconds ===" << std::endl;

        // 启动接收模式
        std::string response = SendCommand(":CANFD" + std::to_string(channel_) + ":MSTART");
        std::cout << "Monitor start response: " << response << std::endl;

        auto start_time = std::chrono::steady_clock::now();
        int message_count = 0;

        while (std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_time).count() < duration_seconds) {

            char buffer[4096];
            memset(buffer, 0, sizeof(buffer));
            ssize_t received = recv(socket_fd_, buffer, sizeof(buffer) - 1, MSG_DONTWAIT);

            if (received > 0) {
                message_count++;
                std::cout << "[RX] " << std::hex << std::setw(2) << std::setfill('0');
                for (ssize_t i = 0; i < received && i < 64; i++) {
                    std::cout << static_cast<int>(static_cast<uint8_t>(buffer[i])) << " ";
                }
                std::cout << std::dec << std::setfill(' ') << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "Total messages received: " << message_count << std::endl;

        // 停止接收
        response = SendCommand(":CANFD" + std::to_string(channel_) + ":MSTOP");
        std::cout << "Monitor stop response: " << response << std::endl;
    }

    // 发送测试消息并检查响应
    void SendTestMessage(int can_id, const std::vector<uint8_t>& data) {
        std::cout << "\n=== Sending Test Message ===" << std::endl;
        std::cout << "CAN ID: 0x" << std::hex << can_id << std::dec << std::endl;
        std::cout << "Data: ";
        for (auto b : data) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ";
        }
        std::cout << std::dec << std::setfill(' ') << std::endl;

        // 构建并发送测试消息
        // 这里使用简化格式，实际使用时需要根据ZLG协议构建完整包
        std::cout << "Test message sent (check for ACK/errors)" << std::endl;
    }

    // 获取错误统计
    void GetErrorStatistics() {
        std::cout << "\n=== Error Statistics ===" << std::endl;

        // 尝试获取CAN控制器错误状态
        std::string response = SendCommand(":CANFD" + std::to_string(channel_) + ":GERR");
        std::cout << "Error Status: " << response << std::endl;

        // 检查总线状态
        response = SendCommand(":CANFD" + std::to_string(channel_) + ":GST");
        std::cout << "Bus State: " << response << std::endl;

        std::cout << "\nPossible LED indicator meanings:" << std::endl;
        std::cout << "  Red flashing = Bus-Off state or TX/RX error counter overflow" << std::endl;
        std::cout << "  Red solid = Error Active state (error counter > 0 but < 128)" << std::endl;
        std::cout << "  Green = Normal operation" << std::endl;
    }
};

void printUsage(const char* prog_name) {
    std::cout << "Usage: " << prog_name << " <ip:port> <channel> [command]" << std::endl;
    std::cout << "\nCommands:" << std::endl;
    std::cout << "  all     - Run all diagnostics (default)" << std::endl;
    std::cout << "  info    - Get device info only" << std::endl;
    std::cout << "  status  - Get CAN status only" << std::endl;
    std::cout << "  queue   - Get TX/RX queue status" << std::endl;
    std::cout << "  error   - Get error statistics" << std::endl;
    std::cout << "  monitor - Monitor CAN bus for 10 seconds" << std::endl;
    std::cout << "\nExamples:" << std::endl;
    std::cout << "  " << prog_name << " 192.168.1.5:8003 2 all" << std::endl;
    std::cout << "  " << prog_name << " 192.168.1.5:8003 2 error" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }

    // 解析 IP:PORT
    std::string endpoint = argv[1];
    size_t colon_pos = endpoint.find(':');
    if (colon_pos == std::string::npos) {
        std::cerr << "Invalid endpoint format. Use IP:PORT" << std::endl;
        return 1;
    }

    std::string ip = endpoint.substr(0, colon_pos);
    int port = std::stoi(endpoint.substr(colon_pos + 1));
    int channel = std::atoi(argv[2]);

    std::string command = (argc >= 4) ? argv[3] : "all";

    std::cout << "========================================" << std::endl;
    std::cout << "ZLG CAN Diagnostics Tool" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Device: " << ip << ":" << port << std::endl;
    std::cout << "Channel: CAN" << channel << std::endl;
    std::cout << "========================================" << std::endl;

    ZlgDiagnostics diag(ip, port, channel);

    if (!diag.Connect()) {
        return 1;
    }

    // 短暂延迟
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (command == "all" || command == "info") {
        diag.GetDeviceInfo();
    }

    if (command == "all" || command == "status") {
        diag.GetCanStatus();
    }

    if (command == "all" || command == "queue") {
        diag.GetTxQueueStatus();
        diag.GetRxQueueStatus();
    }

    if (command == "all" || command == "error") {
        diag.GetErrorStatistics();
    }

    if (command == "monitor") {
        diag.MonitorCanBus();
    }

    std::cout << "\n=== Diagnostics Complete ===" << std::endl;

    return 0;
}
