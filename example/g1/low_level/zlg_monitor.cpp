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

// ZLG 状态帧格式 (根据实际接收到的数据推断)
// 包头: 0x55 0x03 ...
// 可能包含错误计数器、总线状态等信息

class ZlgStatusMonitor {
private:
    int socket_fd_;
    bool running_;

    // 解析状态帧
    void ParseStatusFrame(const uint8_t* data, size_t len) {
        if (len < 27 || data[0] != 0x55) {
            return;
        }

        // 打印原始帧
        std::cout << "[RX] ";
        for (size_t i = 0; i < std::min(len, size_t(31)); i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(data[i]) << " ";
        }
        std::cout << std::dec << std::setfill(' ') << std::endl;

        // 尝试解析CAN ID和状态
        if (len >= 27) {
            // ZLG CAN包格式: 55 03 00 00 00 18 ... (类型03可能是接收帧)
            uint16_t data_len = data[4] | (data[5] << 8);
            uint32_t can_id = data[14] | (data[15] << 8) | (data[16] << 16) | (data[17] << 24);
            uint8_t channel = data[20];
            uint8_t dlc = data[21];

            std::cout << "  -> Type: 0x" << std::hex << static_cast<int>(data[1])
                      << " Len: " << std::dec << data_len
                      << " CAN_ID: 0x" << std::hex << can_id
                      << " CH: " << std::dec << static_cast<int>(channel)
                      << " DLC: " << static_cast<int>(dlc) << std::endl;

            // 打印数据段
            if (dlc > 0 && len >= 22 + dlc) {
                std::cout << "  -> Data: ";
                for (uint8_t i = 0; i < dlc; i++) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0')
                              << static_cast<int>(data[22 + i]) << " ";
                }
                std::cout << std::dec << std::setfill(' ') << std::endl;
            }

            // 检查是否是错误帧 (CAN ID 的高位可能表示错误状态)
            if ((can_id & 0x20000000) || (can_id & 0x1C000000)) {
                std::cout << "  *** POSSIBLE ERROR FRAME ***" << std::endl;
            }
        }
    }

public:
    ZlgStatusMonitor() : socket_fd_(-1), running_(false) {}

    ~ZlgStatusMonitor() {
        Stop();
        if (socket_fd_ >= 0) {
            close(socket_fd_);
        }
    }

    bool Connect(const std::string& ip, int port) {
        socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ < 0) {
            std::cerr << "Socket creation failed" << std::endl;
            return false;
        }

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);

        if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) <= 0) {
            std::cerr << "Invalid IP address" << std::endl;
            close(socket_fd_);
            return false;
        }

        if (connect(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "Connection failed to " << ip << ":" << port << std::endl;
            close(socket_fd_);
            return false;
        }

        std::cout << "Connected to ZLG device " << ip << ":" << port << std::endl;
        return true;
    }

    void Start() {
        running_ = true;
        std::cout << "\n=== Monitoring ZLG status frames ===" << std::endl;
        std::cout << "Press Ctrl+C to stop" << std::endl << std::endl;

        uint8_t buffer[4096];
        int frame_count = 0;
        auto start_time = std::chrono::steady_clock::now();
        auto last_print = start_time;

        while (running_) {
            memset(buffer, 0, sizeof(buffer));
            ssize_t received = recv(socket_fd_, buffer, sizeof(buffer), 0);

            if (received > 0) {
                frame_count++;

                // 每秒打印一次统计
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print).count() >= 1) {
                    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
                    std::cout << "\n--- Statistics (running for " << elapsed << "s) ---" << std::endl;
                    std::cout << "Total frames: " << frame_count
                              << " | Rate: " << (frame_count / std::max(1L, static_cast<long>(elapsed))) << " fps"
                              << std::endl << std::endl;
                    last_print = now;
                }

                // 解析帧
                ParseStatusFrame(buffer, received);
            } else if (received < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    std::cerr << "Recv error: " << strerror(errno) << std::endl;
                    break;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::cout << "\n=== Monitoring stopped ===" << std::endl;
    }

    void Stop() {
        running_ = false;
    }
};

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <ip:port>" << std::endl;
        std::cout << "Example: " << argv[0] << " 192.168.1.5:8003" << std::endl;
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

    ZlgStatusMonitor monitor;
    if (!monitor.Connect(ip, port)) {
        return 1;
    }

    monitor.Start();

    return 0;
}
