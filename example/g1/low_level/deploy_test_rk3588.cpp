#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <iomanip>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <mutex>

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_STATE_TOPIC = "rt/lowstate";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

const int G1_NUM_MOTOR = 29;

// Network protocol structures (必须与客户端一致)
struct MotorCommand {
    int motor_id;
    double q;        // position
    double dq;       // velocity
    double kp;       // position gain
    double kd;       // damping gain
    double tau;      // torque
    uint64_t timestamp;
};

struct NetworkPacket {
    uint32_t magic;           // 协议魔数
    uint32_t sequence;        // 序列号
    uint64_t timestamp;       // 时间戳
    int motor_count;          // 关节数量
    MotorCommand motors[29];  // 电机命令数组
    uint32_t crc;            // CRC校验
};

const uint32_t PACKET_MAGIC = 0xDEADBEEF;

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

class NetworkCommandServer {
private:
    std::atomic<bool> should_run_{false};
    std::thread server_thread_;
    std::thread dds_thread_;
    std::thread monitor_thread_;

    // 网络相关
    int server_socket_;
    int client_socket_;
    struct sockaddr_in server_addr_;
    struct sockaddr_in client_addr_;
    socklen_t client_addr_len_;
    int server_port_;

    // DDS相关
    ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
    ChannelSubscriberPtr<LowState_> lowstate_subscriber_;

    // 状态统计
    uint64_t received_count_;
    uint64_t forwarded_count_;
    uint64_t state_count_;
    uint64_t error_count_;
    std::chrono::high_resolution_clock::time_point start_time_;

    // 缓冲最新的网络命令
    NetworkPacket latest_packet_;
    std::atomic<bool> has_new_packet_{false};
    std::mutex packet_mutex_;

public:
    NetworkCommandServer(int server_port)
        : server_socket_(-1), client_socket_(-1), server_port_(server_port),
          received_count_(0), forwarded_count_(0), state_count_(0), error_count_(0),
          client_addr_len_(sizeof(client_addr_)) {

        std::cout << "🌐 网络命令服务端初始化完成" << std::endl;
        std::cout << "🔗 监听端口: " << server_port_ << std::endl;
    }

    ~NetworkCommandServer() {
        should_run_ = false;

        if (client_socket_ >= 0) {
            close(client_socket_);
        }
        if (server_socket_ >= 0) {
            close(server_socket_);
        }

        if (server_thread_.joinable()) {
            server_thread_.join();
        }
        if (dds_thread_.joinable()) {
            dds_thread_.join();
        }
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }
    }

    uint32_t CalculateCRC(const NetworkPacket& packet) {
        // 简单的CRC实现，与客户端一致
        uint32_t crc = 0;
        const uint8_t* data = reinterpret_cast<const uint8_t*>(&packet);
        size_t len = sizeof(packet) - sizeof(packet.crc); // 不包括CRC字段本身

        for (size_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 1) {
                    crc = (crc >> 1) ^ 0xEDB88320;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }

    bool ValidatePacket(const NetworkPacket& packet) {
        // 检查魔数
        if (packet.magic != PACKET_MAGIC) {
            std::cerr << "❌ 无效的魔数: 0x" << std::hex << packet.magic << std::endl;
            return false;
        }

        // 检查关节数量
        if (packet.motor_count != G1_NUM_MOTOR) {
            std::cerr << "❌ 无效的关节数量: " << packet.motor_count << std::endl;
            return false;
        }

        // 检查CRC
        uint32_t calculated_crc = CalculateCRC(packet);
        if (calculated_crc != packet.crc) {
            std::cerr << "❌ CRC校验失败: 计算值=0x" << std::hex << calculated_crc
                      << ", 接收值=0x" << packet.crc << std::endl;
            return false;
        }

        return true;
    }

    void InitializeDDS(const std::string& network_interface) {
        std::cout << "🔗 初始化DDS网络接口: " << network_interface << std::endl;

        // 初始化DDS通道
        ChannelFactory::Instance()->Init(0, network_interface);

        // 创建LowCmd发布器
        lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
        lowcmd_publisher_->InitChannel();
        std::cout << "✅ LowCmd发布器已创建 (话题: " << HG_CMD_TOPIC << ")" << std::endl;

        // 创建LowState订阅器
        lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
        lowstate_subscriber_->InitChannel(std::bind(&NetworkCommandServer::LowStateHandler, this, std::placeholders::_1), 1);
        std::cout << "✅ LowState订阅器已创建 (话题: " << HG_STATE_TOPIC << ")" << std::endl;
    }

    void LowStateHandler(const void *message) {
        state_count_++;

        // 每100个状态消息打印一次
        if (state_count_ % 100 == 0) {
            std::cout << "📥 接收到状态消息 #" << state_count_ << std::endl;
        }
    }

    bool SetupServer() {
        std::cout << "🔧 设置TCP服务器..." << std::endl;

        // 创建服务器socket
        server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_socket_ < 0) {
            std::cerr << "❌ 创建服务器socket失败" << std::endl;
            return false;
        }

        // 设置socket选项，允许地址重用
        int opt = 1;
        if (setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            std::cerr << "❌ 设置socket选项失败" << std::endl;
            close(server_socket_);
            server_socket_ = -1;
            return false;
        }

        // 绑定地址和端口
        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_addr.s_addr = INADDR_ANY;
        server_addr_.sin_port = htons(server_port_);

        if (bind(server_socket_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0) {
            std::cerr << "❌ 绑定端口 " << server_port_ << " 失败: " << strerror(errno) << std::endl;
            close(server_socket_);
            server_socket_ = -1;
            return false;
        }

        // 开始监听
        if (listen(server_socket_, 1) < 0) {
            std::cerr << "❌ 监听失败: " << strerror(errno) << std::endl;
            close(server_socket_);
            server_socket_ = -1;
            return false;
        }

        std::cout << "✅ TCP服务器已启动，监听端口 " << server_port_ << std::endl;
        return true;
    }

    void ServerThread() {
        std::cout << "🚀 启动服务器线程" << std::endl;

        while (should_run_) {
            std::cout << "⏳ 等待Jetson客户端连接..." << std::endl;

            // 等待客户端连接
            client_socket_ = accept(server_socket_, (struct sockaddr*)&client_addr_, &client_addr_len_);
            if (client_socket_ < 0) {
                if (should_run_) {
                    std::cerr << "❌ 接受连接失败: " << strerror(errno) << std::endl;
                }
                continue;
            }

            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &client_addr_.sin_addr, client_ip, INET_ADDRSTRLEN);
            std::cout << "✅ 客户端已连接: " << client_ip << ":" << ntohs(client_addr_.sin_port) << std::endl;

            // 处理客户端数据
            HandleClient();

            // 关闭客户端连接
            close(client_socket_);
            client_socket_ = -1;
            std::cout << "🔌 客户端连接已断开" << std::endl;
        }

        std::cout << "🛑 服务器线程结束" << std::endl;
    }

    void HandleClient() {
        NetworkPacket packet;

        while (should_run_) {
            ssize_t received = recv(client_socket_, &packet, sizeof(packet), 0);

            if (received < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    std::cerr << "❌ 接收数据失败: " << strerror(errno) << std::endl;
                    error_count_++;
                    break;
                }
                continue;
            } else if (received == 0) {
                std::cout << "📞 客户端断开连接" << std::endl;
                break;
            } else if (received != sizeof(packet)) {
                std::cerr << "❌ 数据包大小不正确: " << received << "/" << sizeof(packet) << " bytes" << std::endl;
                error_count_++;
                break;
            }

            received_count_++;

            // 验证数据包
            if (!ValidatePacket(packet)) {
                error_count_++;
                continue;
            }

            // 更新最新数据包
            {
                std::lock_guard<std::mutex> lock(packet_mutex_);
                latest_packet_ = packet;
                has_new_packet_ = true;
            }

            // 调试输出 (每50个包)
            if (received_count_ % 50 == 0) {
                double elapsed = std::chrono::duration<double>(
                    std::chrono::high_resolution_clock::now() - start_time_).count();

                std::cout << "📨 ["
                         << std::fixed << std::setprecision(2) << elapsed << "s] "
                         << "接收 #" << std::setw(4) << received_count_ << " | "
                         << "SEQ #" << std::setw(6) << packet.sequence << " | "
                         << "频率: " << received_count_ / elapsed << " Hz"
                         << std::endl;
            }
        }
    }

    void DDSThread() {
        std::cout << "🚀 启动DDS转发线程" << std::endl;

        const auto dds_interval = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::duration<double>(1.0 / 50.0)); // 50Hz DDS频率
        auto next_dds_time = std::chrono::high_resolution_clock::now();

        while (should_run_) {
            auto now = std::chrono::high_resolution_clock::now();

            if (now >= next_dds_time) {
                NetworkPacket packet;
                bool has_packet = false;

                // 获取最新数据包
                {
                    std::lock_guard<std::mutex> lock(packet_mutex_);
                    if (has_new_packet_) {
                        packet = latest_packet_;
                        has_new_packet_ = false;
                        has_packet = true;
                    }
                }

                if (has_packet) {
                    // 转换为DDS格式并发送
                    LowCmd_ dds_low_command;
                    ConvertPacketToDDS(packet, dds_low_command);

                    // 计算CRC
                    dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);

                    // 发布DDS命令
                    lowcmd_publisher_->Write(dds_low_command);
                    forwarded_count_++;
                }

                // 计算下一次DDS发送时间
                next_dds_time += dds_interval;

                if (now > next_dds_time + dds_interval) {
                    next_dds_time = now;
                }
            }

            // 短暂休眠
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        std::cout << "🛑 DDS转发线程结束" << std::endl;
    }

    void ConvertPacketToDDS(const NetworkPacket& packet, LowCmd_& cmd) {
        cmd.mode_pr() = 1;  // PR模式
        cmd.mode_machine() = 1;  // G1类型

        for (int i = 0; i < G1_NUM_MOTOR && i < packet.motor_count; i++) {
            const auto& motor_cmd = packet.motors[i];

            cmd.motor_cmd().at(i).mode() = 1;  // Enable
            cmd.motor_cmd().at(i).q() = motor_cmd.q;
            cmd.motor_cmd().at(i).dq() = motor_cmd.dq;
            cmd.motor_cmd().at(i).kp() = motor_cmd.kp;
            cmd.motor_cmd().at(i).kd() = motor_cmd.kd;
            cmd.motor_cmd().at(i).tau() = motor_cmd.tau;
        }

        // 填充剩余关节
        for (int i = packet.motor_count; i < G1_NUM_MOTOR; i++) {
            cmd.motor_cmd().at(i).mode() = 0;  // Disable
            cmd.motor_cmd().at(i).q() = 0.0;
            cmd.motor_cmd().at(i).dq() = 0.0;
            cmd.motor_cmd().at(i).kp() = 0.0;
            cmd.motor_cmd().at(i).kd() = 0.0;
            cmd.motor_cmd().at(i).tau() = 0.0;
        }
    }

    void MonitorThread() {
        std::cout << "📊 启动监控线程" << std::endl;

        while (should_run_) {
            std::this_thread::sleep_for(std::chrono::seconds(3));

            if (received_count_ > 0) {
                auto elapsed = std::chrono::duration<double>(
                    std::chrono::high_resolution_clock::now() - start_time_).count();

                double recv_hz = received_count_ / elapsed;
                double forward_hz = forwarded_count_ / elapsed;
                double state_hz = state_count_ / elapsed;

                std::cout << "📈 实时统计 ["
                         << std::fixed << std::setprecision(1) << elapsed << "s] - "
                         << "接收: " << std::setw(3) << recv_hz << "Hz, "
                         << "转发: " << std::setw(3) << forward_hz << "Hz, "
                         << "状态: " << std::setw(3) << state_hz << "Hz, "
                         << "错误: " << error_count_
                         << std::endl;
            }
        }

        std::cout << "📊 监控线程结束" << std::endl;
    }

    void StartServer(const std::string& network_interface) {
        std::cout << "\n🎯 开始网络命令服务端" << std::endl;
        std::cout << "======================================" << std::endl;

        // 初始化DDS
        InitializeDDS(network_interface);

        // 设置TCP服务器
        if (!SetupServer()) {
            std::cerr << "❌ 无法启动TCP服务器" << std::endl;
            return;
        }

        should_run_ = true;
        received_count_ = 0;
        forwarded_count_ = 0;
        state_count_ = 0;
        error_count_ = 0;
        start_time_ = std::chrono::high_resolution_clock::now();

        // 启动服务器线程
        server_thread_ = std::thread(&NetworkCommandServer::ServerThread, this);

        // 启动DDS转发线程
        dds_thread_ = std::thread(&NetworkCommandServer::DDSThread, this);

        // 启动监控线程
        monitor_thread_ = std::thread(&NetworkCommandServer::MonitorThread, this);

        std::cout << "📋 服务端配置:" << std::endl;
        std::cout << "   - 监听端口: " << server_port_ << std::endl;
        std::cout << "   - DDS接口: " << network_interface << std::endl;
        std::cout << "   - DDS话题: " << HG_CMD_TOPIC << std::endl;
        std::cout << "======================================" << std::endl;
        std::cout << "等待Jetson客户端连接..." << std::endl << std::endl;

        // 等待服务器线程结束
        server_thread_.join();

        // 停止其他线程
        should_run_ = false;
        if (dds_thread_.joinable()) {
            dds_thread_.join();
        }
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }

        // 显示最终统计
        ShowFinalStats();
    }

    void ShowFinalStats() {
        auto elapsed = std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now() - start_time_).count();

        std::cout << "\n📊 服务端运行统计" << std::endl;
        std::cout << "======================================" << std::endl;
        std::cout << "运行时长: " << elapsed << " 秒" << std::endl;
        std::cout << "接收包数: " << received_count_ << std::endl;
        std::cout << "转发包数: " << forwarded_count_ << std::endl;
        std::cout << "状态包数: " << state_count_ << std::endl;
        std::cout << "错误次数: " << error_count_ << std::endl;

        if (elapsed > 0) {
            std::cout << "平均接收频率: " << received_count_ / elapsed << " Hz" << std::endl;
            std::cout << "平均转发频率: " << forwarded_count_ / elapsed << " Hz" << std::endl;

            if (received_count_ > 0) {
                double forward_rate = (forwarded_count_ * 100.0) / received_count_;
                std::cout << "转发成功率: " << forward_rate << "%" << std::endl;
            }
        }

        std::cout << "======================================" << std::endl;
    }
};

int main(int argc, char const *argv[]) {
    if (argc < 3) {
        std::cout << "用法: deploy_test_rk3588 <network_interface> <server_port> [options]" << std::endl;
        std::cout << std::endl;
        std::cout << "参数说明:" << std::endl;
        std::cout << "  network_interface  DDS网络接口 (如: veth0, eth0, wlan0)" << std::endl;
        std::cout << "  server_port       TCP服务器监听端口" << std::endl;
        std::cout << std::endl;
        std::cout << "示例:" << std::endl;
        std::cout << "  ./deploy_test_rk3588 wlan0 8888                    # 使用wlan0接口和8888端口" << std::endl;
        std::cout << "  ./deploy_test_rk3588 eth0 9999                     # 使用eth0接口和9999端口" << std::endl;
        std::cout << "  ./deploy_test_rk3588 veth0 7777                    # 使用veth0接口和7777端口" << std::endl;
        std::cout << std::endl;
        std::cout << "注意: 确保防火墙允许指定端口的TCP连接" << std::endl;
        return 1;
    }

    std::string network_interface = argv[1];
    int server_port = std::stoi(argv[2]);

    try {
        std::cout << "======================================" << std::endl;
        std::cout << "🤖 G1机器人网络命令服务端 (RK3588)" << std::endl;
        std::cout << "======================================" << std::endl;

        // 创建服务端实例
        NetworkCommandServer server(server_port);

        // 启动服务端
        server.StartServer(network_interface);

    } catch (const std::exception& e) {
        std::cerr << "❌ 错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}