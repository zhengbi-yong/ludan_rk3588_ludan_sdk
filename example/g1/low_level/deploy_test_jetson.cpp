#include <iostream>
#include <cmath>
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

// Network protocol structures
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
const int G1_NUM_MOTOR = 29;

class NetworkCommandClient {
private:
    std::atomic<bool> should_run_{false};
    std::thread command_thread_;
    std::thread monitor_thread_;

    // 网络相关
    int socket_fd_;
    struct sockaddr_in server_addr_;
    std::string server_ip_;
    int server_port_;

    // 运动参数
    double amplitude_;
    double frequency_;
    double publish_rate_;
    double duration_;

    // 状态统计
    uint64_t command_count_;
    uint64_t sent_count_;
    uint64_t ack_count_;
    std::chrono::high_resolution_clock::time_point start_time_;

    // 目标关节配置
    struct JointConfig {
        int g1_id;
        std::string name;
        double phase_offset;
        double kp;
        double kd;
    };

    std::vector<JointConfig> target_joints_;
    uint32_t sequence_number_;

public:
    NetworkCommandClient(const std::string& server_ip, int server_port)
        : server_ip_(server_ip), server_port_(server_port),
          socket_fd_(-1),
          amplitude_(0.3), frequency_(0.5), publish_rate_(50.0), duration_(30.0),
          command_count_(0), sent_count_(0), ack_count_(0), sequence_number_(0) {

        // 配置目标关节 (踝关节)
        target_joints_ = {
            {4, "LeftAnklePitch", 0.0, 40.0, 1.0},
            {5, "LeftAnkleRoll", 0.0, 40.0, 1.0},
            {10, "RightAnklePitch", M_PI/2, 40.0, 1.0},
            {11, "RightAnkleRoll", 0.0, 40.0, 1.0}
        };

        std::cout << "🌐 网络命令客户端初始化完成" << std::endl;
        std::cout << "🔗 目标服务器: " << server_ip_ << ":" << server_port_ << std::endl;
        std::cout << "📊 测试参数:" << std::endl;
        std::cout << "   - 幅度: " << amplitude_ << " rad (" << amplitude_ * 180.0 / M_PI << "°)" << std::endl;
        std::cout << "   - 频率: " << frequency_ << " Hz" << std::endl;
        std::cout << "   - 发布频率: " << publish_rate_ << " Hz" << std::endl;
        std::cout << "   - 持续时间: " << duration_ << " 秒" << std::endl;
    }

    ~NetworkCommandClient() {
        should_run_ = false;
        if (socket_fd_ >= 0) {
            close(socket_fd_);
        }
        if (command_thread_.joinable()) {
            command_thread_.join();
        }
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }
    }

    void SetAmplitude(double amplitude) {
        amplitude_ = amplitude;
        std::cout << "📊 更新幅度: " << amplitude << " rad (" << amplitude * 180.0 / M_PI << "°)" << std::endl;
    }

    void SetFrequency(double frequency) {
        frequency_ = frequency;
        std::cout << "📊 更新频率: " << frequency << " Hz" << std::endl;
    }

    void SetPublishRate(double rate) {
        publish_rate_ = rate;
        std::cout << "📊 更新发布频率: " << rate << " Hz" << std::endl;
    }

    void SetDuration(double duration) {
        duration_ = duration;
        std::cout << "📊 更新持续时间: " << duration << " 秒" << std::endl;
    }

    bool ConnectToServer() {
        std::cout << "🔗 连接到服务器 " << server_ip_ << ":" << server_port_ << std::endl;

        // 创建socket
        socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ < 0) {
            std::cerr << "❌ 创建socket失败" << std::endl;
            return false;
        }

        // 设置服务器地址
        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(server_port_);

        if (inet_pton(AF_INET, server_ip_.c_str(), &server_addr_.sin_addr) <= 0) {
            std::cerr << "❌ 无效的服务器IP地址" << std::endl;
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        // 连接到服务器
        if (connect(socket_fd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0) {
            std::cerr << "❌ 连接服务器失败: " << strerror(errno) << std::endl;
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        std::cout << "✅ 成功连接到服务器" << std::endl;
        return true;
    }

    uint32_t CalculateCRC(const NetworkPacket& packet) {
        // 简单的CRC实现，实际可以使用更复杂的算法
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

    void GenerateSineWaveCommand(double time, NetworkPacket& packet) {
        // 清空packet
        memset(&packet, 0, sizeof(packet));
        packet.magic = PACKET_MAGIC;
        packet.sequence = sequence_number_++;
        packet.timestamp = static_cast<uint64_t>(time * 1e9); // 纳秒
        packet.motor_count = G1_NUM_MOTOR;

        // 生成关节命令
        for (int i = 0; i < G1_NUM_MOTOR; i++) {
            packet.motors[i].motor_id = i;

            // 检查是否是目标关节
            bool is_target = false;
            for (const auto& joint : target_joints_) {
                if (i == joint.g1_id) {
                    is_target = true;

                    // 生成正弦波位置命令
                    double phase = 2.0 * M_PI * frequency_ * time + joint.phase_offset;
                    double sine_value = amplitude_ * std::sin(phase);

                    packet.motors[i].q = sine_value;
                    packet.motors[i].dq = amplitude_ * 2.0 * M_PI * frequency_ * std::cos(phase);
                    packet.motors[i].kp = joint.kp;
                    packet.motors[i].kd = joint.kd;
                    packet.motors[i].tau = 0.0;
                    packet.motors[i].timestamp = packet.timestamp;
                    break;
                }
            }

            // 非目标关节设置为零位
            if (!is_target) {
                packet.motors[i].q = 0.0;
                packet.motors[i].dq = 0.0;
                packet.motors[i].kp = 0.0;
                packet.motors[i].kd = 0.0;
                packet.motors[i].tau = 0.0;
                packet.motors[i].timestamp = packet.timestamp;
            }
        }

        // 计算CRC
        packet.crc = CalculateCRC(packet);
    }

    bool SendPacket(const NetworkPacket& packet) {
        if (socket_fd_ < 0) {
            return false;
        }

        ssize_t sent = send(socket_fd_, &packet, sizeof(packet), MSG_NOSIGNAL);
        if (sent < 0) {
            std::cerr << "❌ 发送失败: " << strerror(errno) << std::endl;
            return false;
        } else if (sent != sizeof(packet)) {
            std::cerr << "❌ 发送不完整: " << sent << "/" << sizeof(packet) << " bytes" << std::endl;
            return false;
        }

        sent_count_++;
        return true;
    }

    void CommandThread() {
        std::cout << "🚀 启动命令发送线程" << std::endl;

        const auto interval = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_));
        auto next_send_time = std::chrono::high_resolution_clock::now();

        while (should_run_) {
            auto now = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration<double>(now - start_time_).count();

            // 检查是否超时
            if (elapsed >= duration_) {
                std::cout << "⏰ 测试时间结束 (" << duration_ << " 秒)" << std::endl;
                break;
            }

            if (now >= next_send_time) {
                // 生成正弦波命令
                NetworkPacket packet;
                GenerateSineWaveCommand(elapsed, packet);

                // 发送命令
                if (SendPacket(packet)) {
                    command_count_++;

                    // 调试输出 (每50个命令)
                    if (command_count_ % 50 == 0) {
                        double phase = 2.0 * M_PI * frequency_ * elapsed;
                        double sine_value = amplitude_ * std::sin(phase);

                        std::cout << "🌐 ["
                                 << std::fixed << std::setprecision(2) << elapsed << "s] "
                                 << "CMD #" << std::setw(4) << command_count_ << " | "
                                 << "SEQ #" << std::setw(6) << packet.sequence << " | "
                                 << "正弦值: " << std::setw(6) << std::setprecision(3) << sine_value << " rad ("
                                 << std::setw(5) << std::setprecision(1) << sine_value * 180.0 / M_PI << "°) | "
                                 << "频率: " << command_count_ / elapsed << " Hz"
                                 << std::endl;
                    }
                } else {
                    std::cerr << "❌ 发送命令失败" << std::endl;
                }

                // 计算下一次发送时间
                next_send_time += interval;

                // 如果已经落后太多，跳过一些周期
                if (now > next_send_time + interval) {
                    next_send_time = now;
                }
            }

            // 短暂休眠
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        std::cout << "🛑 命令发送线程结束" << std::endl;
    }

    void MonitorThread() {
        std::cout << "📊 启动监控线程" << std::endl;

        while (should_run_) {
            std::this_thread::sleep_for(std::chrono::seconds(2));

            if (command_count_ > 0) {
                auto elapsed = std::chrono::duration<double>(
                    std::chrono::high_resolution_clock::now() - start_time_).count();

                double cmd_hz = command_count_ / elapsed;
                double success_rate = (sent_count_ * 100.0) / command_count_;

                std::cout << "📈 实时统计 ["
                         << std::fixed << std::setprecision(1) << elapsed << "s] - "
                         << "命令: " << std::setw(3) << cmd_hz << "Hz, "
                         << "成功率: " << std::setw(3) << success_rate << "%"
                         << std::endl;
            }
        }

        std::cout << "📊 监控线程结束" << std::endl;
    }

    void StartTest() {
        std::cout << "\n🎯 开始网络正弦波运动测试" << std::endl;
        std::cout << "======================================" << std::endl;

        // 连接到服务器
        if (!ConnectToServer()) {
            std::cerr << "❌ 无法连接到服务器，测试终止" << std::endl;
            return;
        }

        should_run_ = true;
        command_count_ = 0;
        sent_count_ = 0;
        ack_count_ = 0;
        start_time_ = std::chrono::high_resolution_clock::now();

        // 启动命令线程
        command_thread_ = std::thread(&NetworkCommandClient::CommandThread, this);

        // 启动监控线程
        monitor_thread_ = std::thread(&NetworkCommandClient::MonitorThread, this);

        std::cout << "📋 测试配置:" << std::endl;
        std::cout << "   - 目标服务器: " << server_ip_ << ":" << server_port_ << std::endl;
        std::cout << "   - 目标关节: ";
        for (const auto& joint : target_joints_) {
            std::cout << joint.name << "(" << joint.g1_id << ") ";
        }
        std::cout << std::endl;
        std::cout << "   - 运动模式: 正弦波" << std::endl;
        std::cout << "   - 运动范围: ±" << amplitude_ << " rad (±" << amplitude_ * 180.0 / M_PI << "°)" << std::endl;
        std::cout << "   - 周期: " << 1.0 / frequency_ << " 秒" << std::endl;
        std::cout << "   - 总周期数: " << duration_ * frequency_ << std::endl;
        std::cout << "======================================" << std::endl;
        std::cout << "按 Ctrl+C 提前停止测试" << std::endl << std::endl;

        // 等待测试完成
        command_thread_.join();

        // 停止监控线程
        should_run_ = false;
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }

        // 显示最终统计
        ShowFinalStats();
    }

    void ShowFinalStats() {
        auto elapsed = std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now() - start_time_).count();

        std::cout << "\n📊 测试完成统计" << std::endl;
        std::cout << "======================================" << std::endl;
        std::cout << "测试时长: " << elapsed << " 秒" << std::endl;
        std::cout << "总命令数: " << command_count_ << std::endl;
        std::cout << "成功发送: " << sent_count_ << std::endl;
        std::cout << "接收确认: " << ack_count_ << std::endl;

        if (elapsed > 0) {
            std::cout << "平均命令频率: " << command_count_ / elapsed << " Hz" << std::endl;
            double success_rate = (sent_count_ * 100.0) / command_count_;
            std::cout << "发送成功率: " << success_rate << "%" << std::endl;
        }

        std::cout << "======================================" << std::endl;

        if (elapsed >= duration_ - 1.0) {
            std::cout << "✅ 测试完成 - 运行了完整的" << duration_ << "秒" << std::endl;
        } else {
            std::cout << "⚠️  测试提前结束 - 运行了" << elapsed << "秒" << std::endl;
        }
    }
};

int main(int argc, char const *argv[]) {
    if (argc < 3) {
        std::cout << "用法: deploy_test_jetson <server_ip> <server_port> [options]" << std::endl;
        std::cout << std::endl;
        std::cout << "参数说明:" << std::endl;
        std::cout << "  server_ip    RK3588服务器IP地址" << std::endl;
        std::cout << "  server_port  RK3588服务器端口" << std::endl;
        std::cout << std::endl;
        std::cout << "可选参数:" << std::endl;
        std::cout << "  --amplitude <rad> 正弦波幅度 (默认: 0.3 rad)" << std::endl;
        std::cout << "  --frequency <hz>  正弦波频率 (默认: 0.5 Hz)" << std::endl;
        std::cout << "  --rate <hz>       发布频率 (默认: 50 Hz)" << std::endl;
        std::cout << "  --duration <s>    测试持续时间 (默认: 30 秒)" << std::endl;
        std::cout << std::endl;
        std::cout << "示例:" << std::endl;
        std::cout << "  ./deploy_test_jetson 192.168.1.100 8888                    # 使用默认参数" << std::endl;
        std::cout << "  ./deploy_test_jetson 192.168.1.100 8888 --amplitude 0.5      # 更大幅度" << std::endl;
        std::cout << "  ./deploy_test_jetson 192.168.1.100 8888 --frequency 1.0       # 更快频率" << std::endl;
        std::cout << "  ./deploy_test_jetson 192.168.1.100 8888 --rate 20 --duration 60  # 20Hz测试60秒" << std::endl;
        return 1;
    }

    std::string server_ip = argv[1];
    int server_port = std::stoi(argv[2]);

    // 创建客户端实例
    NetworkCommandClient client(server_ip, server_port);

    // 解析可选参数
    for (int i = 3; i < argc; i += 2) {
        if (i + 1 < argc) {
            std::string param = argv[i];
            std::string value = argv[i + 1];

            if (param == "--amplitude") {
                client.SetAmplitude(std::stod(value));
            } else if (param == "--frequency") {
                client.SetFrequency(std::stod(value));
            } else if (param == "--rate") {
                client.SetPublishRate(std::stod(value));
            } else if (param == "--duration") {
                client.SetDuration(std::stod(value));
            }
        }
    }

    try {
        std::cout << "======================================" << std::endl;
        std::cout << "🤖 G1机器人网络正弦波测试工具 (Jetson客户端)" << std::endl;
        std::cout << "======================================" << std::endl;

        // 开始测试
        client.StartTest();

    } catch (const std::exception& e) {
        std::cerr << "❌ 错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}