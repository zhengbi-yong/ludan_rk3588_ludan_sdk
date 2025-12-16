#include <iostream>
#include <cmath>
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <iomanip>

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_IMU_TORSO = "rt/secondary_imu";
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

class SineWaveTest {
private:
    ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
    ChannelSubscriberPtr<LowState_> lowstate_subscriber_;

    std::atomic<bool> should_run_{false};
    std::thread test_thread_;
    std::thread monitor_thread_;

    // 运动参数
    double amplitude_;      // 正弦波幅度 (rad)
    double frequency_;      // 正弦波频率 (Hz)
    double publish_rate_;   // 发布频率 (Hz)
    double duration_;       // 测试持续时间 (seconds)

    // 状态变量
    uint64_t command_count_;
    uint64_t state_count_;
    std::chrono::high_resolution_clock::time_point start_time_;

    // 目标关节配置
    struct JointConfig {
        int g1_id;           // G1关节索引
        std::string name;    // 关节名称
        double phase_offset; // 相位偏移
        double kp;           // 位置增益
        double kd;           // 阻尼增益
    };

    std::vector<JointConfig> target_joints_;

public:
    SineWaveTest()
        : amplitude_(0.3),       // 0.3 rad ≈ 17度
          frequency_(0.5),        // 0.5 Hz 正弦波
          publish_rate_(50.0),    // 50 Hz 发布频率
          duration_(30.0),        // 30秒测试时间
          command_count_(0),
          state_count_(0) {

        // 配置目标关节 (踝关节)
        target_joints_ = {
            {4, "LeftAnklePitch", 0.0, 40.0, 1.0},    // 左踝关节俯仰
            {5, "LeftAnkleRoll", 0.0, 40.0, 1.0},     // 左踝关节滚转
            {10, "RightAnklePitch", M_PI/2, 40.0, 1.0}, // 右踝关节俯仰 (180度相位差)
            {11, "RightAnkleRoll", 0.0, 40.0, 1.0}     // 右踝关节滚转
        };

        std::cout << "🌊 正弦波测试工具初始化完成" << std::endl;
        std::cout << "📊 测试参数:" << std::endl;
        std::cout << "   - 幅度: " << amplitude_ << " rad (" << amplitude_ * 180.0 / M_PI << "°)" << std::endl;
        std::cout << "   - 频率: " << frequency_ << " Hz" << std::endl;
        std::cout << "   - 发布频率: " << publish_rate_ << " Hz" << std::endl;
        std::cout << "   - 持续时间: " << duration_ << " 秒" << std::endl;
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
        lowstate_subscriber_->InitChannel(std::bind(&SineWaveTest::LowStateHandler, this, std::placeholders::_1), 1);
        std::cout << "✅ LowState订阅器已创建 (话题: " << HG_STATE_TOPIC << ")" << std::endl;
    }

    void LowStateHandler(const void *message) {
        state_count_++;

        // 每100个状态消息打印一次
        if (state_count_ % 100 == 0) {
            std::cout << "📥 接收到状态消息 #" << state_count_ << std::endl;
        }
    }

    void GenerateSineWaveCommand(double time, LowCmd_ &cmd) {
        cmd.mode_pr() = 1;  // PR模式
        cmd.mode_machine() = 1;  // G1类型

        for (size_t i = 0; i < G1_NUM_MOTOR; i++) {
            cmd.motor_cmd().at(i).mode() = 1;  // Enable

            // 检查是否是目标关节
            bool is_target = false;
            for (const auto& joint : target_joints_) {
                if (i == joint.g1_id) {
                    is_target = true;

                    // 生成正弦波位置命令
                    double phase = 2.0 * M_PI * frequency_ * time + joint.phase_offset;
                    double sine_value = amplitude_ * std::sin(phase);

                    cmd.motor_cmd().at(i).q() = sine_value;        // 位置
                    cmd.motor_cmd().at(i).dq() = amplitude_ * 2.0 * M_PI * frequency_ * std::cos(phase);  // 速度
                    cmd.motor_cmd().at(i).kp() = joint.kp;           // 位置增益
                    cmd.motor_cmd().at(i).kd() = joint.kd;           // 阻尼增益
                    cmd.motor_cmd().at(i).tau() = 0.0;               // 扭矩
                    break;
                }
            }

            // 非目标关节设置为零位
            if (!is_target) {
                cmd.motor_cmd().at(i).q() = 0.0;
                cmd.motor_cmd().at(i).dq() = 0.0;
                cmd.motor_cmd().at(i).kp() = 0.0;
                cmd.motor_cmd().at(i).kd() = 0.0;
                cmd.motor_cmd().at(i).tau() = 0.0;
            }
        }
    }

    void TestThread() {
        std::cout << "🚀 启动正弦波测试线程" << std::endl;

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
                LowCmd_ dds_low_command;
                GenerateSineWaveCommand(elapsed, dds_low_command);

                // 计算CRC
                dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);

                // 发布命令
                lowcmd_publisher_->Write(dds_low_command);
                command_count_++;

                // 调试输出 (每50个命令)
                if (command_count_ % 50 == 0) {
                    // 显示当前正弦波值
                    double phase = 2.0 * M_PI * frequency_ * elapsed;
                    double sine_value = amplitude_ * std::sin(phase);

                    std::cout << "🌊 ["
                             << std::fixed << std::setprecision(2) << elapsed << "s] "
                             << "CMD #" << std::setw(4) << command_count_ << " | "
                             << "正弦值: " << std::setw(6) << std::setprecision(3) << sine_value << " rad ("
                             << std::setw(5) << std::setprecision(1) << sine_value * 180.0 / M_PI << "°) | "
                             << "频率: " << command_count_ / elapsed << " Hz"
                             << std::endl;
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

        std::cout << "🛑 正弦波测试线程结束" << std::endl;
    }

    void MonitorThread() {
        std::cout << "📊 启动监控线程" << std::endl;

        while (should_run_) {
            std::this_thread::sleep_for(std::chrono::seconds(2));

            if (command_count_ > 0 && state_count_ > 0) {
                auto elapsed = std::chrono::duration<double>(
                    std::chrono::high_resolution_clock::now() - start_time_).count();

                double cmd_hz = command_count_ / elapsed;
                double state_hz = state_count_ / elapsed;

                std::cout << "📈 实时统计 ["
                         << std::fixed << std::setprecision(1) << elapsed << "s] - "
                         << "命令: " << std::setw(3) << cmd_hz << "Hz, "
                         << "状态: " << std::setw(3) << state_hz << "Hz"
                         << std::endl;
            }
        }

        std::cout << "📊 监控线程结束" << std::endl;
    }

    void StartTest() {
        std::cout << "\n🎯 开始正弦波运动测试" << std::endl;
        std::cout << "======================================" << std::endl;

        should_run_ = true;
        command_count_ = 0;
        state_count_ = 0;
        start_time_ = std::chrono::high_resolution_clock::now();

        // 启动测试线程
        test_thread_ = std::thread(&SineWaveTest::TestThread, this);

        // 启动监控线程
        monitor_thread_ = std::thread(&SineWaveTest::MonitorThread, this);

        std::cout << "📋 测试配置:" << std::endl;
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
        test_thread_.join();

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
        std::cout << "总状态数: " << state_count_ << std::endl;

        if (elapsed > 0) {
            std::cout << "平均命令频率: " << command_count_ / elapsed << " Hz" << std::endl;
            std::cout << "平均状态频率: " << state_count_ / elapsed << " Hz" << std::endl;
        }

        std::cout << "======================================" << std::endl;

        if (elapsed >= duration_ - 1.0) {
            std::cout << "✅ 测试完成 - 运行了完整的" << duration_ << "秒" << std::endl;
        } else {
            std::cout << "⚠️  测试提前结束 - 运行了" << elapsed << "秒" << std::endl;
        }

        // 生成正弦波数据报告
        std::cout << "\n🌊 正弦波参数验证:" << std::endl;
        std::cout << "   预期周期数: " << duration_ * frequency_ << std::endl;
        std::cout << "   实际周期数: " << command_count_ / (publish_rate_ / frequency_) << std::endl;
    }

    ~SineWaveTest() {
        should_run_ = false;
        if (test_thread_.joinable()) {
            test_thread_.join();
        }
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }
    }
};

int main(int argc, char const *argv[]) {
    if (argc < 2) {
        std::cout << "用法: deploy_test <network_interface> [options]" << std::endl;
        std::cout << std::endl;
        std::cout << "参数说明:" << std::endl;
        std::cout << "  network_interface  DDS网络接口 (如: veth0, eth0)" << std::endl;
        std::cout << std::endl;
        std::cout << "可选参数:" << std::endl;
        std::cout << "  --amplitude <rad> 正弦波幅度 (默认: 0.3 rad)" << std::endl;
        std::cout << "  --frequency <hz>  正弦波频率 (默认: 0.5 Hz)" << std::endl;
        std::cout << "  --rate <hz>       发布频率 (默认: 50 Hz)" << std::endl;
        std::cout << "  --duration <s>    测试持续时间 (默认: 30 秒)" << std::endl;
        std::cout << std::endl;
        std::cout << "示例:" << std::endl;
        std::cout << "  ./deploy_test veth0                    # 使用默认参数" << std::endl;
        std::cout << "  ./deploy_test veth0 --amplitude 0.5      # 更大幅度" << std::endl;
        std::cout << "  ./deploy_test veth0 --frequency 1.0       # 更快频率" << std::endl;
        std::cout << "  ./deploy_test veth0 --rate 20 --duration 60  # 20Hz测试60秒" << std::endl;
        return 1;
    }

    std::string network_interface = argv[1];

    // 创建测试实例
    SineWaveTest test;

    // 解析可选参数
    for (int i = 2; i < argc; i += 2) {
        if (i + 1 < argc) {
            std::string param = argv[i];
            std::string value = argv[i + 1];

            if (param == "--amplitude") {
                test.SetAmplitude(std::stod(value));
            } else if (param == "--frequency") {
                test.SetFrequency(std::stod(value));
            } else if (param == "--rate") {
                test.SetPublishRate(std::stod(value));
            } else if (param == "--duration") {
                test.SetDuration(std::stod(value));
            }
        }
    }

    try {
        std::cout << "======================================" << std::endl;
        std::cout << "🤖 G1机器人正弦波测试工具" << std::endl;
        std::cout << "======================================" << std::endl;

        // 初始化DDS
        test.InitializeDDS(network_interface);

        // 开始测试
        test.StartTest();

    } catch (const std::exception& e) {
        std::cerr << "❌ 错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}