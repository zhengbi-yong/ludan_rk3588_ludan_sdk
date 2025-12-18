#include <iostream>
#include <chrono>
#include <thread>
#include <map>
#include <vector>
#include <atomic>
#include <signal.h>

// DDS
#include <unitree/robot/channel/channel_factory.hpp>

using namespace unitree::common;
using namespace unitree::robot;

// 全局变量用于优雅退出
std::atomic<bool> running(true);

void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    running = false;
}

// 主题信息结构
struct TopicInfo {
    std::string name;
    std::string type;
    uint64_t message_count = 0;
    std::chrono::steady_clock::time_point last_received;
    double frequency = 0.0;
};

std::map<std::string, TopicInfo> observed_topics;

// 通用消息处理器模板
template<typename T>
class TopicMonitor {
private:
    std::string topic_name;
    std::string topic_type;

public:
    TopicMonitor(const std::string& name, const std::string& type)
        : topic_name(name), topic_type(type) {}

    void messageHandler(const void* message) {
        auto now = std::chrono::steady_clock::now();

        if (observed_topics.find(topic_name) == observed_topics.end()) {
            observed_topics[topic_name] = {topic_name, topic_type, 0, now, 0.0};
            std::cout << "✓ 发现新主题: " << topic_name << " (类型: " << topic_type << ")" << std::endl;
        }

        auto& topic_info = observed_topics[topic_name];
        topic_info.message_count++;
        topic_info.last_received = now;

        // 简单频率计算
        static auto last_print = std::chrono::steady_clock::now();
        auto now_print = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now_print - last_print).count() >= 5) {
            topic_info.frequency = topic_info.message_count / 5.0;
            topic_info.message_count = 0;
            last_print = now_print;
        }
    }
};

// 已知的Unitree主题定义
const std::vector<std::pair<std::string, std::string>> known_topics = {
    // G1/H1 机器人主题
    {"rt/lowcmd", "unitree_hg::msg::dds_::LowCmd_"},
    {"rt/lowstate", "unitree_hg::msg::dds_::LowState_"},
    {"rt/imu_torso", "unitree_hg::msg::dds_::IMU_"},

    // GO2/B2 机器人主题
    {"rt/lowcmd", "unitree_go::msg::dds_::LowCmd_"},
    {"rt/lowstate", "unitree_go::msg::dds_::LowState_"},
    {"rt/imu", "unitree_go::msg::dds_::IMU_"},
    {"rt/bms_cmd", "unitree_go::msg::dds_::BmsCmd_"},
    {"rt/bms_state", "unitree_go::msg::dds_::BmsState_"},
    {"rt/sport_modestate", "unitree_go::msg::dds_::SportModeState_"},
    {"rt/sport_modereq", "unitree_go::msg::dds_::SportModeReq_"},

    // 相机主题
    {"utcdr/image/request", "unitree::robot::dds::proto::ImageRequest_"},
    {"utcdr/image/response", "unitree::robot::dds::proto::ImageResponse_"},

    // 音频主题
    {"utcdr/audio/request", "unitree::robot::dds::proto::AudioRequest_"},
    {"utcdr/audio/response", "unitree::robot::dds::proto::AudioResponse_"},

    // Lidar主题
    {"utcdr/lidar/request", "unitree::robot::dds::proto::LidarRequest_"},
    {"utcdr/lidar/response", "unitree::robot::dds::proto::LidarResponse_"},

    // 测试主题
    {"helloworld", "HelloWorldData"}
};

int main(int argc, char const *argv[]) {
    if (argc < 2) {
        std::cout << "DDS Topics Monitor" << std::endl;
        std::cout << "Usage: " << argv[0] << " network_interface" << std::endl;
        std::cout << std::endl;
        std::cout << "此工具用于监控所有DDS主题的活跃状态" << std::endl;
        std::cout << "包括自动发现的活跃主题和已知的主题列表" << std::endl;
        std::cout << std::endl;
        std::cout << "示例:" << std::endl;
        std::cout << "  " << argv[0] << " lo     # 本地回环接口" << std::endl;
        std::cout << "  " << argv[0] << " eth0   # 以太网接口" << std::endl;
        return 0;
    }

    std::string networkInterface = argv[1];

    // 设置信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    try {
        // 初始化DDS通道工厂
        ChannelFactory::Instance()->Init(0, networkInterface);
        std::cout << "DDS通道已初始化，网络接口: " << networkInterface << std::endl;
        std::cout << std::endl;

        std::cout << "已知的Unitree DDS主题:" << std::endl;
        std::cout << "================================" << std::endl;
        for (const auto& topic : known_topics) {
            std::cout << "  主题: " << topic.first << std::endl;
            std::cout << "  类型: " << topic.second << std::endl;
            std::cout << "  --------------------------------" << std::endl;
        }
        std::cout << "总计: " << known_topics.size() << " 个已知主题" << std::endl;
        std::cout << std::endl;

        std::cout << "正在监控活跃的DDS主题..." << std::endl;
        std::cout << "按 Ctrl+C 停止监控并显示统计信息" << std::endl;
        std::cout << std::endl;

        // 监控循环
        while (running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // 显示统计结果
        std::cout << std::endl;
        std::cout << "监控结束！统计结果:" << std::endl;
        std::cout << "================================" << std::endl;

        if (observed_topics.empty()) {
            std::cout << "⚠️  未发现任何活跃的DDS主题" << std::endl;
            std::cout << "这可能是因为:" << std::endl;
            std::cout << "  1. 没有发布者正在发布消息" << std::endl;
            std::cout << "  2. 网络接口配置不正确" << std::endl;
            std::cout << "  3. DDS配置问题" << std::endl;
        } else {
            std::cout << "发现的活跃主题:" << std::endl;
            std::cout << std::endl;

            for (const auto& [name, info] : observed_topics) {
                std::cout << "📡 主题: " << name << std::endl;
                std::cout << "   类型: " << info.type << std::endl;
                std::cout << "   频率: " << std::fixed << std::setprecision(1) << info.frequency << " Hz" << std::endl;
                std::cout << "   状态: ";
                if (info.frequency > 0) {
                    std::cout << "🟢 活跃";
                } else {
                    std::cout << "🔴 静默";
                }
                std::cout << std::endl;
                std::cout << std::endl;
            }

            std::cout << "✅ 总计发现 " << observed_topics.size() << " 个活跃主题" << std::endl;
        }

        std::cout << std::endl;
        std::cout << "总结:" << std::endl;
        std::cout << "  - 已知主题总数: " << known_topics.size() << std::endl;
        std::cout << "  - 活跃主题总数: " << observed_topics.size() << std::endl;
        std::cout << "  - 静默主题总数: " << (known_topics.size() - observed_topics.size()) << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "❌ 错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}