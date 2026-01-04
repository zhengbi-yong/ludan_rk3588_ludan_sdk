// motor_feedback_reader.cpp
// 简单的DM电机反馈数据接收和显示程序

#include <iostream>
#include <iomanip>
#include <cstring>
#include <chrono>
#include <thread>
#include <signal.h>
#include <atomic>
#include <fstream>
#include <sstream>
#include <ctime>
#include <sys/stat.h>

// ZLG CANFDNET SDK
#include "CANFDNET.h"

// ==================== 配置 ====================
struct Config {
    std::string zlg_ip = "192.168.1.5";
    int zlg_port = 8002;
    int channel = 1;
    int arb_baud = 1000000;   // 1M bps
    int data_baud = 5000000;  // 5M bps
    int target_motor_id = 9;  // 目标电机ID
    std::string log_dir = "log";  // 日志目录
};

// ==================== DM电机反馈数据结构 ====================
struct DMMotorFeedback {
    // 解码后的值
    uint8_t  motor_id;
    uint8_t  error;
    int16_t  position_raw;
    int16_t  velocity_raw;
    int16_t  torque_raw;
    int8_t   temp_mos;
    int8_t   temp_rotor;

    // 物理值
    double position_rad;
    double velocity_rad_s;
    double torque_nm;

    // 时间戳
    uint64_t timestamp_ms;

    bool valid;
};

// ==================== DM电机参数范围 (DM4310) ====================
class DMMotorDecoder {
public:
    // DM4310 范围
    static constexpr double P_MIN = -12.5f;    // 位置最小值
    static constexpr double P_MAX = 12.5f;     // 位置最大值
    static constexpr double V_MIN = -30.0f;    // 速度最小值
    static constexpr double V_MAX = 30.0f;     // 速度最大值
    static constexpr double T_MIN = -10.0f;    // 力矩最小值
    static constexpr double T_MAX = 10.0f;     // 力矩最大值

    static DMMotorFeedback DecodeFrame(const ZCAN_ReceiveFD_Data& frame) {
        DMMotorFeedback fb;
        memset(&fb, 0, sizeof(fb));

        const uint8_t* d = frame.frame.data;

        // 检查数据长度
        if (frame.frame.len < 8) {
            fb.valid = false;
            return fb;
        }

        // 字节0: [ERR(4bit)] [ID(4bit)]
        fb.motor_id = d[0] & 0x0F;
        fb.error = (d[0] >> 4) & 0x0F;

        // 字节1-2: 位置 (16位)
        fb.position_raw = static_cast<int16_t>((d[1] << 8) | d[2]);
        // 反量化: position = raw * (P_MAX - P_MIN) / 65535.0 + P_MIN
        fb.position_rad = static_cast<double>(fb.position_raw) * (P_MAX - P_MIN) / 65535.0 + P_MIN;

        // 字节3-4: 速度 (12位)
        int16_t vel_encoded = ((d[3] & 0xFF) << 4) | (d[4] & 0x0F);
        fb.velocity_raw = vel_encoded;
        // 反量化: velocity = raw * (V_MAX - V_MIN) / 4095.0 + V_MIN
        fb.velocity_rad_s = static_cast<double>(vel_encoded) * (V_MAX - V_MIN) / 4095.0 + V_MIN;

        // 字节4-5: 力矩 (12位)
        int16_t torque_encoded = (((d[4] >> 4) & 0x0F) << 8) | d[5];
        fb.torque_raw = torque_encoded;
        // 反量化: torque = raw * (T_MAX - T_MIN) / 4095.0 + T_MIN
        fb.torque_nm = static_cast<double>(torque_encoded) * (T_MAX - T_MIN) / 4095.0 + T_MIN;

        // 字节6-7: 温度
        fb.temp_mos = static_cast<int8_t>(d[6]);
        fb.temp_rotor = static_cast<int8_t>(d[7]);

        fb.valid = true;

        return fb;
    }
};

// ==================== 全局变量 ====================
std::atomic<bool> running{true};
DEVICE_HANDLE g_device_handle = nullptr;
CHANNEL_HANDLE g_channel_handle = nullptr;

// 日志文件
std::ofstream g_log_file;
std::string g_log_file_path;
double g_last_timestamp = 0.0;
int g_log_count = 0;

// ==================== 日志功能 ====================
void SetupLogging(const Config& config) {
    // 创建日志目录
    std::string mkdir_cmd = "mkdir -p " + config.log_dir;
    system(mkdir_cmd.c_str());

    // 生成日志文件名（带时间戳）
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << config.log_dir << "/motor_log_" << config.target_motor_id << "_"
       << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S") << ".csv";
    g_log_file_path = ss.str();

    // 打开日志文件
    g_log_file.open(g_log_file_path);
    if (g_log_file.is_open()) {
        // 写入CSV表头
        g_log_file << "timestamp,motor_id,error,position,velocity,torque,raw_pos,raw_vel,raw_torque,temp_mos,temp_rotor,frequency\n";
        g_log_file << std::fixed << std::setprecision(6);
        std::cout << "✅ 日志文件: " << g_log_file_path << std::endl;
    } else {
        std::cerr << "❌ 无法打开日志文件: " << g_log_file_path << std::endl;
    }
}

void LogFeedback(const DMMotorFeedback& fb) {
    if (!g_log_file.is_open()) return;

    // 获取当前时间戳（秒）
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto epoch = now_ms.time_since_epoch();
    double timestamp = std::chrono::duration<double>(epoch).count();

    // 计算频率
    double frequency = 0.0;
    if (g_last_timestamp > 0.0 && (timestamp - g_last_timestamp) > 0.001) {
        frequency = 1.0 / (timestamp - g_last_timestamp);
    }
    g_last_timestamp = timestamp;

    // 写入CSV行
    g_log_file << timestamp << ","
               << (int)fb.motor_id << ","
               << (int)fb.error << ","
               << fb.position_rad << ","
               << fb.velocity_rad_s << ","
               << fb.torque_nm << ","
               << fb.position_raw << ","
               << fb.velocity_raw << ","
               << fb.torque_raw << ","
               << (int)fb.temp_mos << ","
               << (int)fb.temp_rotor << ","
               << frequency << "\n";

    g_log_count++;

    // 每100行刷新一次
    if (g_log_count % 100 == 0) {
        g_log_file.flush();
    }
}

void CloseLogging() {
    if (g_log_file.is_open()) {
        g_log_file.close();
        std::cout << "✅ 日志已保存: " << g_log_file_path << " (共" << g_log_count << "条记录)" << std::endl;
    }
}

// ==================== 信号处理 ====================
void SignalHandler(int signal) {
    std::cout << "\n收到信号 " << signal << ", 正在停止..." << std::endl;
    running = false;
}

// ==================== 打印反馈数据 ====================
void PrintFeedback(const DMMotorFeedback& fb, int count) {
    std::cout << "\r";
    std::cout << "[" << std::setw(6) << count << "] ";
    std::cout << "ID:" << std::setw(2) << (int)fb.motor_id << " ";
    std::cout << "ERR:" << std::setw(2) << (int)fb.error << " | ";

    std::cout << "Pos:" << std::fixed << std::setprecision(3) << std::setw(8) << fb.position_rad << " rad ";
    std::cout << "(" << std::showpos << std::setw(6) << fb.position_raw << std::noshowpos << ") ";

    std::cout << "Vel:" << std::fixed << std::setprecision(2) << std::setw(6) << fb.velocity_rad_s << " r/s ";
    std::cout << "(" << std::showpos << std::setw(5) << fb.velocity_raw << std::noshowpos << ") ";

    std::cout << "Tor:" << std::fixed << std::setprecision(3) << std::setw(6) << fb.torque_nm << " Nm ";
    std::cout << "(" << std::showpos << std::setw(5) << fb.torque_raw << std::noshowpos << ") ";

    std::cout << "Tmos:" << std::setw(3) << (int)fb.temp_mos << "°C ";
    std::cout << "Trot:" << std::setw(3) << (int)fb.temp_rotor << "°C";

    std::cout << std::flush;
}

// ==================== 主函数 ====================
int main(int argc, char** argv) {
    Config config;

    // 解析命令行参数
    if (argc >= 2) {
        config.target_motor_id = std::atoi(argv[1]);
    }
    if (argc >= 3) {
        config.channel = std::atoi(argv[2]);
    }

    std::cout << "========================================" << std::endl;
    std::cout << "    DM电机反馈数据读取器" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "ZLG: " << config.zlg_ip << ":" << config.zlg_port << std::endl;
    std::cout << "Channel: CAN" << config.channel << std::endl;
    std::cout << "目标电机ID: " << config.target_motor_id << std::endl;
    std::cout << "========================================" << std::endl;

    // 设置信号处理
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    // 1. 打开设备
    std::cout << "[1/4] 打开ZCAN设备..." << std::endl;
    g_device_handle = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, 0, 0);
    if (g_device_handle == INVALID_DEVICE_HANDLE) {
        std::cerr << "❌ 打开设备失败" << std::endl;
        return 1;
    }
    std::cout << "✅ 设备已打开" << std::endl;

    // 2. 初始化CAN通道
    std::cout << "[2/4] 初始化CAN通道..." << std::endl;
    ZCAN_CHANNEL_INIT_CONFIG init_config;
    memset(&init_config, 0, sizeof(init_config));
    init_config.can_type = TYPE_CANFD;
    init_config.canfd.acc_code = 0;  // 接收所有CAN ID
    init_config.canfd.acc_mask = 0;  // 接收所有CAN ID
    init_config.canfd.abit_timing = config.arb_baud;
    init_config.canfd.dbit_timing = config.data_baud;
    init_config.canfd.brp = 0;
    init_config.canfd.filter = 0;
    init_config.canfd.mode = 0;

    g_channel_handle = ZCAN_InitCAN(g_device_handle, config.channel, &init_config);
    if (g_channel_handle == INVALID_CHANNEL_HANDLE) {
        std::cerr << "❌ 初始化CAN通道失败" << std::endl;
        ZCAN_CloseDevice(g_device_handle);
        return 1;
    }
    std::cout << "✅ CAN通道已初始化 (CAN-FD " << config.arb_baud << "/" << config.data_baud << ")" << std::endl;

    // 3. 配置网络
    std::cout << "[3/4] 配置TCP客户端..." << std::endl;
    uint32_t val = 1;
    ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, 0, SETREF_SET_DATA_RECV_MERGE, &val);

    val = 0;  // TCP client模式
    ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, 0, CMD_TCP_TYPE, &val);
    ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, 0, CMD_DESIP, (void*)config.zlg_ip.c_str());
    val = config.zlg_port;
    ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, 0, CMD_DESPORT, &val);
    std::cout << "✅ TCP客户端已配置" << std::endl;

    // 4. 启动CAN
    std::cout << "[4/4] 启动CAN..." << std::endl;
    if (ZCAN_StartCAN(g_channel_handle) != STATUS_OK) {
        std::cerr << "❌ 启动CAN失败" << std::endl;
        ZCAN_CloseDevice(g_device_handle);
        return 1;
    }
    std::cout << "✅ CAN已启动" << std::endl;
    std::cout << "========================================" << std::endl;

    // 5. 设置日志
    SetupLogging(config);

    std::cout << "========================================" << std::endl;
    std::cout << "📡 开始接收电机反馈数据..." << std::endl;
    std::cout << "按 Ctrl+C 停止" << std::endl;
    std::cout << "========================================" << std::endl;

    // 接收缓冲区 - 同时支持标准CAN和CAN-FD
    ZCAN_ReceiveFD_Data canfd_buffer[50];
    ZCAN_Receive_Data can_buffer[50];
    DMMotorDecoder decoder;
    int total_count = 0;
    int motor_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (running) {
        // 先接收标准CAN帧 (DM电机发送的是标准CAN)
        uint32_t can_received = ZCAN_Receive(g_channel_handle, can_buffer, 50, 5);

        if (can_received > 0) {
            total_count += can_received;

            for (uint32_t i = 0; i < can_received; i++) {
                uint32_t can_id = can_buffer[i].frame.can_id & 0x7FF;

                // 检查是否是我们想要的电机 (ID 1-30)
                if (can_id >= 1 && can_id <= 30 && can_buffer[i].frame.can_dlc >= 8) {
                    // 转换标准CAN到CAN-FD结构用于解码
                    ZCAN_ReceiveFD_Data fd_frame;
                    memset(&fd_frame, 0, sizeof(fd_frame));
                    fd_frame.frame.can_id = can_buffer[i].frame.can_id;
                    fd_frame.frame.len = can_buffer[i].frame.can_dlc;
                    memcpy(fd_frame.frame.data, can_buffer[i].frame.data, 8);

                    DMMotorFeedback fb = decoder.DecodeFrame(fd_frame);

                    if (fb.valid) {
                        motor_count++;

                        // 写入日志（记录所有电机数据）
                        LogFeedback(fb);

                        // 只显示目标电机的数据，或者显示所有电机的数据
                        if (config.target_motor_id == 0 || fb.motor_id == config.target_motor_id) {
                            PrintFeedback(fb, motor_count);
                        }
                    }
                }
            }
        }

        // 再接收CAN-FD帧（如果有）
        uint32_t canfd_received = ZCAN_ReceiveFD(g_channel_handle, canfd_buffer, 50, 5);

        if (canfd_received > 0) {
            total_count += canfd_received;

            for (uint32_t i = 0; i < canfd_received; i++) {
                uint32_t can_id = canfd_buffer[i].frame.can_id & 0x7FF;

                // 检查是否是我们想要的电机 (ID 1-30)
                if (can_id >= 1 && can_id <= 30 && canfd_buffer[i].frame.len >= 8) {
                    DMMotorFeedback fb = decoder.DecodeFrame(canfd_buffer[i]);

                    if (fb.valid) {
                        motor_count++;

                        // 写入日志（记录所有电机数据）
                        LogFeedback(fb);

                        // 只显示目标电机的数据，或者显示所有电机的数据
                        if (config.target_motor_id == 0 || fb.motor_id == config.target_motor_id) {
                            PrintFeedback(fb, motor_count);
                        }
                    }
                }
            }
        }

        // 每100帧打印一次统计
        if (total_count > 0 && total_count % 100 == 0) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
            if (elapsed > 0) {
                std::cout << "\n[统计] 总接收: " << total_count
                          << " 帧率: " << (total_count / elapsed) << " fps" << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // 清理
    std::cout << "\n========================================" << std::endl;
    std::cout << "正在关闭..." << std::endl;
    std::cout << "总接收帧数: " << total_count << std::endl;
    std::cout << "电机反馈帧数: " << motor_count << std::endl;

    // 关闭日志文件
    CloseLogging();

    if (g_channel_handle) {
        ZCAN_ResetCAN(g_channel_handle);
    }
    if (g_device_handle) {
        ZCAN_CloseDevice(g_device_handle);
    }

    std::cout << "✅ 程序已退出" << std::endl;

    return 0;
}
