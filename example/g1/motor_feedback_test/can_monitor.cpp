// can_monitor.cpp
// CAN总线监控工具 - 显示所有接收到的CAN帧

#include <iostream>
#include <iomanip>
#include <cstring>
#include <chrono>
#include <thread>
#include <signal.h>
#include <atomic>

// ZLG CANFDNET SDK
#include "CANFDNET.h"

// ==================== 配置 ====================
struct Config {
    std::string zlg_ip = "192.168.1.5";
    int zlg_port = 8002;
    int channel = 2;
    int arb_baud = 1000000;   // 1M bps
    int data_baud = 5000000;  // 5M bps
    bool show_all = true;      // 显示所有CAN帧
};

// ==================== 全局变量 ====================
std::atomic<bool> running{true};
DEVICE_HANDLE g_device_handle = nullptr;
CHANNEL_HANDLE g_channel_handle = nullptr;

// ==================== 信号处理 ====================
void SignalHandler(int signal) {
    std::cout << "\n收到信号 " << signal << ", 正在停止..." << std::endl;
    running = false;
}

// ==================== 打印CAN帧 ====================
void PrintCANFrame(const ZCAN_ReceiveFD_Data& frame, int count) {
    uint32_t can_id = frame.frame.can_id & 0x7FF;

    std::cout << "\r[";
    std::cout << std::setw(6) << count << "] ";
    std::cout << "CAN_ID: 0x" << std::hex << std::setw(3) << std::setfill('0') << can_id << std::dec << std::setfill(' ');
    std::cout << " (" << std::setw(3) << can_id << ") ";

    if (frame.frame.len > 0) {
        std::cout << " Data: ";
        for (uint8_t i = 0; i < frame.frame.len && i < 8; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)frame.frame.data[i] << " " << std::dec;
        }
    }

    // 如果是电机反馈帧 (ID 1-30, 长度8字节)
    if (can_id >= 1 && can_id <= 30 && frame.frame.len >= 8) {
        const uint8_t* d = frame.frame.data;

        uint8_t motor_id = d[0] & 0x0F;
        uint8_t error = (d[0] >> 4) & 0x0F;

        int16_t pos_raw = (int16_t)((d[1] << 8) | d[2]);
        int16_t vel_raw = ((d[3] & 0xFF) << 4) | (d[4] & 0x0F);
        int16_t tor_raw = (((d[4] >> 4) & 0x0F) << 8) | d[5];
        int8_t temp_mos = (int8_t)d[6];
        int8_t temp_rotor = (int8_t)d[7];

        // 转换为物理值
        double pos = pos_raw * (12.5 - (-12.5)) / 65535.0 + (-12.5);
        double vel = vel_raw * (30.0 - (-30.0)) / 4095.0 + (-30.0);
        double tor = tor_raw * (10.0 - (-10.0)) / 4095.0 + (-10.0);

        std::cout << " <== MOTOR FB: ID=" << (int)motor_id
                  << " ERR=" << (int)error
                  << " P=" << std::fixed << std::setprecision(2) << pos
                  << " V=" << vel
                  << " T=" << tor
                  << " Tmos=" << (int)temp_mos << "C";
    }

    std::cout << "         " << std::flush;
}

// ==================== 主函数 ====================
int main(int argc, char** argv) {
    Config config;

    // 解析命令行参数
    if (argc >= 2) {
        config.channel = std::atoi(argv[1]);
    }

    std::cout << "========================================" << std::endl;
    std::cout << "    CAN总线监控工具" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "ZLG: " << config.zlg_ip << ":" << config.zlg_port << std::endl;
    std::cout << "Channel: CAN" << config.channel << std::endl;
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
    std::cout << "📡 开始监控CAN总线..." << std::endl;
    std::cout << "按 Ctrl+C 停止" << std::endl;
    std::cout << "提示: 请先使能电机并发送控制命令" << std::endl;
    std::cout << "========================================" << std::endl;

    // 接收缓冲区
    ZCAN_ReceiveFD_Data receive_buffer[100];
    int total_count = 0;
    int motor_fb_count = 0;
    auto start_time = std::chrono::steady_clock::now();
    uint32_t can_ids_seen[256] = {0};  // 记录见过的CAN ID

    while (running) {
        // 接收CAN数据
        uint32_t received = ZCAN_ReceiveFD(g_channel_handle, receive_buffer, 100, 10);

        if (received > 0) {
            total_count += received;

            for (uint32_t i = 0; i < received; i++) {
                uint32_t can_id = receive_buffer[i].frame.can_id & 0x7FF;
                if (can_id < 256) {
                    can_ids_seen[can_id]++;
                }

                // 打印CAN帧
                PrintCANFrame(receive_buffer[i], total_count - received + i + 1);

                // 检查是否是电机反馈帧
                if (can_id >= 1 && can_id <= 30 && receive_buffer[i].frame.len >= 8) {
                    motor_fb_count++;
                }
            }

            std::cout << std::endl;
        }

        // 每100帧打印一次统计
        if (total_count > 0 && total_count % 100 == 0) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
            if (elapsed > 0) {
                std::cout << "\n========== 统计 ==========" << std::endl;
                std::cout << "总接收: " << total_count << " 帧率: " << (total_count / elapsed) << " fps" << std::endl;
                std::cout << "电机反馈: " << motor_fb_count << std::endl;
                std::cout << "见过的CAN ID: ";
                for (int i = 0; i < 256; i++) {
                    if (can_ids_seen[i] > 0) {
                        std::cout << i << "(" << can_ids_seen[i] << ") ";
                    }
                }
                std::cout << "\n==========================" << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // 清理
    std::cout << "\n========================================" << std::endl;
    std::cout << "正在关闭..." << std::endl;
    std::cout << "总接收帧数: " << total_count << std::endl;
    std::cout << "电机反馈帧数: " << motor_fb_count << std::endl;
    std::cout << "见过的CAN ID: ";
    for (int i = 0; i < 256; i++) {
        if (can_ids_seen[i] > 0) {
            std::cout << i << "(" << can_ids_seen[i] << ") ";
        }
    }
    std::cout << std::endl;

    if (g_channel_handle) {
        ZCAN_ResetCAN(g_channel_handle);
    }
    if (g_device_handle) {
        ZCAN_CloseDevice(g_device_handle);
    }

    std::cout << "✅ 程序已退出" << std::endl;

    return 0;
}
