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
#include <set>
#include <array>
#include <ctime>
#include <iomanip>
#include <atomic>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <fcntl.h>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include "xixilowcmd/msg/low_cmd.hpp"
#include "xixilowcmd/msg/motor_cmd.hpp"

// ZLG CANFDNET SDK (using newly compiled ARM64 library)
// Note: CANFDNET.h must be included before zlgcan.h for extern "C" linkage
#include "CANFDNET.h"

// ==================== CAN ID 宏定义 ====================
// 这些宏用于构造和解析CAN帧的ID字段
#define MAKE_CAN_ID(id, src_id, seg_id, rtr) ((id & 0x1FFFFFFF))
#define GET_ID(can_id) (can_id & 0x1FFFFFFF)

static const std::string ROS2_CMD_TOPIC = "/lowcmd";
const int G1_NUM_MOTOR = 30;

// 全局控制：发送命令时是否自动使能电机
// false: 发送命令时电机保持失能状态（需要手动使能）
// true:  发送命令时自动使能电机
static bool g_auto_enable_on_send = false;

// ==================== 禁用 ZLG SDK 调试输出 ====================
// 在运行时将 ZLG 库的 [SYS] 输出重定向到 /dev/null
static void DisableZlgDebugOutput() {
    // 打开 /dev/null
    int devnull = open("/dev/null", O_WRONLY);
    if (devnull >= 0) {
        // 重定向 stdout 和 stderr 到 /dev/null
        // 注意：这也会屏蔽我们自己的输出，所以不推荐
        // close(1); dup2(devnull, 1);
        // close(2); dup2(devnull, 2);
        close(devnull);
    }
}

// 更好的方案：使用环境变量控制（如果 ZLG SDK 支持）
static inline void SetZlgQuietMode() {
    // 尝试设置可能的环境变量
    setenv("ZCAN_DEBUG", "0", 1);
    setenv("ZLG_QUIET", "1", 1);
    setenv("CANFDNET_DEBUG", "0", 1);
}

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

// 周立功设备配置 (使用ZCAN官方封装函数)
struct ZhilgongConfig {
    std::string remote_ip = "192.168.1.5";  // 周立功设备IP
    int remote_port = 8003;                // 周立功设备TCP端口
    int channel = 2;                       // CAN通道 (0-7, 默认2=CAN2)

    // ZCAN 设备和通道句柄
    DEVICE_HANDLE device_handle = INVALID_DEVICE_HANDLE;
    CHANNEL_HANDLE channel_handle = INVALID_CHANNEL_HANDLE;
    bool initialized = false;

    // CAN-FD 波特率配置
    int arbitration_baud = 1000000;  // 仲裁域波特率 (1M bps)
    int data_baud = 5000000;         // 数据域波特率 (5M bps)
};

// Motor command structure
struct MotorCommandCan {
    uint16_t motor_id;        // CAN ID (控制命令用) 或 global_motor_id (心跳命令用)
    float pos;
    float vel;
    float kp;
    float kd;
    float torq;
    bool is_heartbeat = false;  // true=心跳命令(0xCC), false=控制命令
    uint16_t global_motor_id = 0;  // 全局电机ID (1-30), 心跳命令需要用这个设置Data[0]
};

// ==================== Multi-Port Configuration ====================
// Port configuration for 4-port motor control (matches cantoudp.cpp mapping)
struct PortConfig {
    int port;         // TCP port (8000-8003)
    int motor_count;  // Number of motors on this port
    int motor_offset; // Starting motor index (0-based)
    int channel;      // CAN channel number
};

constexpr PortConfig PORT_CONFIGS[4] = {
    {8000, 10, 0, 0},   // Motors 1-10  -> Local CAN ID 1-10
    {8001, 7, 10, 1},   // Motors 11-17 -> Local CAN ID 1-7
    {8002, 7, 17, 2},   // Motors 18-24 -> Local CAN ID 1-7
    {8003, 6, 24, 3}    // Motors 25-30 -> Local CAN ID 1-6
};

// ==================== Motor Protocol Helper ====================
// 支持两种电机协议：
// 1. DM 协议（达妙电机）：使用 float_to_uint 线性缩放格式（无偏移）
// 2. MIT 协议：使用线性缩放格式

// 设置使用的协议类型（默认使用 DM 协议）
static bool g_use_dm_protocol = true;

// ==================== float_to_uint 和 uint_to_float 转换函数 ====================
// 这些函数与 DM 电机 C 代码中的转换逻辑完全一致

/**
 * @brief Convert float to unsigned integer with quantization (DM 电机协议)
 * @param x_float Input float value
 * @param x_min   Minimum value of the range
 * @param x_max   Maximum value of the range
 * @param bits    Number of bits for quantization
 * @return Quantized integer value (0 to (2^bits - 1))
 * @note Formula: int = (x_float - x_min) * (2^bits - 1) / (x_max - x_min)
 */
static inline int float_to_uint(float x_float, float x_min, float x_max, int bits) {
    // Clamp input value to valid range
    if (x_float < x_min) {
        x_float = x_min;
    } else if (x_float > x_max) {
        x_float = x_max;
    }

    // Convert float to unsigned int, given range and number of bits
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief Convert unsigned integer to float value (DM 电机协议)
 * @param x_int  Input integer value
 * @param x_min  Minimum value of the output range
 * @param x_max  Maximum value of the output range
 * @param bits   Number of bits used for quantization
 * @return Dequantized float value
 * @note Formula: float = x_int * (x_max - x_min) / (2^bits - 1) + x_min
 */
static inline float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// ==================== DM Motor Protocol (达妙电机 MIT 模式) ====================
// DM 电机 MIT 模式命令格式（与 C 代码中的 mit_ctrl 函数一致）:
// data[0] = (pos_tmp >> 8);     /* Position high byte */
// data[1] = pos_tmp;            /* Position low byte */
// data[2] = (vel_tmp >> 4);     /* Velocity high 8 bits */
// data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);  /* Velocity low 4 bits | Kp high 4 bits */
// data[4] = kp_tmp;             /* Kp low 8 bits */
// data[5] = (kd_tmp >> 4);      /* Kd high 4 bits */
// data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8); /* Kd low 4 bits | Torque high 4 bits */
// data[7] = tor_tmp;            /* Torque low 8 bits */

// ==================== 电机参数配置结构体 ====================
// 电机参数范围结构体，支持按电机ID区分不同的PMAX/VMAX/TMAX
struct MotorLimits {
    float p_min;   // Position min
    float p_max;   // Position max
    float v_min;   // Velocity min
    float v_max;   // Velocity max
    float t_min;   // Torque min
    float t_max;   // Torque max
    float kp_min;  // Kp min
    float kp_max;  // Kp max
    float kd_min;  // Kd min
    float kd_max;  // Kd max
};

// ==================== 电机限位配置 ====================
// 根据电机ID分组设置PMAX、VMAX、TMAX参数
// Group A: motorID 18,19,20,21,22,25,26,27,28 -> PMAX 12.5, VMAX 25, TMAX 200
// Group B: motorID 4,5,6,11,12,13,23,24,29,30 -> PMAX 12.566, VMAX 20, TMAX 120
// Group C: motorID 1,2,3,7,8,9,10,14,15,16,17 -> PMAX 12.5, VMAX 10, TMAX 28
static inline MotorLimits GetMotorLimits(uint16_t motor_id) {
    // 默认Kp/Kd限位
    float kp_min = 0.0f, kp_max = 500.0f;     // Kp范围: 0.0 到 500.0
    float kd_min = 0.0f, kd_max = 5.0f;       // Kd范围: 0.0 到 5.0

    // 根据电机ID分组设置PMAX、VMAX、TMAX
    float p_min, p_max, v_min, v_max, t_min, t_max;

    if ((motor_id >= 18 && motor_id <= 22) || (motor_id >= 25 && motor_id <= 28)) {
        // Group A: motorID 18,19,20,21,22,25,26,27,28
        // PMAX 12.5, VMAX 25, TMAX 200
        p_min = -12.5f; p_max = 12.5f;
        v_min = -25.0f; v_max = 25.0f;
        t_min = -200.0f; t_max = 200.0f;
    } else if ((motor_id >= 4 && motor_id <= 6) || (motor_id >= 11 && motor_id <= 13) ||
               (motor_id >= 23 && motor_id <= 24) || (motor_id >= 29 && motor_id <= 30)) {
        // Group B: motorID 4,5,6,11,12,13,23,24,29,30
        // PMAX 12.566, VMAX 20, TMAX 120
        p_min = -12.566f; p_max = 12.566f;
        v_min = -20.0f; v_max = 20.0f;
        t_min = -120.0f; t_max = 120.0f;
    } else {
        // Group C: motorID 1,2,3,7,8,9,10,14,15,16,17
        // PMAX 12.5, VMAX 10, TMAX 28
        p_min = -12.5f; p_max = 12.5f;
        v_min = -10.0f; v_max = 10.0f;
        t_min = -28.0f; t_max = 28.0f;
    }

    // 根据具体电机ID覆盖位置限位（保持原有的位置限位设置）
    switch (motor_id) {
        case 1:  p_min = -1.7f;  p_max = 1.7f;   break;   // 正负1.7
        case 2:  p_min = -0.3f;  p_max = 0.3f;   break;   // 正负0.3
        case 3:  p_min = -0.3f;  p_max = 0.3f;   break;   // 正负0.3
        case 4:  p_min = -1.57f; p_max = 1.57f;  break;   // 正负1.57
        case 5:  p_min = -0.26f; p_max = 2.6f;   break;   // -0.26至2.6
        case 6:  p_min = -1.57f; p_max = 1.57f;  break;   // 正负1.57
        case 7:  p_min = -2.09f; p_max = 0.79f;  break;   // -2.09至0.79
        case 8:  p_min = -1.57f; p_max = 1.57f;  break;   // 正负1.57
        case 9:  p_min = -1.83f; p_max = 1.83f;  break;   // 正负1.83
        case 10: p_min = -1.57f; p_max = 1.57f;  break;   // 正负1.57
        case 11: p_min = -1.57f; p_max = 1.57f;  break;   // 正负1.57
        case 12: p_min = -2.6f;  p_max = 0.26f;  break;   // -2.6至0.26
        case 13: p_min = -1.57f; p_max = 1.57f;  break;   // 正负1.57
        case 14: p_min = -0.79f; p_max = 2.09f;  break;   // -0.79至2.09
        case 15: p_min = -1.57f; p_max = 1.57f;  break;   // 正负1.57
        case 16: p_min = -1.83f; p_max = 1.83f;  break;   // 正负1.83
        case 17: p_min = -1.57f; p_max = 1.57f;  break;   // 正负1.57
        case 18: p_min = -1.57f; p_max = 1.57f;  break;   // 正负1.57
        case 19: p_min = -1.6f;  p_max = 1.6f;   break;   // 正负1.6
        case 20: p_min = -0.35f; p_max = 1.6f;   break;   // -0.35至1.6
        case 21: p_min = -1.57f; p_max = 1.57f;  break;   // 正负1.57
        case 22: p_min = 0.0f;   p_max = 1.6f;   break;   // 0至1.6
        case 23: p_min = -0.3f;  p_max = 0.3f;   break;   // 正负0.3
        case 24: p_min = -0.3f;  p_max = 0.3f;   break;   // 正负0.3
        case 25: p_min = -1.6f;  p_max = 1.6f;   break;   // 正负1.6
        case 26: p_min = -1.6f;  p_max = 0.35f;  break;   // -1.6至0.35
        case 27: p_min = -1.57f; p_max = 1.57f;  break;   // 正负1.57
        case 28: p_min = -1.6f;  p_max = 0.0f;   break;   // -1.6至0
        case 29: p_min = -0.3f;  p_max = 0.3f;   break;   // 正负0.3
        case 30: p_min = -0.3f;  p_max = 0.3f;   break;   // 正负0.3
        default: break;  // 使用分组设置的默认值
    }

    return {p_min, p_max, v_min, v_max, t_min, t_max, kp_min, kp_max, kd_min, kd_max};
}

// ==================== DM Motor Control Mode Definitions ====================
// 与 motor_config.h 中的定义一致
constexpr uint16_t MIT_MODE = 0x000;    // MIT 模式 (混合控制: 位置+速度+力矩)
constexpr uint16_t POS_MODE = 0x100;    // 位置模式
constexpr uint16_t SPEED_MODE = 0x200;  // 速度模式

// DM4310 电机参数范围（与 motor_config.h 中的定义一致）
// 注意：现在使用 GetMotorLimits(motor_id) 来获取电机特定的参数
constexpr float P_MIN_DM = -12.5f;   // Position min: -12.5 rad
constexpr float P_MAX_DM = 12.5f;    // Position max: 12.5 rad
constexpr float V_MIN_DM = -30.0f;   // Velocity min: -30.0 rad/s
constexpr float V_MAX_DM = 30.0f;    // Velocity max: 30.0 rad/s
constexpr float T_MIN_DM = -10.0f;   // Torque min: -10.0 Nm
constexpr float T_MAX_DM = 10.0f;    // Torque max: 10.0 Nm
constexpr float KP_MIN_DM = 0.0f;    // Kp min: 0.0
constexpr float KP_MAX_DM = 500.0f;  // Kp max: 500.0
constexpr float KD_MIN_DM = 0.0f;    // Kd min: 0.0
constexpr float KD_MAX_DM = 5.0f;    // Kd max: 5.0

// ==================== DM Motor Control Command ====================
// DM 电机 MIT 模式控制命令格式（与 motor_config.c 中的 mit_ctrl 函数完全一致）
// DM4310/DM4340/DM6006/DM8006 等电机使用相同的 MIT 模式协议
// 参考: motor_config.c 第 625-656 行
static inline void MotorCommandToCanData_DM(const MotorCommandCan& cmd, uint8_t* can_data) {
    // DM 协议使用固定的全局参数范围（与 motor_config.c 中的 P_MIN1, V_MIN1 等一致）
    // 不能使用电机特定的限制值，否则会导致数据转换错误
    constexpr float P_MIN = -12.5f;   // 与 motor_config.h 中的 P_MIN1 一致
    constexpr float P_MAX = 12.5f;    // 与 motor_config.h 中的 P_MAX1 一致
    constexpr float V_MIN = -30.0f;   // 与 motor_config.h 中的 V_MIN1 一致
    constexpr float V_MAX = 30.0f;    // 与 motor_config.h 中的 V_MAX1 一致
    constexpr float KP_MIN = 0.0f;    // 与 motor_config.h 中的 KP_MIN1 一致
    constexpr float KP_MAX = 500.0f;  // 与 motor_config.h 中的 KP_MAX1 一致
    constexpr float KD_MIN = 0.0f;    // 与 motor_config.h 中的 KD_MIN1 一致
    constexpr float KD_MAX = 5.0f;    // 与 motor_config.h 中的 KD_MAX1 一致
    constexpr float T_MIN = -10.0f;   // 与 motor_config.h 中的 T_MIN1 一致
    constexpr float T_MAX = 10.0f;    // 与 motor_config.h 中的 T_MAX1 一致

    // Step 1: Quantize float parameters to integers using float_to_uint
    // 与 motor_config.c 第 636-640 行完全一致
    uint16_t pos_tmp = float_to_uint(cmd.pos, P_MIN, P_MAX, 16);
    uint16_t vel_tmp = float_to_uint(cmd.vel, V_MIN, V_MAX, 12);
    uint16_t kp_tmp = float_to_uint(cmd.kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_tmp = float_to_uint(cmd.kd, KD_MIN, KD_MAX, 12);
    uint16_t tor_tmp = float_to_uint(cmd.torq, T_MIN, T_MAX, 12);

    // Step 2: Pack data into 8-byte frame (与 motor_config.c 第 643-652 行完全一致)
    can_data[0] = (pos_tmp >> 8);     /* Position high byte */
    can_data[1] = pos_tmp;            /* Position low byte */
    can_data[2] = (vel_tmp >> 4);     /* Velocity high 8 bits */
    can_data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);  /* Velocity low 4 bits | Kp high 4 bits */
    can_data[4] = kp_tmp;             /* Kp low 8 bits */
    can_data[5] = (kd_tmp >> 4);      /* Kd high 4 bits */
    can_data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8); /* Kd low 4 bits | Torque high 4 bits */
    can_data[7] = tor_tmp;            /* Torque low 8 bits */
}

// ==================== DM Protocol Unified Conversion ====================
// 使用 DM 电机 MIT 模式协议（与 motor_config.c 中的 mit_ctrl 函数完全一致）
// 心跳命令：is_heartbeat=true 时发送 pos=0, vel=0, kp=0, kd=0, tau=0 的控制命令
// 这样电机不执行动作，但会返回状态反馈
static inline void MotorCommandToCanData(const MotorCommandCan& cmd, uint8_t* can_data) {
    if (cmd.is_heartbeat) {
        // 心跳命令：发送 pos=0, vel=0, kp=0, kd=0, tau=0 的 DM 控制命令
        // 电机不会产生任何输出力矩，但仍然会返回状态反馈
        MotorCommandCan heartbeat_cmd = cmd;
        heartbeat_cmd.pos = 0.0f;
        heartbeat_cmd.vel = 0.0f;
        heartbeat_cmd.kp = 0.0f;
        heartbeat_cmd.kd = 0.0f;
        heartbeat_cmd.torq = 0.0f;
        MotorCommandToCanData_DM(heartbeat_cmd, can_data);
        return;
    }
    // 使用 DM 电机 MIT 模式控制命令编码（与 motor_config.c 第 625-656 行完全一致）
    MotorCommandToCanData_DM(cmd, can_data);
}

// ==================== ZLG TCP 协议原始数据包显示 ====================
// ZLG CANFDNET TCP 数据包格式:
// [PacketHead: 6 bytes]
//   [0] nPacketHead     = 0x55
//   [1] nPacketType     = 0x00 (CAN)
//   [2] nPacketTypeParam= 0x00
//   [3] nReServed       = 0x00
//   [4-5] nPacketDataLen= FRAME_LEN_CAN (24) = 0x0018
// [PacketDataCAN: 24 bytes]
//   [6-13] nTimestamp   = 0 (8 bytes)
//   [14-17] nID         = CAN ID (4 bytes)
//   [18-19] frameInfo   = 帧信息 (2 bytes)
//   [20] nChnl          = 通道号 (1 byte)
//   [21] nDataLen       = DLC (1 byte)
//   [22-29] canData     = 数据 (8 bytes)
// [30] CRC = 校验和 (1 byte)
static inline void PrintZLGRawPacket(uint32_t can_id, const uint8_t* data, uint8_t dlc, uint8_t channel, bool show_hex_dump = true) {
    std::vector<uint8_t> packet;

    // PacketHead (6 bytes)
    packet.push_back(0x55);                    // nPacketHead
    packet.push_back(0x00);                    // nPacketType (CAN)
    packet.push_back(0x00);                    // nPacketTypeParam
    packet.push_back(0x00);                    // nReServed
    packet.push_back(24 & 0xFF);               // nPacketDataLen low
    packet.push_back((24 >> 8) & 0xFF);        // nPacketDataLen high

    // PacketDataCANHead
    // nTimestamp (8 bytes) = 0 when sending
    for (int i = 0; i < 8; i++) packet.push_back(0x00);

    // nID (4 bytes, little endian)
    packet.push_back(can_id & 0xFF);
    packet.push_back((can_id >> 8) & 0xFF);
    packet.push_back((can_id >> 16) & 0xFF);
    packet.push_back((can_id >> 24) & 0xFF);

    // frameInfo (2 bytes) - CAN标准帧
    packet.push_back(0x00);                    // 低字节: 所有标志为0
    packet.push_back(0x00);                    // 高字节: 保留

    // nChnl (1 byte)
    packet.push_back(channel);

    // nDataLen (1 byte)
    packet.push_back(dlc);

    // canData (8 bytes)
    for (int i = 0; i < 8; i++) {
        packet.push_back(i < dlc ? data[i] : 0x00);
    }

    // CRC (1 byte) - 简单累加校验
    uint8_t crc = 0;
    for (size_t i = 1; i < packet.size(); i++) {  // 从 nPacketType 开始计算
        crc += packet[i];
    }
    packet.push_back(crc);

    // ==================== 完整 TCP 字节流打印 ====================
    std::cout << "\n  ========== ZLG TCP Raw Packet ==========" << std::endl;
    std::cout << "  Total: " << packet.size() << " bytes" << std::endl;

    // 1. 单行十六进制格式
    std::cout << "  [HEX]   ";
    for (size_t i = 0; i < packet.size(); i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << std::uppercase
                  << static_cast<int>(packet[i]);
        if (i < packet.size() - 1) std::cout << " ";
    }
    std::cout << std::dec << std::nouppercase << std::endl;

    // 2. 十六进制转储格式 (16字节一行)
    if (show_hex_dump) {
        std::cout << "  [DUMP]  " << std::endl;
        for (size_t row = 0; row < packet.size(); row += 16) {
            // 偏移量
            std::cout << "    " << std::hex << std::setw(4) << std::setfill('0') << row << std::dec << " | ";

            // 十六进制
            for (size_t i = 0; i < 16; i++) {
                if (row + i < packet.size()) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << std::uppercase
                              << static_cast<int>(packet[row + i]) << std::dec;
                } else {
                    std::cout << "  ";
                }
                if (i == 7) std::cout << " ";
                std::cout << " ";
            }

            // ASCII
            std::cout << "| ";
            for (size_t i = 0; i < 16 && row + i < packet.size(); i++) {
                uint8_t b = packet[row + i];
                if (b >= 32 && b <= 126) {
                    std::cout << static_cast<char>(b);
                } else {
                    std::cout << ".";
                }
            }
            std::cout << std::endl;
        }
    }

    // 3. 字段解析
    std::cout << "  [FIELD] PacketHead (0-5):   ";
    std::cout << "55 " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(packet[0])
              << " " << std::setw(2) << static_cast<int>(packet[1]) << "(CAN)"
              << " " << std::setw(2) << static_cast<int>(packet[2])
              << " " << std::setw(2) << static_cast<int>(packet[3])
              << " [" << std::setw(2) << static_cast<int>(packet[4])
              << " " << std::setw(2) << static_cast<int>(packet[5]) << "=0x0018]"
              << std::dec << std::endl;

    std::cout << "  [FIELD] Timestamp (6-13):  ";
    for (int i = 6; i < 14; i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(packet[i]) << " ";
    }
    std::cout << "= 0x0000000000000000" << std::dec << std::endl;

    std::cout << "  [FIELD] CAN_ID (14-17):    0x"
              << std::hex << std::setw(8) << std::setfill('0') << can_id << std::dec
              << " (LE: ";
    for (int i = 14; i < 18; i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(packet[i]) << " ";
    }
    std::cout << ")" << std::dec << std::endl;

    std::cout << "  [FIELD] FrameInfo (18-19): "
              << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(packet[18]) << " "
              << std::setw(2) << static_cast<int>(packet[19]) << std::dec
              << " (Std Frame)" << std::endl;

    std::cout << "  [FIELD] Channel (20):      " << static_cast<int>(packet[20]) << std::endl;
    std::cout << "  [FIELD] DLC (21):          " << static_cast<int>(packet[21]) << std::endl;

    std::cout << "  [FIELD] Data (22-29):      ";
    for (int i = 0; i < 8; i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(packet[22 + i]) << " ";
    }
    std::cout << std::dec << std::endl;

    std::cout << "  [FIELD] CRC (30):          0x"
              << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(crc)
              << std::dec << std::endl;

    std::cout << "  =======================================" << std::endl;
}

// ==================== 打印批量 CAN 帧的 TCP 字节流 ====================
static inline void PrintZLGRawPacketBatch(const ZCAN_Transmit_Data* frames, uint32_t count, uint8_t channel) {
    if (count == 0 || frames == nullptr) return;

    std::cout << "\n  ========== ZLG TCP Raw Packet Batch ==========" << std::endl;
    std::cout << "  Total Frames: " << count << std::endl;

    for (uint32_t frame_idx = 0; frame_idx < count && frame_idx < 5; frame_idx++) {
        uint32_t can_id = GET_ID(frames[frame_idx].frame.can_id);
        uint8_t dlc = frames[frame_idx].frame.can_dlc;

        std::cout << "\n  [Frame " << frame_idx << "] CAN_ID=0x"
                  << std::hex << std::setw(3) << std::setfill('0') << can_id << std::dec
                  << ", DLC=" << static_cast<int>(dlc) << std::endl;

        // 构建单个数据包
        std::vector<uint8_t> packet;
        packet.push_back(0x55);
        packet.push_back(0x00);
        packet.push_back(0x00);
        packet.push_back(0x00);
        packet.push_back(24 & 0xFF);
        packet.push_back((24 >> 8) & 0xFF);

        for (int i = 0; i < 8; i++) packet.push_back(0x00);

        packet.push_back(can_id & 0xFF);
        packet.push_back((can_id >> 8) & 0xFF);
        packet.push_back((can_id >> 16) & 0xFF);
        packet.push_back((can_id >> 24) & 0xFF);

        packet.push_back(0x00);
        packet.push_back(0x00);
        packet.push_back(channel);
        packet.push_back(dlc);

        for (uint8_t i = 0; i < 8; i++) {
            packet.push_back(i < dlc ? frames[frame_idx].frame.data[i] : 0x00);
        }

        uint8_t crc = 0;
        for (size_t i = 1; i < packet.size(); i++) {
            crc += packet[i];
        }
        packet.push_back(crc);

        // 打印十六进制
        std::cout << "    HEX: ";
        for (size_t i = 0; i < packet.size(); i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << std::uppercase
                      << static_cast<int>(packet[i]) << " ";
        }
        std::cout << std::dec << std::nouppercase << std::endl;
    }

    if (count > 5) {
        std::cout << "\n  ... and " << (count - 5) << " more frames" << std::endl;
    }

    std::cout << "  ===============================================" << std::endl;
}

// Motor ID mapping function
// 根据端口计算本地CAN ID（每个端口上的电机使用独立的本地CAN ID）
// 8000 (offset=0):  全局1-10  → 本地CAN ID 1-10  (0x01-0x0A)
// 8001 (offset=10): 全局11-17 → 本地CAN ID 1-7   (0x01-0x07)
// 8002 (offset=17): 全局18-24 → 本地CAN ID 1-7   (0x01-0x07)
// 8003 (offset=24): 全局25-30 → 本地CAN ID 1-6   (0x01-0x06)
static inline int get_can_id_for_motor(int motor_id) {
    if (motor_id < 1 || motor_id > 30) {
        return 0;
    }

    // Find the port config for this motor and return local CAN ID
    for (const auto& config : PORT_CONFIGS) {
        int motor_start = config.motor_offset + 1;
        int motor_end = config.motor_offset + config.motor_count;
        if (motor_id >= motor_start && motor_id <= motor_end) {
            return motor_id - config.motor_offset;  // 返回本地CAN ID
        }
    }

    return 0;
}

// ==================== DM Motor Format Decoder ====================
// DM motor feedback format:
// D[0] D[1] D[2] D[3] D[4] D[5] D[6] D[7]
// ID|ERR<<4, POS[15:8], POS[7:0], VEL[11:4], VEL[3:0]|T[11:8], T[7:0], T_MOS, T_Rotor
//
// Encoding (float to fixed-point linear mapping):
//   POS_16bit  = position_rad  / (PMAX - PMIN) * 2^16 + 2^15
//   VEL_12bit = velocity_rad_s / (VMAX - VMIN) * 2^12 + 2^11
//   T_12bit   = torque_nm      / (TMAX - TMIN) * 2^12 + 2^11
//
// Decoding (fixed-point to float):
//   position_rad  = (POS_16bit  - 2^15) / 2^16 * (PMAX - PMIN)
//   velocity_rad_s = (VEL_12bit - 2^11) / 2^12 * (VMAX - VMIN)
//   torque_nm      = (T_12bit   - 2^11) / 2^12 * (TMAX - TMIN)

class DMMotorFrameDecoder {
public:
    // DM motor range limits (adjust based on your motor specifications)
    static constexpr double PMAX = 3.14159;    // Position max: π rad (180°)
    static constexpr double PMIN = -3.14159;   // Position min: -π rad (-180°)
    static constexpr double VMAX = 45.0;       // Velocity max: 45 rad/s
    static constexpr double VMIN = -45.0;      // Velocity min: -45 rad/s
    static constexpr double TMAX = 20.0;       // Torque max: 20 Nm
    static constexpr double TMIN = -20.0;      // Torque min: -20 Nm

    struct DMMotorData {
        // Raw CAN frame data
        uint8_t raw[8];      // Raw CAN data bytes

        // Decoded values (DM motor format, fixed-point)
        uint8_t  motor_id;   // Motor ID (4 bits from D[0] lower)
        uint8_t  error;      // Error code (4 bits from D[0] upper)
        int16_t  position;   // Position raw (16-bit signed from D[1], D[2])
        int16_t  velocity;   // Velocity raw (12-bit signed from D[3], D[4] lower)
        int16_t  torque;     // Torque raw (12-bit signed from D[4] upper, D[5])
        int8_t   temp_mos;   // MOS temperature (8-bit signed from D[6])
        int8_t   temp_rotor; // Rotor temperature (8-bit signed from D[7])

        // Decoded physical values (linear mapping from fixed-point)
        double position_rad;   // Position in radians
        double velocity_rad_s; // Velocity in rad/s
        double torque_nm;      // Torque in Nm
    };

    // Decode CAN FD frame (DM motor format with physical value conversion)
    static DMMotorData DecodeFrameFD(const ZCAN_ReceiveFD_Data& frame) {
        DMMotorData data;
        memset(&data, 0, sizeof(data));

        const uint8_t* d = frame.frame.data;

        // Store raw data
        memcpy(data.raw, d, 8);

        // D[0]: ID[3:0] | ERR[3:0]<<4
        data.motor_id = d[0] & 0x0F;           // Lower 4 bits: Motor ID
        data.error = (d[0] >> 4) & 0x0F;       // Upper 4 bits: Error code

        // D[1-2]: Position (16-bit signed)
        // Encoding: POS_16bit = position_rad / (PMAX - PMIN) * 2^16 + 2^15
        // Decoding: position_rad = (POS_16bit - 2^15) / 2^16 * (PMAX - PMIN)
        data.position = static_cast<int16_t>((d[1] << 8) | d[2]);
        data.position_rad = (data.position - 32768.0) / 65536.0 * (PMAX - PMIN);

        // D[3-4]: Velocity (12-bit signed)
        // Encoding: VEL_12bit = velocity_rad_s / (VMAX - VMIN) * 2^12 + 2^11
        // Decoding: velocity_rad_s = (VEL_12bit - 2^11) / 2^12 * (VMAX - VMIN)
        int16_t vel_raw = ((d[3] & 0xFF) << 4) | (d[4] & 0x0F);
        if (vel_raw & 0x800) {
            vel_raw |= 0xF000;  // Sign extension for negative 12-bit values
        }
        data.velocity = vel_raw;
        data.velocity_rad_s = (vel_raw - 2048.0) / 4096.0 * (VMAX - VMIN);

        // D[4-5]: Torque (12-bit signed)
        // Encoding: T_12bit = torque_nm / (TMAX - TMIN) * 2^12 + 2^11
        // Decoding: torque_nm = (T_12bit - 2^11) / 2^12 * (TMAX - TMIN)
        int16_t torque_raw = (((d[4] >> 4) & 0x0F) << 8) | d[5];
        if (torque_raw & 0x800) {
            torque_raw |= 0xF000;
        }
        data.torque = torque_raw;
        data.torque_nm = (torque_raw - 2048.0) / 4096.0 * (TMAX - TMIN);

        // D[6-7]: Temperatures
        data.temp_mos = static_cast<int8_t>(d[6]);
        data.temp_rotor = static_cast<int8_t>(d[7]);

        return data;
    }

    // Format decoded data as string (physical values)
    static std::string FormatDecoded(const DMMotorData& data) {
        std::stringstream ss;
        ss << "ID=" << std::setw(2) << (int)data.motor_id << " | ";
        ss << "Pos=" << std::fixed << std::setprecision(3) << data.position_rad << " rad ";
        ss << "(raw=" << data.position << ") | ";
        ss << "Vel=" << std::setprecision(2) << data.velocity_rad_s << " rad/s ";
        ss << "(raw=" << data.velocity << ") | ";
        ss << "Tau=" << std::setprecision(2) << data.torque_nm << " Nm ";
        ss << "(raw=" << data.torque << ") | ";
        ss << "Tmos=" << std::setw(3) << (int)data.temp_mos << " | ";
        ss << "Trt=" << std::setw(3) << (int)data.temp_rotor << " | ";
        ss << "Err=0x" << std::hex << (int)data.error << std::dec;
        return ss.str();
    }

    // Format raw data as hex string
    static std::string FormatRaw(const DMMotorData& data) {
        std::stringstream ss;
        ss << "RAW: ";
        for (int i = 0; i < 8; i++) {
            ss << std::hex << std::setw(2) << std::setfill('0') << (int)data.raw[i];
            if (i < 7) ss << " ";
        }
        ss << std::dec << std::setfill(' ');
        return ss.str();
    }

    // Get error description
    static std::string GetErrorDescription(uint8_t error) {
        switch (error) {
            case 0x0: return "OK/Disabled";
            case 0x1: return "Enabled";
            case 0x8: return "Over-voltage";
            case 0x9: return "Under-voltage";
            case 0xA: return "Over-current";
            case 0xB: return "MOS Over-temp";
            case 0xC: return "Coil Over-temp";
            case 0xD: return "Comms Lost";
            case 0xE: return "Overload";
            default: return "Unknown(0x" + std::to_string(error) + ")";
        }
    }
};

// ==================== MIT 电机响应解析 (Legacy, 兼容保留) ====================
// MIT 协议电机返回数据格式:
// Bytes 0-1: Position (int16, ±12.5 rad -> ±32767)
// Bytes 2-3: Velocity (int16, ±30 rad/s -> ±32767)
// Bytes 4-5: 拉力/力矩 (int16, scale depends on motor)
// Bytes 6-7: 温度/状态 (int16)
// Byte 6: 温度或错误码
// Byte 7: 状态标志位

struct MotorResponse {
    uint32_t motor_id;
    float pos;        // 位置
    float vel;        // 速度
    float torque;     // 力矩
    int16_t raw_pos;
    int16_t raw_vel;
    int16_t raw_torque;
    uint8_t temp;     // 温度
    uint8_t status;   // 状态
    uint64_t timestamp; // 接收时间戳 (us)
    bool valid;
};

// 从 CAN 数据解析电机响应
static inline MotorResponse ParseMotorResponse(const ZCAN_Receive_Data& can_data) {
    MotorResponse resp = {0};

    uint32_t can_id = GET_ID(can_data.frame.can_id);
    resp.motor_id = can_id;
    resp.timestamp = can_data.timestamp;

    if (can_data.frame.can_dlc >= 8) {
        // 解析位置 (int16)
        resp.raw_pos = static_cast<int16_t>(can_data.frame.data[0] |
                                            (can_data.frame.data[1] << 8));
        resp.pos = static_cast<float>(resp.raw_pos) * 12.5f / 32767.0f;

        // 解析速度 (int16)
        resp.raw_vel = static_cast<int16_t>(can_data.frame.data[2] |
                                            (can_data.frame.data[3] << 8));
        resp.vel = static_cast<float>(resp.raw_vel) * 30.0f / 32767.0f;

        // 解析力矩 (int16)
        resp.raw_torque = static_cast<int16_t>(can_data.frame.data[4] |
                                              (can_data.frame.data[5] << 8));
        // 力矩缩放因子取决于具体电机，这里假设为 -18 ~ 18 Nm
        resp.torque = static_cast<float>(resp.raw_torque) * 18.0f / 32767.0f;

        // 解析温度和状态
        resp.temp = can_data.frame.data[6];
        resp.status = can_data.frame.data[7];

        resp.valid = true;
    } else {
        resp.valid = false;
    }

    return resp;
}

// 打印接收到的 CAN 帧（原始字节流）
static inline void PrintReceivedCANFrame(const ZCAN_Receive_Data& can_data, uint8_t channel) {
    uint32_t can_id = GET_ID(can_data.frame.can_id);

    std::cout << "\n  ========== Received CAN Frame ==========" << std::endl;
    std::cout << "  [Header] Timestamp: " << can_data.timestamp << " us" << std::endl;
    std::cout << "  [Header] Channel:   " << static_cast<int>(channel) << std::endl;
    std::cout << "  [Header] Flags:     0x" << std::hex << std::setw(8) << std::setfill('0')
              << can_data.frame.can_id << std::dec;

    // 解析标志位
    if (can_data.frame.can_id & CAN_EFF_FLAG) std::cout << " [EFF]";
    if (can_data.frame.can_id & CAN_RTR_FLAG) std::cout << " [RTR]";
    if (can_data.frame.can_id & CAN_ERR_FLAG) std::cout << " [ERR]";
    std::cout << std::endl;

    std::cout << "  [CAN_ID] 0x" << std::hex << std::setw(3) << std::setfill('0')
              << can_id << std::dec << " (ID=" << can_id << ")" << std::endl;

    std::cout << "  [DLC]    " << static_cast<int>(can_data.frame.can_dlc) << " bytes" << std::endl;

    // 十六进制数据
    std::cout << "  [DATA]   HEX: ";
    for (uint8_t i = 0; i < can_data.frame.can_dlc; i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << std::uppercase
                  << static_cast<int>(can_data.frame.data[i]) << " ";
    }
    std::cout << std::dec << std::nouppercase << std::endl;

    // 十进制数据
    std::cout << "  [DATA]   DEC: ";
    for (uint8_t i = 0; i < can_data.frame.can_dlc; i++) {
        std::cout << std::setw(3) << static_cast<int>(can_data.frame.data[i]) << " ";
    }
    std::cout << std::endl;

    std::cout << "  =========================================" << std::endl;
}

// 打印解析后的电机响应
static inline void PrintMotorResponse(const MotorResponse& resp) {
    if (!resp.valid) {
        std::cout << "  [MOTOR] Invalid response (DLC < 8)" << std::endl;
        return;
    }

    std::cout << "  [MOTOR] ID=" << resp.motor_id << " | ";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "pos=" << resp.pos << " rad (" << resp.raw_pos << ") | ";
    std::cout << "vel=" << resp.vel << " rad/s (" << resp.raw_vel << ") | ";
    std::cout << "tau=" << resp.torque << " Nm (" << resp.raw_torque << ") | ";
    std::cout << "temp=" << static_cast<int>(resp.temp) << " | ";
    std::cout << "status=0x" << std::hex << static_cast<int>(resp.status) << std::dec;
    std::cout << std::endl;
}

// ==================== 直接命令模式辅助函数 (使用ZCAN官方封装) ====================
// 用于直接命令模式的简化 ZCAN 操作

static ZhilgongConfig direct_zlg_config;

// 获取电机对应的端口和通道配置（统一的电机到端口映射）
static inline void GetPortConfigForMotor(int motor_id, int& port, int& channel) {
    // 统一的电机到端口映射 (motor_id: 1-30)
    // 端口8000: 电机1-10  (channel 0)
    // 端口8001: 电机11-17 (channel 1)
    // 端口8002: 电机18-24 (channel 2)
    // 端口8003: 电机25-30 (channel 3)
    if (motor_id >= 1 && motor_id <= 10)  { port = 8000; channel = 0; return; }
    if (motor_id >= 11 && motor_id <= 17) { port = 8001; channel = 1; return; }
    if (motor_id >= 18 && motor_id <= 24) { port = 8002; channel = 2; return; }
    if (motor_id >= 25 && motor_id <= 30) { port = 8003; channel = 3; return; }

    // 默认值
    port = 8000;
    channel = 0;
}

// 获取电机对应的端口索引（用于访问PORT_CONFIGS数组）
static inline int GetPortIndexForMotor(int motor_id) {
    // 统一的电机到端口映射 (motor_id: 1-30)
    // 端口索引0: 电机1-10  (port 8000)
    // 端口索引1: 电机11-17 (port 8001)
    // 端口索引2: 电机18-24 (port 8002)
    // 端口索引3: 电机25-30 (port 8003)
    if (motor_id >= 1 && motor_id <= 10)  return 0;
    if (motor_id >= 11 && motor_id <= 17) return 1;
    if (motor_id >= 18 && motor_id <= 24) return 2;
    if (motor_id >= 25 && motor_id <= 30) return 3;
    return -1;  // 无效的电机ID
}

// 读取并显示 CAN 通道状态
// ==================== CAN 总线状态和错误定义 ====================
// ZCAN_CHANNEL_STATUS 结构体字段说明:
//   regTECounter - 发送错误计数器 (Transmit Error Counter)
//   regRECounter - 接收错误计数器 (Receive Error Counter)
//   regStatus    - 寄存器状态

// CAN 总线状态定义
static const char* GetCANBusStateString(UINT regStatus) {
    switch (regStatus) {
        case 0:  return "Error-Active (正常活动状态)";
        case 1:  return "Error-Passive (错误被动状态)";
        case 2:  return "Bus-Off (总线关闭状态)";
        default: return "Unknown (未知状态)";
    }
}

// 错误计数器状态说明
static void ExplainErrorCounter(UINT teCounter, UINT reCounter) {
    if (teCounter < 128 && reCounter < 128) {
        std::cout << "    [状态] Error-Active: 节点可以正常发送和接收" << std::endl;
    } else if (teCounter >= 128 && teCounter < 256) {
        std::cout << "    [状态] TX Error-Passive: 发送错误计数 >= 128，节点仍可接收但不能主动发送" << std::endl;
        std::cout << "    [原因] 可能原因: ACK错误、格式错误、位错误等" << std::endl;
    } else if (reCounter >= 128 && reCounter < 256) {
        std::cout << "    [状态] RX Error-Passive: 接收错误计数 >= 128，节点仍可发送" << std::endl;
        std::cout << "    [原因] 可能原因: CRC错误、帧错误、位错误等" << std::endl;
    } else if (teCounter >= 256) {
        std::cout << "    [状态] Bus-Off: 发送错误计数 >= 256，节点不能发送或接收" << std::endl;
        std::cout << "    [原因] 总线错误过多，节点已进入Bus-Off状态，需要恢复" << std::endl;
    }

    // 详细错误分析
    if (teCounter > 0 || reCounter > 0) {
        std::cout << "    [诊断] ";
        if (teCounter > reCounter) {
            std::cout << "发送错误为主，可能原因: 总线上无应答节点、终端电阻问题、线缆长度问题" << std::endl;
        } else if (reCounter > teCounter) {
            std::cout << "接收错误为主，可能原因: 信号干扰、波特率不匹配、帧格式错误" << std::endl;
        } else {
            std::cout << "收发错误均衡，可能是总线上的普遍性问题" << std::endl;
        }
    }
}

// 读取并详细显示 CAN 状态
static void ReadAndDisplayCanStatusDetailed(CHANNEL_HANDLE channel_handle, bool explain = true) {
    if (channel_handle == INVALID_CHANNEL_HANDLE) {
        std::cerr << "    [!] Invalid channel handle" << std::endl;
        return;
    }

    ZCAN_CHANNEL_STATUS status;
    UINT ret = ZCAN_ReadChannelStatus(channel_handle, &status);

    std::cout << "\n    ========== CAN Channel Status ==========" << std::endl;

    if (ret == 1) {
        std::cout << "    TX_Err (发送错误): " << std::dec << static_cast<int>(status.regTECounter);
        if (status.regTECounter > 127) std::cout << " [!] HIGH (>127)";
        if (status.regTECounter >= 256) std::cout << " [!!!] Bus-Off";
        std::cout << std::endl;

        std::cout << "    RX_Err (接收错误): " << static_cast<int>(status.regRECounter);
        if (status.regRECounter > 127) std::cout << " [!] HIGH (>127)";
        std::cout << std::endl;

        std::cout << "    BusState: " << GetCANBusStateString(status.regStatus) << std::endl;

        if (explain && (status.regTECounter > 0 || status.regRECounter > 0)) {
            std::cout << "\n    [错误分析]" << std::endl;
            ExplainErrorCounter(status.regTECounter, status.regRECounter);
        }

        // 总线健康度评估
        std::cout << "\n    [总线健康度] ";
        if (status.regTECounter == 0 && status.regRECounter == 0) {
            std::cout << "100% (完美)" << std::endl;
        } else if (status.regTECounter < 10 && status.regRECounter < 10) {
            std::cout << "90%+ (良好)" << std::endl;
        } else if (status.regTECounter < 50 && status.regRECounter < 50) {
            std::cout << "70-90% (一般)" << std::endl;
        } else if (status.regTECounter < 128 && status.regRECounter < 128) {
            std::cout << "50-70% (较差)" << std::endl;
        } else {
            std::cout << "<50% (严重问题)" << std::endl;
        }
    } else {
        std::cerr << "    [!] Failed to read channel status (ret=" << ret << ")" << std::endl;
    }
    std::cout << "    =======================================" << std::endl;
}

// 简化版状态读取（向后兼容）
static void ReadAndDisplayCanStatus(CHANNEL_HANDLE channel_handle) {
    if (channel_handle == INVALID_CHANNEL_HANDLE) {
        return;
    }

    ZCAN_CHANNEL_STATUS status;
    UINT ret = ZCAN_ReadChannelStatus(channel_handle, &status);
    if (ret == 1) {
        std::cout << "  [CAN Status] TX_Err: " << static_cast<int>(status.regTECounter)
                  << " | RX_Err: " << static_cast<int>(status.regRECounter);
        if (status.regTECounter > 100 || status.regRECounter > 100) {
            std::cout << " [!] HIGH ERRORS";
        }
        std::cout << std::endl;
    }
}

// 初始化直接命令模式的 ZCAN 连接
static bool InitializeDirectZCAN(const std::string& ip, int port, int channel, int arb_baud, int data_baud) {
    if (direct_zlg_config.initialized) {
        return true;
    }

    std::cout << "\n=== Initializing ZCAN Connection ===" << std::endl;
    std::cout << "Target: " << ip << ":" << port << std::endl;
    std::cout << "Channel: CAN" << channel << std::endl;

    // 1. ZCAN_OpenDevice - 打开设备
    direct_zlg_config.device_handle = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, 0, 0);
    if (direct_zlg_config.device_handle == INVALID_DEVICE_HANDLE) {
        std::cerr << "Failed to open ZCAN device" << std::endl;
        return false;
    }
    std::cout << "[1/4] Device opened (ZCAN_OpenDevice)" << std::endl;

    // 2. ZCAN_InitCAN - 初始化CAN通道 (使用CAN-FD模式，显式配置)
    ZCAN_CHANNEL_INIT_CONFIG init_config;
    memset(&init_config, 0, sizeof(init_config));

    // 使用 CAN-FD 模式，显式设置波特率
    init_config.can_type = TYPE_CANFD;            // CAN-FD模式
    init_config.canfd.acc_code = 0;              // 接受码
    init_config.canfd.acc_mask = 0;              // 接受掩码
    init_config.canfd.abit_timing = arb_baud;     // 仲裁域波特率: 1000000 = 1Mbps
    init_config.canfd.dbit_timing = data_baud;     // 数据域波特率: 5000000 = 5Mbps
    init_config.canfd.brp = 0;                    // 波特率预分频器
    init_config.canfd.filter = 0;                 // 过滤器
    init_config.canfd.mode = 0;                   // 正常模式

    std::cout << "  CAN-FD config: abit_timing=" << arb_baud << ", dbit_timing=" << data_baud << std::endl;

    direct_zlg_config.channel_handle = ZCAN_InitCAN(direct_zlg_config.device_handle, channel, &init_config);
    if (direct_zlg_config.channel_handle == INVALID_CHANNEL_HANDLE) {
        std::cerr << "Failed to initialize CAN channel " << channel << std::endl;
        ZCAN_CloseDevice(direct_zlg_config.device_handle);
        return false;
    }
    std::cout << "[2/4] CAN channel initialized (ZCAN_InitCAN)" << std::endl;

    // 3. ZCAN_SetReference - 设置工作模式和连接参数
    UINT val = 0;  // 0 = TCP client mode
    ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, channel, CMD_TCP_TYPE, &val);

    // 设置目标 IP
    ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, channel, CMD_DESIP, (void*)ip.c_str());

    // 设置目标端口
    val = port;
    ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, channel, CMD_DESPORT, &val);
    std::cout << "[3/4] Connection parameters set (ZCAN_SetReference)" << std::endl;

    // 4. ZCAN_StartCAN - 启动CAN通道
    if (ZCAN_StartCAN(direct_zlg_config.channel_handle) == STATUS_ERR) {
        std::cerr << "Failed to start CAN channel" << std::endl;
        ZCAN_CloseDevice(direct_zlg_config.device_handle);
        return false;
    }
    std::cout << "[4/4] CAN channel started (ZCAN_StartCAN)" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "=====================================" << std::endl << std::endl;

    direct_zlg_config.initialized = true;
    direct_zlg_config.remote_ip = ip;
    direct_zlg_config.remote_port = port;
    direct_zlg_config.channel = channel;
    direct_zlg_config.arbitration_baud = arb_baud;
    direct_zlg_config.data_baud = data_baud;

    return true;
}

// 发送单个 CAN 帧
static bool SendDirectCANFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc, bool verbose = false) {
    if (!direct_zlg_config.initialized || direct_zlg_config.channel_handle == INVALID_CHANNEL_HANDLE) {
        std::cerr << "[!] ZCAN device not connected" << std::endl;
        return false;
    }

    ZCAN_Transmit_Data transmit_data;
    memset(&transmit_data, 0, sizeof(transmit_data));

    transmit_data.frame.can_id = MAKE_CAN_ID(can_id, 0, 0, 0);

    if (dlc > 8) dlc = 8;
    transmit_data.frame.can_dlc = dlc;

    for (uint8_t i = 0; i < dlc; i++) {
        transmit_data.frame.data[i] = data[i];
    }

    transmit_data.transmit_type = 0;

    // 打印完整的数据包内容（包括 TCP 字节流）
    if (verbose) {
        std::cout << "  >>> Sending CAN Frame (Complete Packet) <<<" << std::endl;
        std::cout << "  +----------------------------------------+" << std::endl;
        std::cout << "  | ZCAN_Transmit_Data Structure:         |" << std::endl;
        std::cout << "  +----------------------------------------+" << std::endl;
        std::cout << "  | transmit_type : " << std::dec << static_cast<int>(transmit_data.transmit_type) << " (0=Normal)" << std::endl;
        std::cout << "  | can_id (raw)  : 0x" << std::hex << std::setw(8) << std::setfill('0') << transmit_data.frame.can_id << std::dec << std::endl;
        std::cout << "  |   - CAN ID    : 0x" << std::hex << std::setw(3) << std::setfill('0') << can_id << std::dec << std::endl;
        std::cout << "  |   - Flags     : 0x" << std::hex << std::setw(3) << std::setfill('0') << (transmit_data.frame.can_id & ~0x7FF) << std::dec;
        // 解析标志位
        if (transmit_data.frame.can_id & 0x80000000) std::cout << " [EFF]";
        if (transmit_data.frame.can_id & 0x40000000) std::cout << " [RTR]";
        if (transmit_data.frame.can_id & 0x20000000) std::cout << " [ERR]";
        std::cout << std::endl;
        std::cout << "  | can_dlc       : " << static_cast<int>(transmit_data.frame.can_dlc) << " bytes" << std::endl;
        std::cout << "  | data[]        : ";
        for (uint8_t i = 0; i < dlc; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(transmit_data.frame.data[i]);
            if (i < dlc - 1) std::cout << " ";
        }
        std::cout << std::dec << std::endl;
        std::cout << "  +----------------------------------------+" << std::endl;

        // ==================== 打印 TCP 字节流 ====================
        PrintZLGRawPacket(can_id, data, dlc, direct_zlg_config.channel, true);
    }

    // ==================== 验证单帧发送：测量发送时间 ====================
    auto start_time = std::chrono::high_resolution_clock::now();
    uint32_t ret = ZCAN_Transmit(direct_zlg_config.channel_handle, &transmit_data, 1);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    if (ret == 1) {
        if (verbose) {
            std::cout << "    Status    : [OK] Sent successfully" << std::endl;
            std::cout << "    [TIME] Single frame send took: " << duration_us << " us" << std::endl;
            std::cout << "    [VERIFY] Single frame: ONE API call for ONE frame" << std::endl;
        }
        ReadAndDisplayCanStatus(direct_zlg_config.channel_handle);
        return true;
    } else {
        std::cerr << "    Status    : [FAIL] ret=" << ret << std::endl;
        ReadAndDisplayCanStatus(direct_zlg_config.channel_handle);
        return false;
    }
}

// 队列批量发送 CAN 帧（ZLG SDK 队列发送）
static int SendDirectCANFrameBatch(const ZCAN_Transmit_Data* frames, uint32_t count, bool verbose = false) {
    if (!direct_zlg_config.initialized || direct_zlg_config.channel_handle == INVALID_CHANNEL_HANDLE) {
        std::cerr << "[!] ZCAN device not connected" << std::endl;
        return 0;
    }

    if (count == 0 || frames == nullptr) {
        return 0;
    }

    // 打印批量发送信息（完整包结构 + TCP 字节流）
    if (verbose) {
        std::cout << "  >>> Batch Sending " << count << " CAN Frames <<<" << std::endl;

        // 打印 TCP 字节流（前5个帧的详细包）
        PrintZLGRawPacketBatch(frames, count, direct_zlg_config.channel);

        // 打印 ZCAN_Transmit_Data 结构信息
        std::cout << "\n  [ZCAN_Structure]" << std::endl;
        for (uint32_t i = 0; i < count && i < 5; i++) { // 显示前5个
            uint32_t can_id = GET_ID(frames[i].frame.can_id);
            std::cout << "  [Frame " << i << "] ";
            std::cout << "ID=0x" << std::hex << std::setw(3) << std::setfill('0') << can_id << std::dec << " ";
            std::cout << "dlc=" << static_cast<int>(frames[i].frame.can_dlc) << " ";
            std::cout << "data=";
            for (uint8_t j = 0; j < frames[i].frame.can_dlc && j < 8; j++) {
                std::cout << std::hex << std::setw(2) << std::setfill('0')
                          << static_cast<int>(frames[i].frame.data[j]);
                if (j < frames[i].frame.can_dlc - 1) std::cout << " ";
            }
            std::cout << std::dec << std::endl;
        }
        if (count > 5) {
            std::cout << "  ... and " << (count - 5) << " more frames" << std::endl;
        }
    }

    // ==================== 验证队列发送：测量发送时间 ====================
    auto start_time = std::chrono::high_resolution_clock::now();

    // 使用 ZLG SDK 的队列发送功能（一次发送多个帧）
    uint32_t ret = ZCAN_Transmit(direct_zlg_config.channel_handle, const_cast<ZCAN_Transmit_Data*>(frames), count);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    if (verbose) {
        std::cout << "    Batch Result: " << ret << "/" << count << " frames sent" << std::endl;
        std::cout << "    [TIME] Single ZCAN_Transmit call took: " << duration_us << " us" << std::endl;
        std::cout << "    [TIME] Average per frame: " << (duration_us / count) << " us/frame" << std::endl;
        std::cout << "    [VERIFY] Queue send confirmed: " << count << " frames in ONE API call" << std::endl;
        ReadAndDisplayCanStatus(direct_zlg_config.channel_handle);
    }

    return ret;
}

// 发送使能命令
static void SendDirectEnableCommand(int motor_id) {
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

    SendDirectCANFrame(can_id, enable_frame, 8, true);
}

// 发送失能命令
static void SendDirectDisableCommand(int motor_id) {
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

    SendDirectCANFrame(can_id, disable_frame, 8, true);
}

// ==================== CAN 接收线程 ====================
static std::atomic<bool> receive_thread_running{false};
static std::thread can_receive_thread;

// CAN 接收统计
static std::atomic<uint64_t> receive_count_{0};
static std::atomic<uint64_t> motor_response_count_{0};
static std::map<uint32_t, MotorResponse> latest_motor_states_;
static std::mutex motor_states_mutex_;

// 从 CANFD 帧解析电机响应
static inline MotorResponse ParseMotorResponseFromFD(const ZCAN_ReceiveFD_Data& canfd_data) {
    MotorResponse resp = {0};

    uint32_t can_id = GET_ID(canfd_data.frame.can_id);
    resp.motor_id = can_id;
    resp.timestamp = canfd_data.timestamp;

    if (canfd_data.frame.len >= 8) {
        // 解析位置 (int16)
        resp.raw_pos = static_cast<int16_t>(canfd_data.frame.data[0] |
                                            (canfd_data.frame.data[1] << 8));
        resp.pos = static_cast<float>(resp.raw_pos) * 12.5f / 32767.0f;

        // 解析速度 (int16)
        resp.raw_vel = static_cast<int16_t>(canfd_data.frame.data[2] |
                                            (canfd_data.frame.data[3] << 8));
        resp.vel = static_cast<float>(resp.raw_vel) * 30.0f / 32767.0f;

        // 解析力矩 (int16)
        resp.raw_torque = static_cast<int16_t>(canfd_data.frame.data[4] |
                                              (canfd_data.frame.data[5] << 8));
        // 力矩缩放因子取决于具体电机，这里假设为 -18 ~ 18 Nm
        resp.torque = static_cast<float>(resp.raw_torque) * 18.0f / 32767.0f;

        // 解析温度和状态
        resp.temp = canfd_data.frame.data[6];
        resp.status = canfd_data.frame.data[7];

        resp.valid = true;
    } else {
        resp.valid = false;
    }

    return resp;
}

// 打印接收到的 CANFD 帧（原始字节流）
static inline void PrintReceivedCANFDFrame(const ZCAN_ReceiveFD_Data& canfd_data, uint8_t channel) {
    uint32_t can_id = GET_ID(canfd_data.frame.can_id);

    std::cout << "\n  ========== Received CANFD Frame ==========" << std::endl;
    std::cout << "  [Header] Timestamp: " << canfd_data.timestamp << " us" << std::endl;
    std::cout << "  [Header] Channel:   " << static_cast<int>(channel) << std::endl;
    std::cout << "  [Header] Flags:     0x" << std::hex << std::setw(8) << std::setfill('0')
              << canfd_data.frame.can_id << std::dec;

    // 解析标志位
    if (canfd_data.frame.can_id & CAN_EFF_FLAG) std::cout << " [EFF]";
    if (canfd_data.frame.can_id & CAN_RTR_FLAG) std::cout << " [RTR]";
    if (canfd_data.frame.can_id & CAN_ERR_FLAG) std::cout << " [ERR]";
    std::cout << std::endl;

    std::cout << "  [Header] FD Flags:  0x" << std::hex << static_cast<int>(canfd_data.frame.flags) << std::dec;
    if (canfd_data.frame.flags & CANFD_BRS) std::cout << " [BRS]";
    if (canfd_data.frame.flags & CANFD_ESI) std::cout << " [ESI]";
    std::cout << std::endl;

    std::cout << "  [CAN_ID] 0x" << std::hex << std::setw(3) << std::setfill('0')
              << can_id << std::dec << " (ID=" << can_id << ")" << std::endl;

    std::cout << "  [DLC]    " << static_cast<int>(canfd_data.frame.len) << " bytes" << std::endl;

    // 十六进制数据
    std::cout << "  [DATA]   HEX: ";
    for (uint8_t i = 0; i < canfd_data.frame.len && i < 64; i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << std::uppercase
                  << static_cast<int>(canfd_data.frame.data[i]) << " ";
    }
    std::cout << std::dec << std::nouppercase << std::endl;

    // 十进制数据
    std::cout << "  [DATA]   DEC: ";
    for (uint8_t i = 0; i < canfd_data.frame.len && i < 64; i++) {
        std::cout << std::setw(3) << static_cast<int>(canfd_data.frame.data[i]) << " ";
    }
    std::cout << std::endl;

    std::cout << "  ===========================================" << std::endl;
}

// CAN 接收线程函数 - 使用 ZCAN_ReceiveFD 直接从通道接收 CANFD 帧
static void CANReceiveThread(CHANNEL_HANDLE channel_handle, uint8_t channel, bool verbose) {
    const int MAX_RECEIVE_LEN = 100;
    ZCAN_ReceiveFD_Data receive_buffer[MAX_RECEIVE_LEN];

    std::cout << "\n>>> CAN Receive Thread Started (ZCAN_ReceiveFD mode) <<<" << std::endl;
    std::cout << "  Waiting for CANFD messages on channel " << static_cast<int>(channel) << "..." << std::endl;

    auto last_stats_time = std::chrono::steady_clock::now();
    uint64_t last_count = 0;
    int receive_fail_count = 0;

    while (receive_thread_running) {
        // ==================== 接收 CANFD 帧 ====================
        // 使用 ZCAN_ReceiveFD 直接从通道接收 CANFD 帧
        // wait_time = 10ms，避免 CPU 占用过高
        uint32_t received = ZCAN_ReceiveFD(channel_handle, receive_buffer, MAX_RECEIVE_LEN, 10);

        if (received > 0) {
            receive_fail_count = 0;  // 重置失败计数
            receive_count_ += received;

            for (uint32_t i = 0; i < received; i++) {
                uint32_t can_id = GET_ID(receive_buffer[i].frame.can_id);

                // 只在 verbose 模式打印接收到的 CANFD 帧详细信息
                if (verbose) {
                    std::cout << "\n  ========== Received CANFD Frame ==========" << std::endl;
                    std::cout << "  [Header] Timestamp: " << receive_buffer[i].timestamp << " us" << std::endl;
                    std::cout << "  [Header] Channel:   " << static_cast<int>(channel) << std::endl;
                    std::cout << "  [Header] Flags:     0x" << std::hex << std::setw(8) << std::setfill('0')
                              << receive_buffer[i].frame.can_id << std::dec;

                    if (receive_buffer[i].frame.can_id & CAN_EFF_FLAG) std::cout << " [EFF]";
                    if (receive_buffer[i].frame.can_id & CAN_RTR_FLAG) std::cout << " [RTR]";
                    if (receive_buffer[i].frame.can_id & CAN_ERR_FLAG) std::cout << " [ERR]";
                    std::cout << std::endl;

                    std::cout << "  [Header] FD Flags:  0x" << std::hex << static_cast<int>(receive_buffer[i].frame.flags) << std::dec;
                    if (receive_buffer[i].frame.flags & CANFD_BRS) std::cout << " [BRS]";
                    if (receive_buffer[i].frame.flags & CANFD_ESI) std::cout << " [ESI]";
                    std::cout << std::endl;

                    std::cout << "  [CAN_ID] 0x" << std::hex << std::setw(3) << std::setfill('0')
                              << can_id << std::dec << " (ID=" << can_id << ")" << std::endl;

                    std::cout << "  [DLC]    " << static_cast<int>(receive_buffer[i].frame.len) << " bytes" << std::endl;

                    // 十六进制数据
                    std::cout << "  [DATA]   HEX: ";
                    for (uint8_t j = 0; j < receive_buffer[i].frame.len && j < 64; j++) {
                        std::cout << std::hex << std::setw(2) << std::setfill('0') << std::uppercase
                                  << static_cast<int>(receive_buffer[i].frame.data[j]) << " ";
                    }
                    std::cout << std::dec << std::nouppercase << std::endl;

                    // 十六进制转储
                    std::cout << "  [DUMP]   ";
                    for (uint8_t j = 0; j < receive_buffer[i].frame.len && j < 64; j++) {
                        std::cout << std::setw(3) << static_cast<int>(receive_buffer[i].frame.data[j]) << " ";
                    }
                    std::cout << std::endl;

                    std::cout << "  ===========================================" << std::endl;
                }

                // 如果是电机响应 (ID 1-30)，使用 DM 格式解析并保存
                if (can_id >= 1 && can_id <= 30) {
                    if (receive_buffer[i].frame.len >= 8) {
                        // 使用 DM 格式解码器
                        DMMotorFrameDecoder::DMMotorData dm_data = DMMotorFrameDecoder::DecodeFrameFD(receive_buffer[i]);

                        // 打印原始数据流（始终显示，不受 verbose 控制）
                        std::cout << "\n  [DM Motor ID=" << can_id << "] " << DMMotorFrameDecoder::FormatRaw(dm_data) << std::endl;

                        // 打印解码后的数据
                        std::cout << "  [DECODED] " << DMMotorFrameDecoder::FormatDecoded(dm_data) << std::endl;

                        // 打印错误描述
                        if (dm_data.error != 0x0 && dm_data.error != 0x1) {
                            std::cout << "  [ERROR] " << DMMotorFrameDecoder::GetErrorDescription(dm_data.error) << std::endl;
                        }

                        motor_response_count_++;

                        // 同时更新旧的 MotorResponse 结构（兼容保留）
                        MotorResponse resp = {0};
                        resp.motor_id = dm_data.motor_id;
                        resp.timestamp = receive_buffer[i].timestamp;
                        resp.raw_pos = dm_data.position;
                        resp.raw_vel = dm_data.velocity;
                        resp.raw_torque = dm_data.torque;
                        resp.temp = dm_data.temp_mos;
                        resp.status = dm_data.error;
                        resp.valid = true;

                        // 更新最新状态
                        {
                            std::lock_guard<std::mutex> lock(motor_states_mutex_);
                            latest_motor_states_[can_id] = resp;
                        }

                        if (verbose) {
                            std::cout << "  [DM Format] Position: " << dm_data.position
                                      << ", Velocity: " << dm_data.velocity
                                      << ", Torque: " << dm_data.torque << std::endl;
                        }
                    }
                }
            }

            // 打印简短接收信息
            if (!verbose) {
                std::cout << "[RX] Received " << received << " frames";
                for (uint32_t i = 0; i < received && i < 3; i++) {
                    uint32_t can_id = GET_ID(receive_buffer[i].frame.can_id);
                    std::cout << " | ID=0x" << std::hex << std::setw(3) << std::setfill('0')
                              << can_id << std::dec;
                }
                if (received > 3) {
                    std::cout << " ...";
                }
                std::cout << std::endl;
            }
        } else {
            receive_fail_count++;
        }

        // 每秒打印一次统计（只在有新数据时）
        auto now = std::chrono::steady_clock::now();
        if (now - last_stats_time >= std::chrono::seconds(1)) {
            uint64_t current_count = receive_count_.load();
            uint64_t delta = current_count - last_count;
            uint64_t motor_resp = motor_response_count_.load();

            // 只在有新数据或电机响应时显示统计
            if (delta > 0 || motor_resp > 0) {
                std::cout << "\n[STAT] RX: " << current_count << " total (+" << delta
                          << "/s) | Motor responses: " << motor_resp << std::endl;
            }

            last_stats_time = now;
            last_count = current_count;
        }

        // 短暂休眠避免 CPU 占用过高
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << ">>> CAN Receive Thread Stopped <<<" << std::endl;
}

// 启动 CAN 接收线程
static bool StartCANReceiveThread(CHANNEL_HANDLE channel_handle, uint8_t channel, bool verbose) {
    if (receive_thread_running) {
        std::cout << "CAN receive thread already running" << std::endl;
        return true;
    }

    receive_thread_running = true;
    can_receive_thread = std::thread(CANReceiveThread, channel_handle, channel, verbose);
    can_receive_thread.detach();

    return true;
}

// 停止 CAN 接收线程
static void StopCANReceiveThread() {
    if (receive_thread_running) {
        receive_thread_running = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// 关闭直接命令模式的 ZCAN 连接
static void CloseDirectZCAN() {
    // 先停止接收线程
    StopCANReceiveThread();

    if (direct_zlg_config.initialized) {
        if (direct_zlg_config.channel_handle != INVALID_CHANNEL_HANDLE) {
            ZCAN_ResetCAN(direct_zlg_config.channel_handle);
        }
        if (direct_zlg_config.device_handle != INVALID_DEVICE_HANDLE) {
            ZCAN_CloseDevice(direct_zlg_config.device_handle);
        }
        direct_zlg_config.initialized = false;
        std::cout << "ZCAN connection closed" << std::endl;
    }
}

// ==================== Single Port Sender ====================
// Encapsulates sending logic for a single CAN port
class PortSender {
public:
    // 构造函数：自己打开设备（用于单端口模式）
    PortSender(const PortConfig& port_config, const std::string& zlg_ip,
               int arb_baud, int data_baud)
        : port_config_(port_config), zlg_ip_(zlg_ip),
          arb_baud_(arb_baud), data_baud_(data_baud),
          owns_device_(true), device_index_(0) {}

    // 构造函数：使用外部传入的设备句柄（用于多端口模式）
    PortSender(const PortConfig& port_config, DEVICE_HANDLE device_handle,
               const std::string& zlg_ip, int arb_baud, int data_baud, int device_index)
        : port_config_(port_config), device_handle_(device_handle), zlg_ip_(zlg_ip),
          arb_baud_(arb_baud), data_baud_(data_baud),
          owns_device_(false), device_index_(device_index) {}

    ~PortSender() {
        if (channel_handle_ && channel_handle_ != INVALID_CHANNEL_HANDLE) {
            ZCAN_ResetCAN(channel_handle_);
        }
        // 只有拥有设备时才关闭设备
        if (owns_device_ && device_handle_ && device_handle_ != INVALID_DEVICE_HANDLE) {
            ZCAN_CloseDevice(device_handle_);
        }
    }

    bool Initialize() {
        std::cout << "  [Port " << port_config_.port << "] Initializing..." << std::endl;

        // 1. Open device (只在拥有设备时才打开)
        if (owns_device_) {
            // 注意：ZCAN_OpenDevice 的第二个参数是设备索引(device index)，不是通道号
            // 对于单个多通道设备，设备索引应该固定为 0
            device_handle_ = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, 0, 0);
            if (device_handle_ == INVALID_DEVICE_HANDLE) {
                std::cerr << "  [Port " << port_config_.port << "] Failed to open device" << std::endl;
                return false;
            }
        } else {
            std::cout << "  [Port " << port_config_.port << "] Using shared device handle" << std::endl;
        }

        // 2. Initialize CAN channel
        ZCAN_CHANNEL_INIT_CONFIG init_config;
        memset(&init_config, 0, sizeof(init_config));
        init_config.can_type = TYPE_CANFD;
        init_config.canfd.acc_code = 0;
        init_config.canfd.acc_mask = 0;
        init_config.canfd.abit_timing = arb_baud_;
        init_config.canfd.dbit_timing = data_baud_;
        init_config.canfd.brp = 0;
        init_config.canfd.filter = 0;
        init_config.canfd.mode = 0;

        // 初始化CAN - 使用 port_config_.channel（每个端口对应一个通道）
        // 注意：现在使用单一设备句柄，通过不同的通道号访问不同的TCP端口
        channel_handle_ = ZCAN_InitCAN(device_handle_, port_config_.channel, &init_config);
        if (channel_handle_ == INVALID_CHANNEL_HANDLE) {
            std::cerr << "  [Port " << port_config_.port << "] Failed to init CAN" << std::endl;
            ZCAN_CloseDevice(device_handle_);
            return false;
        }

        // 设置IP和端口 - 使用 device_index_ 作为设备索引和通道号
        // 注意：根据motorenable.cpp的模式，ZCAN_SetReference的两个参数都是device_index_
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, device_index_, device_index_,
                         CMD_DESIP, (void*)zlg_ip_.c_str());
        uint32_t port_val = port_config_.port;
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, device_index_, device_index_,
                         CMD_DESPORT, &port_val);

        // 4. Start CAN
        if (ZCAN_StartCAN(channel_handle_) == STATUS_ERR) {
            std::cerr << "  [Port " << port_config_.port << "] Failed to start CAN" << std::endl;
            ZCAN_CloseDevice(device_handle_);
            return false;
        }

        std::cout << "  [Port " << port_config_.port << "] Initialized (motors "
                  << (port_config_.motor_offset + 1) << "-"
                  << (port_config_.motor_offset + port_config_.motor_count) << ")" << std::endl;

        initialized_ = true;
        return true;
    }

    int SendMotorCommands(const std::vector<MotorCommandCan>& commands) {
        if (!initialized_) {
            error_count_ += commands.size();
            return 0;
        }

        std::vector<ZCAN_Transmit_Data> frames;
        frames.reserve(commands.size());

        static int debug_count = 0;
        for (const auto& cmd : commands) {
            ZCAN_Transmit_Data transmit_data;
            memset(&transmit_data, 0, sizeof(transmit_data));

            // DM 电机 MIT 模式: CAN ID = local_motor_id + MIT_MODE
            // 参考: motor_config.c 第 633 行: uint16_t id = motor_id + MIT_MODE;
            uint32_t can_id_to_use = cmd.motor_id + MIT_MODE;
            transmit_data.frame.can_id = MAKE_CAN_ID(can_id_to_use, 0, 0, 0);
            transmit_data.frame.can_dlc = 8;

            uint8_t can_data[8];
            MotorCommandToCanData(cmd, can_data);
            for (int i = 0; i < 8; i++) {
                transmit_data.frame.data[i] = can_data[i];
            }

            // 调试输出：前10个命令的详细信息
            if (debug_count < 10 && !cmd.is_heartbeat) {
                printf("[CAN_DEBUG] Motor%d(pos=%.3f,vel=%.3f,kp=%.1f,kd=%.2f,tau=%.2f) -> ",
                       cmd.global_motor_id, cmd.pos, cmd.vel, cmd.kp, cmd.kd, cmd.torq);
                printf("CAN_ID=0x%03x Data: ", can_id_to_use);
                for (int i = 0; i < 8; i++) {
                    printf("%02x ", can_data[i]);
                }
                printf("\n");
                debug_count++;
            }

            transmit_data.transmit_type = 0;
            frames.push_back(transmit_data);
        }

        uint32_t ret = ZCAN_Transmit(channel_handle_, frames.data(), frames.size());

        send_count_ += commands.size();
        if (ret == commands.size()) {
            send_success_count_ += ret;
        } else {
            send_success_count_ += ret;
            error_count_ += (commands.size() - ret);
        }

        return ret;
    }

    bool SendEnableCommand(int local_motor_id) {
        if (!initialized_) return false;
        // DM 电机 MIT 模式使能命令: CAN ID = local_motor_id + MIT_MODE
        uint8_t enable_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
        ZCAN_Transmit_Data transmit_data;
        memset(&transmit_data, 0, sizeof(transmit_data));
        transmit_data.frame.can_id = MAKE_CAN_ID(local_motor_id + MIT_MODE, 0, 0, 0);
        transmit_data.frame.can_dlc = 8;
        memcpy(transmit_data.frame.data, enable_frame, 8);
        transmit_data.transmit_type = 0;
        return ZCAN_Transmit(channel_handle_, &transmit_data, 1) == 1;
    }

    bool SendDisableCommand(int local_motor_id) {
        if (!initialized_) return false;
        // DM 电机 MIT 模式失能命令: CAN ID = local_motor_id + MIT_MODE
        uint8_t disable_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
        ZCAN_Transmit_Data transmit_data;
        memset(&transmit_data, 0, sizeof(transmit_data));
        transmit_data.frame.can_id = MAKE_CAN_ID(local_motor_id + MIT_MODE, 0, 0, 0);
        transmit_data.frame.can_dlc = 8;
        memcpy(transmit_data.frame.data, disable_frame, 8);
        transmit_data.transmit_type = 0;
        return ZCAN_Transmit(channel_handle_, &transmit_data, 1) == 1;
    }

    bool IsInitialized() const { return initialized_; }
    uint64_t GetSendCount() const { return send_count_; }
    uint64_t GetErrorCount() const { return error_count_; }
    CHANNEL_HANDLE GetChannelHandleForZero() const { return channel_handle_; }

private:
    PortConfig port_config_;
    std::string zlg_ip_;
    int arb_baud_;
    int data_baud_;
    bool owns_device_;  // 是否拥有设备（true=自己打开并负责关闭，false=使用外部句柄）
    int device_index_;  // ZCAN 设备索引（用于多设备模式）

    DEVICE_HANDLE device_handle_ = INVALID_DEVICE_HANDLE;
    CHANNEL_HANDLE channel_handle_ = INVALID_CHANNEL_HANDLE;
    bool initialized_ = false;

    std::atomic<uint64_t> send_count_{0};
    std::atomic<uint64_t> send_success_count_{0};
    std::atomic<uint64_t> error_count_{0};
};

// ==================== Multi-Port Motor Manager ====================
// Coordinates sending to 4 CAN ports
class MultiPortMotorManager {
public:
    MultiPortMotorManager(const std::string& zlg_ip, int arb_baud, int data_baud)
        : zlg_ip_(zlg_ip), arb_baud_(arb_baud), data_baud_(data_baud), device_handle_(INVALID_DEVICE_HANDLE) {
        // 初始化设备句柄为无效
    }

    ~MultiPortMotorManager() {
        // 关闭单一设备句柄
        if (device_handle_ && device_handle_ != INVALID_DEVICE_HANDLE) {
            ZCAN_CloseDevice(device_handle_);
            device_handle_ = INVALID_DEVICE_HANDLE;
        }
    }

    bool Initialize() {
        std::cout << "\n=== Initializing Multi-Port Motor Manager ===" << std::endl;
        std::cout << "ZLG IP: " << zlg_ip_ << std::endl;
        std::cout << "Arbitration Baud: " << arb_baud_ << " bps" << std::endl;
        std::cout << "Data Baud: " << data_baud_ << " bps" << std::endl;

        // 1. 打开 1 个设备，支持 4 个通道
        // 注意：ZLG CANFDNET-400U 在单设备多通道模式下
        // 所有 4 个 TCP 端口 (8000-8003) 都使用同一个设备句柄
        // 但每个通道需要单独初始化
        std::cout << "[1/5] Opening 1 ZCAN device for 4 channels..." << std::endl;
        device_handle_ = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, 0, 0);
        if (device_handle_ == INVALID_DEVICE_HANDLE) {
            std::cerr << "Failed to open ZCAN device 0" << std::endl;
            return false;
        }
        std::cout << "  Device 0 opened (handles all 4 channels)" << std::endl;
        std::cout << "[2/5] Device opened successfully" << std::endl;

        // 2. 为每个端口创建 PortSender，都使用同一个设备句柄，但使用不同的通道号
        for (int i = 0; i < 4; i++) {
            port_senders_[i] = std::make_unique<PortSender>(
                PORT_CONFIGS[i], device_handle_, zlg_ip_, arb_baud_, data_baud_, PORT_CONFIGS[i].channel);
            if (!port_senders_[i]->Initialize()) {
                std::cerr << "Failed to initialize port " << PORT_CONFIGS[i].port << std::endl;
                return false;
            }
        }

        std::cout << "[3/5] All 4 ports initialized" << std::endl;
        std::cout << "=== Multi-Port Motor Manager Ready ===" << std::endl;
        return true;
    }

    int SendAllMotorCommands(const std::vector<MotorCommandCan>& all_commands) {
        // Group commands by port
        std::array<std::vector<MotorCommandCan>, 4> port_commands;

        // 检查是否包含 motor_id 30
        bool has_motor_30 = false;
        for (const auto& cmd : all_commands) {
            if (cmd.motor_id == 30) {
                has_motor_30 = true;
                break;
            }
        }

        for (const auto& cmd : all_commands) {
            int port_idx = GetPortIndexForMotor(cmd.global_motor_id);
            if (port_idx >= 0 && port_senders_[port_idx]->IsInitialized()) {
                MotorCommandCan local_cmd = cmd;
                local_cmd.motor_id = GetLocalCanId(cmd.global_motor_id);
                port_commands[port_idx].push_back(local_cmd);
            } else if (port_idx >= 0) {
                // 端口索引正确但未初始化
                static int init_warn_count = 0;
                if (init_warn_count < 10) {
                    std::cout << "[WARNING] Port " << (8000 + port_idx)
                              << " for motor " << cmd.global_motor_id << " NOT INITIALIZED!" << std::endl;
                    init_warn_count++;
                }
            } else {
                // 端口索引错误
                static int idx_err_count = 0;
                if (idx_err_count < 10) {
                    std::cout << "[ERROR] Invalid port_idx=" << port_idx
                              << " for motor " << cmd.global_motor_id << std::endl;
                    idx_err_count++;
                }
            }
        }

        // 调试输出：显示每个端口的电机数量
        static int debug_count = 0;
        if (debug_count < 10) {
            std::cout << "[DEBUG] 端口分配: ";
            for (int i = 0; i < 4; i++) {
                std::cout << "800" << i << "=" << port_commands[i].size() << " ";
            }
            std::cout << "(总计=" << all_commands.size() << ")" << std::endl;
            debug_count++;
        }

        // Send to each port sequentially
        int total_sent = 0;
        for (int i = 0; i < 4; i++) {
            if (!port_commands[i].empty()) {
                int sent = port_senders_[i]->SendMotorCommands(port_commands[i]);
                total_sent += sent;

                // 调试：显示每个端口的发送结果
                static int send_debug_count = 0;
                if (send_debug_count < 10) {
                    std::cout << "[SEND] Port " << (8000 + i)
                              << ": commands=" << port_commands[i].size()
                              << ", sent=" << sent << std::endl;
                    send_debug_count++;
                }
            }
        }

        return total_sent;
    }

    bool SendEnableCommand(int global_motor_id) {
        int port_idx = GetPortIndexForMotor(global_motor_id);
        if (port_idx < 0 || !port_senders_[port_idx]->IsInitialized()) {
            return false;
        }
        int local_can_id = GetLocalCanId(global_motor_id);
        return port_senders_[port_idx]->SendEnableCommand(local_can_id);
    }

    bool SendDisableCommand(int global_motor_id) {
        int port_idx = GetPortIndexForMotor(global_motor_id);
        if (port_idx < 0 || !port_senders_[port_idx]->IsInitialized()) {
            return false;
        }
        int local_can_id = GetLocalCanId(global_motor_id);
        return port_senders_[port_idx]->SendDisableCommand(local_can_id);
    }

    // 发送标零命令: FF FF FF FF FF FF FF FE
    bool SendZeroCommand(int global_motor_id) {
        int port_idx = GetPortIndexForMotor(global_motor_id);
        if (port_idx < 0 || !port_senders_[port_idx]->IsInitialized()) {
            return false;
        }
        int local_can_id = GetLocalCanId(global_motor_id);

        // 构造标零命令数据帧: FF FF FF FF FF FF FF FE
        // DM 电机 MIT 模式: CAN ID = local_can_id + MIT_MODE
        ZCAN_Transmit_Data transmit_data;
        memset(&transmit_data, 0, sizeof(transmit_data));
        transmit_data.frame.can_id = MAKE_CAN_ID(local_can_id + MIT_MODE, 0, 0, 0);
        transmit_data.frame.can_dlc = 8;
        transmit_data.frame.data[0] = 0xFF;
        transmit_data.frame.data[1] = 0xFF;
        transmit_data.frame.data[2] = 0xFF;
        transmit_data.frame.data[3] = 0xFF;
        transmit_data.frame.data[4] = 0xFF;
        transmit_data.frame.data[5] = 0xFF;
        transmit_data.frame.data[6] = 0xFF;
        transmit_data.frame.data[7] = 0xFE;  // 标零命令标识
        transmit_data.transmit_type = 0;

        // 获取 channel handle 并发送
        CHANNEL_HANDLE channel_handle = port_senders_[port_idx]->GetChannelHandleForZero();
        if (channel_handle == INVALID_CHANNEL_HANDLE) {
            return false;
        }

        uint32_t ret = ZCAN_Transmit(channel_handle, &transmit_data, 1);
        return ret == 1;
    }

    void PrintStatus() {
        std::cout << "\n=== Multi-Port Status ===" << std::endl;
        for (int i = 0; i < 4; i++) {
            std::cout << "  Port " << PORT_CONFIGS[i].port << ": ";
            if (port_senders_[i] && port_senders_[i]->IsInitialized()) {
                std::cout << "OK, Sent=" << port_senders_[i]->GetSendCount()
                          << ", Errors=" << port_senders_[i]->GetErrorCount();
            } else {
                std::cout << "NOT INITIALIZED";
            }
            std::cout << std::endl;
        }
        std::cout << "=========================" << std::endl;
    }

private:
    int GetPortIndexForMotor(int global_motor_id) const {
        // 统一的电机到端口映射 (motor_id: 1-30)
        // 与PORT_CONFIGS保持一致
        // 端口8000: 电机1-10   (motor_offset=0)
        // 端口8001: 电机11-17  (motor_offset=10)
        // 端口8002: 电机18-24  (motor_offset=17)
        // 端口8003: 电机25-30  (motor_offset=24)
        if (global_motor_id >= 1 && global_motor_id <= 10)  return 0;
        if (global_motor_id >= 11 && global_motor_id <= 17) return 1;
        if (global_motor_id >= 18 && global_motor_id <= 24) return 2;
        if (global_motor_id >= 25 && global_motor_id <= 30) return 3;
        return -1;
    }

    int GetLocalCanId(int global_motor_id) const {
        // 根据端口计算本地CAN ID
        // 8000 (offset=0): 全局1-10  → 本地1-10
        // 8001 (offset=10): 全局11-17 → 本地1-7
        // 8002 (offset=17): 全局18-24 → 本地1-7
        // 8003 (offset=24): 全局25-30 → 本地1-6
        for (const auto& config : PORT_CONFIGS) {
            int motor_start = config.motor_offset + 1;
            int motor_end = config.motor_offset + config.motor_count;
            if (global_motor_id >= motor_start && global_motor_id <= motor_end) {
                return global_motor_id - config.motor_offset;  // 本地CAN ID
            }
        }
        return -1;
    }

private:
    std::string zlg_ip_;
    int arb_baud_;
    int data_baud_;
    DEVICE_HANDLE device_handle_;  // 单一设备句柄（支持4个通道）
    std::array<std::unique_ptr<PortSender>, 4> port_senders_;
};

// ROS2 桥接类 (继承自 rclcpp::Node)
class ROS2_to_TCP_Bridge : public rclcpp::Node {
private:
    TcpConfig tcp_config_;
    // Replace single ZhilgongConfig with multi-port manager
    std::unique_ptr<MultiPortMotorManager> multi_port_manager_;

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

    // 追踪活跃的电机（收到过 ROS2 命令的电机）
    std::array<bool, G1_NUM_MOTOR> active_motors_;

    // 最大速度限制 (rad/s)
    static constexpr double MAX_VELOCITY = 0.3;

    struct MotorCommandHistory {
        MotorCommandCan current;
        MotorCommandCan previous;
        bool has_previous = false;
        std::chrono::high_resolution_clock::time_point current_timestamp;
        std::chrono::high_resolution_clock::time_point previous_timestamp;

        // 速度限制相关：记录上次实际发送的位置和时间
        double last_sent_pos = 0.0;
        double last_sent_time = 0.0;
        bool has_sent = false;  // 是否已经发送过命令
    };
    std::array<MotorCommandHistory, G1_NUM_MOTOR> command_history_;

public:
    ROS2_to_TCP_Bridge(bool verbose = false, bool motor_cmd_enabled = false) : Node("ros2_to_tcp_bridge") {
        verbose_logging_ = verbose;
        motor_cmd_enabled_ = motor_cmd_enabled;

        // motor_id should start from 1, not 0
        for (int i = 0; i < G1_NUM_MOTOR; i++) {
            motor_commands_[i] = {static_cast<uint16_t>(i + 1), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
            active_motors_[i] = false;  // 初始化为非活跃状态
        }

        std::cout << "ROS2-to-TCP Bridge Initializing..." << std::endl;
        std::cout << "  Verbose logging: " << (verbose_logging_ ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "  Motor command: " << (motor_cmd_enabled_ ? "ENABLED" : "DISABLED") << std::endl;
    }

    // Initialize multi-port manager (replaces SetZlgConfig)
    bool InitializeMultiPortManager(const std::string& zlg_ip, int arb_baud, int data_baud) {
        multi_port_manager_ = std::make_unique<MultiPortMotorManager>(zlg_ip, arb_baud, data_baud);
        return multi_port_manager_->Initialize();
    }

    ~ROS2_to_TCP_Bridge() {
        should_stop_ = true;
        if (can_send_thread_.joinable()) {
            can_send_thread_.join();
        }

        if (tcp_config_.initialized && tcp_config_.socket_fd >= 0) {
            close(tcp_config_.socket_fd);
        }

        // multi_port_manager_ auto-cleanup via unique_ptr destructor
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

    void InitializeROS2() {
        std::cout << "Initializing ROS2 subscriber on topic: " << ROS2_CMD_TOPIC << std::endl;

        lowcmd_subscriber_ = this->create_subscription<xixilowcmd::msg::LowCmd>(
            ROS2_CMD_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
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

        // 调试输出：显示收到的电机 ID
        if (msg->motor_cmd.size() > 0 && ros2_msg_count_ <= 5) {
            std::cout << "[DEBUG] 收到 " << msg->motor_cmd.size() << " 个电机命令: ";
            for (const auto& cmd : msg->motor_cmd) {
                std::cout << "ID" << cmd.id << " ";
            }
            std::cout << std::endl;
        }

        // 使用互斥锁保护 motor_commands_ 和 command_history_
        std::lock_guard<std::mutex> lock(motor_commands_mutex_);

        for (const auto& motor_cmd : msg->motor_cmd) {
            int motor_id = motor_cmd.id;

            // motor_id 范围: 1-30
            if (motor_id >= 1 && motor_id <= G1_NUM_MOTOR) {
                // 转换为数组索引 (0-29)
                int array_idx = motor_id - 1;

                // 保存全局 motor_id (在覆盖 motor_id 为本地 CAN ID 之前)
                motor_commands_[array_idx].global_motor_id = motor_id;

                // 获取该电机的位置限位并进行限幅
                MotorLimits limits = GetMotorLimits(motor_id);
                motor_commands_[array_idx].pos = std::max(limits.p_min, std::min(limits.p_max, motor_cmd.q));
                motor_commands_[array_idx].vel = motor_cmd.dq;
                motor_commands_[array_idx].kp = motor_cmd.kp;
                motor_commands_[array_idx].kd = motor_cmd.kd;
                motor_commands_[array_idx].torq = motor_cmd.tau;

                int can_id = get_can_id_for_motor(motor_id);
                if (can_id > 0) {
                    motor_commands_[array_idx].motor_id = can_id;  // 设置为本地 CAN ID

                    // 只有当 mode != 0 时才标记为活跃（mode=0 表示没有有效控制数据）
                    if (motor_cmd.mode != 0) {
                        active_motors_[array_idx] = true;
                    }

                    auto& history = command_history_[array_idx];

                    if (history.has_previous) {
                        history.previous = history.current;
                        history.previous_timestamp = history.current_timestamp;
                    } else {
                        history.previous = motor_commands_[array_idx];
                        history.previous_timestamp = current_time;
                        history.has_previous = true;
                    }

                    history.current = motor_commands_[array_idx];
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

        // motor_id 范围: 1-30
        if (motor_id < 1 || motor_id > 30) {
            std::cerr << "Invalid motor ID: " << motor_id << " (range: 1-30)" << std::endl;
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

    // Send enable command via multi-port manager
    void SendEnableCommandToZhilgong(int motor_id) {
        if (!multi_port_manager_) {
            std::cerr << "Multi-port manager not initialized" << std::endl;
            return;
        }

        if (motor_id < 1 || motor_id > 30) {
            std::cerr << "Invalid motor ID: " << motor_id << std::endl;
            return;
        }

        std::cout << "\n>>> Sending ENABLE Command <<<" << std::endl;
        std::cout << "  Motor ID: " << motor_id << std::endl;

        if (multi_port_manager_->SendEnableCommand(motor_id)) {
            std::cout << ">>> Enable command sent <<<" << std::endl;
        } else {
            std::cerr << ">>> Enable command FAILED <<<" << std::endl;
        }
    }

    // Send disable command via multi-port manager
    void SendDisableCommandToZhilgong(int motor_id) {
        if (!multi_port_manager_) {
            std::cerr << "Multi-port manager not initialized" << std::endl;
            return;
        }

        if (motor_id < 1 || motor_id > 30) {
            std::cerr << "Invalid motor ID: " << motor_id << std::endl;
            return;
        }

        std::cout << "\n>>> Sending DISABLE Command <<<" << std::endl;
        std::cout << "  Motor ID: " << motor_id << std::endl;

        if (multi_port_manager_->SendDisableCommand(motor_id)) {
            std::cout << ">>> Disable command sent <<<" << std::endl;
        } else {
            std::cerr << ">>> Disable command FAILED <<<" << std::endl;
        }
    }

    // Send custom enable command via multi-port manager
    void SendCustomEnableCommandToZhilgong(int motor_id, uint8_t command) {
        if (!multi_port_manager_) {
            std::cerr << "Multi-port manager not initialized" << std::endl;
            return;
        }

        if (motor_id < 1 || motor_id > 30) {
            std::cerr << "Invalid motor ID: " << motor_id << std::endl;
            return;
        }

        // For custom commands, send as enable (0xFC) or disable (0xFD)
        // Other custom values can be added if needed
        if (command == 0xFC) {
            SendEnableCommandToZhilgong(motor_id);
        } else if (command == 0xFD) {
            SendDisableCommandToZhilgong(motor_id);
        } else {
            std::cout << "\n>>> Sending CUSTOM Command <<<" << std::endl;
            std::cout << "  Motor ID: " << motor_id << std::endl;
            std::cout << "  Command:  0x" << std::hex << static_cast<int>(command) << std::dec << std::endl;
            std::cout << "  Note: Custom command 0x" << std::hex << static_cast<int>(command) << std::dec
                      << " not directly supported, using enable/disable logic" << std::endl;
        }
    }

    // Auto-start sending (for testing, bypass /motor_enable requirement)
    void SetAutoStart() {
        std::cout << "\n>>> AUTO-START mode enabled <<<<" << std::endl;
        std::cout << ">>> 500Hz motor command sending STARTED <<<" << std::endl;
        std::cout << ">>> Initial state: sending HEARTBEAT commands (0xCC) <<<" << std::endl;
        std::cout << ">>> Will send control commands when /lowcmd data is received <<<" << std::endl;

        // 在auto-start模式下，初始状态所有电机标记为非活跃
        // 这样会立即发送心跳命令，直到收到 /lowcmd Topic 的有效数据
        for (int i = 0; i < G1_NUM_MOTOR; i++) {
            active_motors_[i] = false;  // 初始为非活跃，发送心跳命令

            // 初始化电机命令为默认值（收到有效数据后会更新）
            int motor_id = i + 1;
            motor_commands_[i].global_motor_id = motor_id;
            motor_commands_[i].motor_id = get_can_id_for_motor(motor_id);
            motor_commands_[i].pos = 0.0f;
            motor_commands_[i].vel = 0.0f;
            motor_commands_[i].kp = 30.0f;   // 位置增益，收到数据后保持位置
            motor_commands_[i].kd = 0.5f;    // 速度增益，提供阻尼
            motor_commands_[i].torq = 0.0f;
            motor_commands_[i].is_heartbeat = false;

            // 设置命令历史记录，避免插值时出现问题
            auto& history = command_history_[i];
            if (!history.has_previous) {
                history.current = motor_commands_[i];
                history.previous = motor_commands_[i];
                history.has_previous = true;
                // 使用旧的时间戳，确保启动时立即发送心跳命令
                auto old_time = std::chrono::high_resolution_clock::now() - std::chrono::seconds(10);
                history.current_timestamp = old_time;
                history.previous_timestamp = old_time;
            }
        }

        can_send_started_ = true;
    }

    // Send all motor commands via multi-port manager
    int SendAllMotorCommands(const std::vector<MotorCommandCan>& commands) {
        if (!multi_port_manager_) {
            send_error_count_ += commands.size();
            return 0;
        }
        send_count_ += commands.size();
        int sent = multi_port_manager_->SendAllMotorCommands(commands);
        send_success_count_ += sent;
        send_error_count_ += (commands.size() - sent);
        return sent;
    }

    // 收集所有需要发送的电机命令到批量数组
    // - 有活跃控制命令的电机：发送插值后的控制命令
    // - 没有活跃控制命令的电机：发送心跳命令（pos=0, vel=0, kp=0, kd=0, tau=0）
    std::vector<MotorCommandCan> CollectMotorCommands(std::chrono::high_resolution_clock::time_point current_time) {
        std::vector<MotorCommandCan> commands;
        commands.reserve(G1_NUM_MOTOR);

        // 超时阈值（秒）：如果超过这个时间没有收到新数据，取消活跃状态
        const double COMMAND_TIMEOUT_SECONDS = 0.5;  // 500ms 超时
        auto current_timestamp_double = std::chrono::duration<double>(current_time.time_since_epoch()).count();

        for (int motor_id = 1; motor_id <= G1_NUM_MOTOR; motor_id++) {  // 遍历所有30个电机
            // 转换为数组索引 (motor_id 1-30 -> array_idx 0-29)
            int array_idx = motor_id - 1;

            int can_id = get_can_id_for_motor(motor_id);
            if (can_id > 0) {
                // 检查该电机是否活跃且有控制命令
                if (active_motors_[array_idx]) {
                    MotorCommandCan cmd_to_send;
                    cmd_to_send.motor_id = can_id;
                    cmd_to_send.global_motor_id = motor_id;

                    auto& history = command_history_[array_idx];
                    double last_command_time = std::chrono::duration<double>(history.current_timestamp.time_since_epoch()).count();
                    double time_since_last_command = current_timestamp_double - last_command_time;

                    if (time_since_last_command > COMMAND_TIMEOUT_SECONDS) {
                        // 超时：取消活跃状态，发送心跳命令
                        if (verbose_logging_ || motor_id <= 6 || motor_id == 30) {
                            std::cout << "[TIMEOUT] Motor " << motor_id
                                      << " 超时 (" << time_since_last_command << "s > "
                                      << COMMAND_TIMEOUT_SECONDS << "s), 切换为心跳命令" << std::endl;
                        }
                        active_motors_[array_idx] = false;  // 取消活跃状态
                        cmd_to_send.is_heartbeat = true;   // 发送心跳命令
                        commands.push_back(cmd_to_send);
                    } else {
                        // 未超时：发送插值后的控制命令
                        auto current_timestamp = std::chrono::duration<double>(current_time.time_since_epoch()).count();

                        // 调试输出：检查是否进入插值逻辑
                        static int debug_entry_count = 0;
                        if (++debug_entry_count % 1000 == 0 && (motor_id <= 3 || motor_id == 30)) {
                            std::cout << "[DEBUG] Motor " << motor_id
                                      << " has_previous=" << history.has_previous
                                      << " active=true" << std::endl;
                        }

                        if (history.has_previous) {
                            double prev_time = std::chrono::duration<double>(history.previous_timestamp.time_since_epoch()).count();
                            double curr_time = std::chrono::duration<double>(history.current_timestamp.time_since_epoch()).count();

                            cmd_to_send = interpolateCommand(history.previous, history.current,
                                                           prev_time, curr_time, current_timestamp,
                                                           history);
                        } else {
                            cmd_to_send = history.current;
                        }
                        cmd_to_send.is_heartbeat = false;  // 标记为控制命令

                        // 更新发送记录（用于下一次速度限制计算）
                        history.last_sent_pos = cmd_to_send.pos;
                        history.last_sent_time = current_timestamp;
                        history.has_sent = true;

                        // 添加活跃的电机命令
                        commands.push_back(cmd_to_send);
                    }
                } else {
                    // 不活跃的电机发送心跳命令
                    MotorCommandCan cmd_to_send;
                    cmd_to_send.motor_id = can_id;
                    cmd_to_send.global_motor_id = motor_id;
                    cmd_to_send.is_heartbeat = true;
                    commands.push_back(cmd_to_send);
                }
            }
        }

        return commands;
    }

    MotorCommandCan interpolateCommand(const MotorCommandCan& prev, const MotorCommandCan& curr,
                                       double prev_time, double curr_time, double target_time,
                                       MotorCommandHistory& history) {
        // 调试输出：确认函数被调用
        static int call_count = 0;
        if (++call_count % 1000 == 0) {
            std::cout << "[INTERPOLATE_CALL] count=" << call_count
                      << " prev.pos=" << prev.pos
                      << " curr.pos=" << curr.pos
                      << " prev_time=" << prev_time
                      << " curr_time=" << curr_time
                      << " target_time=" << target_time
                      << std::endl;
        }

        MotorCommandCan result = curr;

        if (curr_time > prev_time) {
            double t = (target_time - prev_time) / (curr_time - prev_time);
            t = std::max(0.0, std::min(1.0, t));

            // 先计算理论插值位置
            double interpolated_pos = prev.pos + t * (curr.pos - prev.pos);

            // 速度限制：基于上次实际发送的位置，累积执行速度限制
            if (history.has_sent) {
                double dt = target_time - history.last_sent_time;
                if (dt > 0) {
                    double max_delta = MAX_VELOCITY * dt;
                    double actual_delta = interpolated_pos - history.last_sent_pos;

                    // 如果变化超过最大速度，限制位置增量
                    if (std::abs(actual_delta) > max_delta) {
                        // 调试输出
                        static int debug_count = 0;
                        if (++debug_count % 500 == 0) {
                            std::cout << "[VEL_LIMIT] dt=" << dt
                                      << " max_delta=" << max_delta
                                      << " actual_delta=" << actual_delta
                                      << " last_sent=" << history.last_sent_pos
                                      << " target=" << interpolated_pos
                                      << " -> limited=" << (history.last_sent_pos + (actual_delta >= 0 ? 1.0 : -1.0) * max_delta)
                                      << std::endl;
                        }
                        double sign = (actual_delta >= 0) ? 1.0 : -1.0;
                        interpolated_pos = history.last_sent_pos + sign * max_delta;
                    } else {
                        // 速度未超限，也打印一下确认逻辑正常
                        static int debug_count2 = 0;
                        if (++debug_count2 % 1000 == 0) {
                            std::cout << "[VEL_OK] dt=" << dt
                                      << " delta=" << actual_delta
                                      << " max=" << max_delta
                                      << " pos=" << interpolated_pos
                                      << std::endl;
                        }
                    }
                }
            }

            result.pos = interpolated_pos;
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

                    // ==================== 批量队列发送模式 ====================
                    // 收集所有电机命令到批量数组
                    std::vector<MotorCommandCan> motor_commands = CollectMotorCommands(now);

                    // 使用多端口管理器发送所有电机命令
                    // 自动分发到对应的端口 (8000, 8001, 8002, 8003)
                    if (!motor_commands.empty()) {
                        SendAllMotorCommands(motor_commands);
                    }

                    // 更新下次发送时间
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
                              << "4-Port: " << (multi_port_manager_ ? "Connected" : "Disconnected")
                              << std::endl;
                } else if (first_log) {
                    // 第一次开始发送，打印标题
                    std::cout << "\n========== Motor Command Sending Started ==========" << std::endl;
                    std::cout << "[STAT] "
                              << "ROS2_msg: " << current_ros2_count << " (+" << ros2_delta << "/s) | "
                              << "Send: " << current_send_count << " (+" << sends_delta << "/s) | "
                              << "Success: " << current_success_count << " (+" << success_delta << "/s) | "
                              << "Error: " << current_error_count << " (+" << error_delta << "/s) | "
                              << "4-Port: " << (multi_port_manager_ ? "Connected" : "Disconnected")
                              << std::endl;
                    first_log = false;
                } else {
                    // 正常发送状态
                    std::cout << "[STAT] "
                              << "ROS2_msg: " << current_ros2_count << " (+" << ros2_delta << "/s) | "
                              << "Send: " << current_send_count << " (+" << sends_delta << "/s) | "
                              << "Success: " << current_success_count << " (+" << success_delta << "/s) | "
                              << "Error: " << current_error_count << " (+" << error_delta << "/s) | "
                              << "4-Port: " << (multi_port_manager_ ? "Connected" : "Disconnected")
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
        bool verbose = false;
        bool motor_cmd_enabled = false;
        bool auto_start = false;  // 自动开始发送，无需等待/motor_enable话题
        bool quiet = true;  // 默认启用 quiet 模式，屏蔽 ZLG [SYS] 输出

        // CAN-FD 波特率配置
        int arb_baud = 1000000;  // 仲裁域波特率 (默认 1M bps)
        int data_baud = 5000000;  // 数据域波特率 (默认 5M bps)

        // 直接命令模式
        bool direct_mode = false;
        int direct_motor_id = -1;
        std::string motor_id_str;  // 电机 ID 字符串（用于范围模式，如 "1-5"）
        bool direct_enable = false;  // true=enable, false=disable
        bool direct_zero = false;   // true=发送标零命令

        // 接收模式
        bool receive_only = false;  // 只接收不发送
        bool receive_mode = false;  // 接收模式 (发送 + 接收)
        int receive_duration = 0;   // 接收持续时间 (秒)，0=无限
    } params;

    if (argc < 2) {
        std::cout << "Usage: motor_controller_with_enable [options]" << std::endl;
        std::cout << "\nOptions:" << std::endl;
        std::cout << "  -v, --verbose           Enable verbose logging" << std::endl;
        std::cout << "  --no-quiet              Show ZLG SDK [SYS] debug output (default: HIDDEN)" << std::endl;
        std::cout << "  --enable-motor-cmd      Enable motor command sending (default: DISABLED)" << std::endl;
        std::cout << "  --auto-start            Auto-start sending without /motor_enable (for testing)" << std::endl;
        std::cout << "  --auto-enable           Auto-enable motors when sending commands (default: DISABLED)" << std::endl;
        std::cout << "  --protocol <type>       Motor protocol: dm (default) or mit" << std::endl;
        std::cout << "\nCAN-FD Baud Rate Configuration:" << std::endl;
        std::cout << "  --arb-baud <rate>       Arbitration baud rate in bps (default: 1000000 = 1M)" << std::endl;
        std::cout << "  --data-baud <rate>      Data baud rate in bps (default: 5000000 = 5M)" << std::endl;
        std::cout << "\nDirect Command Mode (no ROS2 required):" << std::endl;
        std::cout << "  --enable <motor_id>     Send enable command to motor (1-30)" << std::endl;
        std::cout << "  --disable <motor_id>    Send disable command to motor (1-30)" << std::endl;
        std::cout << "  --zero <motor_id>       Send zero command to motor (1-30)" << std::endl;
        std::cout << "\nCAN Receive Mode (monitor CAN bus):" << std::endl;
        std::cout << "  --receive-only          Receive CAN frames only (no transmission)" << std::endl;
        std::cout << "  --receive-and-enable    Send enable command and then monitor responses" << std::endl;
        std::cout << "  --receive-and-disable   Send disable command and then monitor responses" << std::endl;
        std::cout << "  --receive-duration <s>  Receive duration in seconds (0=infinite, default: 0)" << std::endl;
        std::cout << "\nExamples:" << std::endl;
        std::cout << "  # ROS2 mode with motor commands (4-port mode: 8000, 8001, 8002, 8003)" << std::endl;
        std::cout << "  ./motor_controller_with_enable --enable-motor-cmd" << std::endl;
        std::cout << "\n  # ROS2 mode with DM motor protocol (default)" << std::endl;
        std::cout << "  ./motor_controller_with_enable --enable-motor-cmd --protocol dm" << std::endl;
        std::cout << "\n  # ROS2 mode with MIT motor protocol" << std::endl;
        std::cout << "  ./motor_controller_with_enable --enable-motor-cmd --protocol mit" << std::endl;
        std::cout << "\n  # Direct enable/disable mode" << std::endl;
        std::cout << "  ./motor_controller_with_enable --enable 1" << std::endl;
        std::cout << "  ./motor_controller_with_enable --disable 5" << std::endl;
        std::cout << "\n  # Zero motors (set current position as zero)" << std::endl;
        std::cout << "  ./motor_controller_with_enable --zero 1" << std::endl;
        std::cout << "  ./motor_controller_with_enable --zero 1-30" << std::endl;
        std::cout << "\n  # Enable range of motors (1-10)" << std::endl;
        std::cout << "  ./motor_controller_with_enable --enable 1-10" << std::endl;
        std::cout << "\n  # Receive mode - monitor CAN bus (verbose)" << std::endl;
        std::cout << "  ./motor_controller_with_enable --receive-only -v" << std::endl;
        return 1;
    }

    // 解析可选参数
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "-v" || arg == "--verbose") {
            params.verbose = true;
        }
        else if (arg == "--no-quiet") {
            params.quiet = false;
        }
        else if (arg == "--enable-motor-cmd") {
            params.motor_cmd_enabled = true;
        }
        else if (arg == "--auto-start") {
            params.auto_start = true;
        }
        else if (arg == "--auto-enable") {
            g_auto_enable_on_send = true;
            std::cout << ">>> AUTO-ENABLE mode: Motors will be enabled when sending commands <<<" << std::endl;
        }
        else if (arg == "--protocol" && i + 1 < argc) {
            std::string protocol = argv[++i];
            if (protocol == "dm" || protocol == "DM") {
                g_use_dm_protocol = true;
                std::cout << ">>> Using DM Motor Protocol (达妙电机) <<<" << std::endl;
            } else if (protocol == "mit" || protocol == "MIT") {
                g_use_dm_protocol = false;
                std::cout << ">>> Using MIT Motor Protocol <<<" << std::endl;
            } else {
                std::cerr << "Unknown protocol: " << protocol << ". Using default (DM)." << std::endl;
            }
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
        else if (arg == "--zero" && i + 1 < argc) {
            params.direct_mode = true;
            params.direct_enable = false;
            params.direct_zero = true;
            params.motor_id_str = argv[++i];
            params.direct_motor_id = std::atoi(params.motor_id_str.c_str());
        }
        else if (arg == "--receive-only") {
            params.receive_only = true;
            params.direct_mode = true;  // 使用直接命令模式的连接
        }
        else if (arg == "--receive-and-enable" && i + 1 < argc) {
            params.receive_mode = true;
            params.direct_mode = true;
            params.direct_enable = true;
            params.motor_id_str = argv[++i];  // 保存为字符串，用于范围解析
            params.direct_motor_id = std::atoi(params.motor_id_str.c_str());  // 同时保存为整数
        }
        else if (arg == "--receive-and-disable" && i + 1 < argc) {
            params.receive_mode = true;
            params.direct_mode = true;
            params.direct_enable = false;  // disable 模式
            params.motor_id_str = argv[++i];  // 保存为字符串，用于范围解析
            params.direct_motor_id = std::atoi(params.motor_id_str.c_str());  // 同时保存为整数
        }
        else if (arg == "--receive-duration" && i + 1 < argc) {
            params.receive_duration = std::atoi(argv[++i]);
        }
    }

    std::cout << "========================================" << std::endl;
    std::cout << "ROS2 Motor Controller with ZLG Enable" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "ZLG Device IP: " << params.zlg_ip << std::endl;
    std::cout << "Ports: 8000, 8001, 8002, 8003 (4-port mode)" << std::endl;
    std::cout << "CAN-FD Arbitration Baud: " << params.arb_baud << " bps (" << (params.arb_baud / 1000000) << "M)" << std::endl;
    std::cout << "CAN-FD Data Baud: " << params.data_baud << " bps (" << (params.data_baud / 1000000) << "M)" << std::endl;
    std::cout << "Mode: " << (params.direct_mode ? "Direct Command" : "ROS2 Bridge") << std::endl;
    std::cout << "Verbose: " << (params.verbose ? "ON" : "OFF") << std::endl;
    std::cout << "ZLG [SYS] output: " << (params.quiet ? "HIDDEN (use --no-quiet to show)" : "VISIBLE") << std::endl;
    std::cout << "========================================" << std::endl;

    // 根据 quiet 参数设置 ZLG SDK 调试输出模式
    if (params.quiet) {
        SetZlgQuietMode();
    }

    // 直接命令模式 (不需要ROS2)
    if (params.direct_mode) {
        // ==================== 接收模式 ====================
        if (params.receive_only) {
            std::cout << "\n=== Receive-Only Mode ===" << std::endl;
            std::cout << "Monitoring CAN bus without transmission..." << std::endl;

            // 初始化 ZCAN 连接 (使用默认端口 8000, channel 0)
            if (!InitializeDirectZCAN(params.zlg_ip, 8000, 0, params.arb_baud, params.data_baud)) {
                std::cerr << "Failed to initialize ZCAN connection" << std::endl;
                return 1;
            }

            // 启动接收线程
            StartCANReceiveThread(direct_zlg_config.channel_handle, 0, params.verbose);

            // 等待指定时间或无限等待
            if (params.receive_duration > 0) {
                std::cout << "Receiving for " << params.receive_duration << " seconds..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(params.receive_duration));
                StopCANReceiveThread();
            } else {
                std::cout << "Receiving indefinitely... Press Ctrl+C to stop" << std::endl;
                // 无限等待，直到用户按 Ctrl+C
                while (receive_thread_running) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }

            CloseDirectZCAN();
            std::cout << "\n=== Receive Mode Complete ===" << std::endl;
            std::cout << "Total frames received: " << receive_count_.load() << std::endl;
            std::cout << "Motor responses: " << motor_response_count_.load() << std::endl;
            return 0;
        }

        // ==================== 接收 + 使能/失能模式 ====================
        if (params.receive_mode) {
            bool is_enable = params.direct_enable;
            std::string mode_name = is_enable ? "Receive-and-Enable" : "Receive-and-Disable";
            std::string action = is_enable ? "ENABLE" : "DISABLE";

            std::cout << "\n=== " << mode_name << " Mode ===" << std::endl;
            std::cout << "Sending " << action << " command and monitoring motor responses..." << std::endl;

            // ==================== 解析电机ID范围 ====================
            std::vector<int> motor_ids;
            size_t dash_pos = params.motor_id_str.find('-');
            if (dash_pos != std::string::npos) {
                // 范围模式: 1-5
                int start_id = std::atoi(params.motor_id_str.substr(0, dash_pos).c_str());
                int end_id = std::atoi(params.motor_id_str.substr(dash_pos + 1).c_str());

                std::cout << "\n>>> " << action << " motors " << start_id << " - " << end_id << " <<<" << std::endl;

                for (int motor_id = start_id; motor_id <= end_id; motor_id++) {
                    motor_ids.push_back(motor_id);
                }
            } else {
                // 单个电机模式
                motor_ids.push_back(params.direct_motor_id);
            }

            // ==================== 确定需要初始化哪些端口 ====================
            // 根据电机ID确定需要连接的端口
            std::set<int> ports_needed;
            std::map<int, std::vector<int>> port_motors;  // port -> motor_ids

            for (int motor_id : motor_ids) {
                int port_idx = GetPortIndexForMotor(motor_id);
                if (port_idx >= 0) {
                    int port = PORT_CONFIGS[port_idx].port;
                    int channel = PORT_CONFIGS[port_idx].channel;
                    ports_needed.insert(port);
                    port_motors[port].push_back(motor_id);
                }
            }

            std::cout << "Ports to be used: ";
            for (int port : ports_needed) {
                std::cout << port << " ";
            }
            std::cout << std::endl;

            // ==================== 多端口初始化 ====================
            // 如果只需要一个端口，使用单端口模式
            if (ports_needed.size() == 1) {
                int single_port = *ports_needed.begin();
                int single_channel = PORT_CONFIGS[GetPortIndexForMotor(motor_ids[0])].channel;

                std::cout << "Using single-port mode: Port " << single_port << ", CAN" << single_channel << std::endl;

                if (!InitializeDirectZCAN(params.zlg_ip, single_port, single_channel, params.arb_baud, params.data_baud)) {
                    std::cerr << "Failed to initialize ZCAN connection" << std::endl;
                    return 1;
                }

                // 先启动接收线程
                StartCANReceiveThread(direct_zlg_config.channel_handle, 0, params.verbose);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                // 发送使能/失能命令
                for (int motor_id : motor_ids) {
                    if (is_enable) {
                        SendDirectEnableCommand(motor_id);
                    } else {
                        SendDirectDisableCommand(motor_id);
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            } else {
                // 多端口模式：使用MultiPortMotorManager
                std::cout << "Using multi-port mode for " << ports_needed.size() << " ports" << std::endl;

                // 创建临时多端口管理器
                auto temp_multi_manager = std::make_unique<MultiPortMotorManager>(
                    params.zlg_ip, params.arb_baud, params.data_baud);

                if (!temp_multi_manager->Initialize()) {
                    std::cerr << "Failed to initialize multi-port manager" << std::endl;
                    return 1;
                }

                // 为每个端口启动接收线程
                std::vector<std::thread> receive_threads;
                std::map<int, CHANNEL_HANDLE> channel_handles;

                for (size_t i = 0; i < 4; i++) {
                    if (ports_needed.count(PORT_CONFIGS[i].port)) {
                        // 获取该端口的channel handle（需要通过MultiPortMotorManager访问）
                        // 注意：这里需要修改MultiPortMotorManager以暴露channel handles
                        // 为简化，我们使用SendEnableCommand来发送命令

                        for (int motor_id : port_motors[PORT_CONFIGS[i].port]) {
                            if (is_enable) {
                                temp_multi_manager->SendEnableCommand(motor_id);
                            } else {
                                temp_multi_manager->SendDisableCommand(motor_id);
                            }
                            std::cout << "  Sent " << action << " to motor " << motor_id
                                      << " (Port " << PORT_CONFIGS[i].port << ")" << std::endl;
                            std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        }
                    }
                }

                std::cout << ">>> Total " << motor_ids.size() << " motors " << (is_enable ? "enabled" : "disabled") << " <<<\n" << std::endl;

                // 由于多端口模式的接收需要更复杂的处理，这里简化处理
                std::cout << "Note: Multi-port receive monitoring not fully implemented in direct mode." << std::endl;
                std::cout << "Motors " << (is_enable ? "enabled" : "disabled") << " successfully." << std::endl;
                return 0;
            }

            std::cout << ">>> Total " << motor_ids.size() << " motors " << (is_enable ? "enabled" : "disabled") << " <<<\n" << std::endl;

            // 等待指定时间或无限等待
            if (params.receive_duration > 0) {
                std::cout << "Monitoring responses for " << params.receive_duration << " seconds..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(params.receive_duration));
            } else {
                std::cout << "Monitoring indefinitely... Press Ctrl+C to stop" << std::endl;
                while (receive_thread_running) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }

            // 停止接收线程
            StopCANReceiveThread();
            CloseDirectZCAN();
            std::cout << "\n=== Monitoring Complete ===" << std::endl;
            std::cout << "Total frames received: " << receive_count_.load() << std::endl;
            std::cout << "Motor responses: " << motor_response_count_.load() << std::endl;
            std::cout << "Motors remain " << (is_enable ? "ENABLED" : "DISABLED") << std::endl;
            return 0;
        }

        // ==================== 标准直接命令模式 ====================
        std::cout << "\n=== Direct Command Mode ===" << std::endl;

        // 标零命令模式
        if (params.direct_zero) {
            std::cout << "=== Motor Zeroing Mode ===" << std::endl;
            std::cout << "Sending ZERO command (FF FF FF FF FF FF FF FE)..." << std::endl;

            // 解析电机ID范围
            std::vector<int> motor_ids;
            size_t dash_pos = params.motor_id_str.find('-');

            if (dash_pos != std::string::npos) {
                // 范围模式: 1-30
                int start_id = std::atoi(params.motor_id_str.substr(0, dash_pos).c_str());
                int end_id = std::atoi(params.motor_id_str.substr(dash_pos + 1).c_str());
                std::cout << "\n>>> ZERO motors " << start_id << " - " << end_id << " <<<" << std::endl;
                for (int motor_id = start_id; motor_id <= end_id; motor_id++) {
                    motor_ids.push_back(motor_id);
                }
            } else {
                // 单个电机模式
                motor_ids.push_back(params.direct_motor_id);
            }

            // 确定需要初始化哪些端口
            std::set<int> ports_needed;
            std::map<int, std::vector<int>> port_motors;

            for (int motor_id : motor_ids) {
                int port_idx = GetPortIndexForMotor(motor_id);
                if (port_idx >= 0) {
                    int port = PORT_CONFIGS[port_idx].port;
                    ports_needed.insert(port);
                    port_motors[port].push_back(motor_id);
                }
            }

            // 使用多端口模式发送标零命令
            auto temp_multi_manager = std::make_unique<MultiPortMotorManager>(
                params.zlg_ip, params.arb_baud, params.data_baud);

            if (!temp_multi_manager->Initialize()) {
                std::cerr << "Failed to initialize multi-port manager" << std::endl;
                return 1;
            }

            // 为每个端口发送标零命令
            for (size_t i = 0; i < 4; i++) {
                if (ports_needed.count(PORT_CONFIGS[i].port)) {
                    for (int motor_id : port_motors[PORT_CONFIGS[i].port]) {
                        bool success = temp_multi_manager->SendZeroCommand(motor_id);
                        if (success) {
                            std::cout << "  ✓ Sent ZERO to motor " << motor_id
                                      << " (Port " << PORT_CONFIGS[i].port << ")" << std::endl;
                        } else {
                            std::cout << "  ✗ Failed to send ZERO to motor " << motor_id << std::endl;
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    }
                }
            }

            std::cout << "\n>>> Total " << motor_ids.size() << " motors ZEROED <<<" << std::endl;
            return 0;
        }

        // 解析电机ID范围
        std::vector<int> motor_ids;
        std::string motor_str = params.motor_id_str.empty() ? std::to_string(params.direct_motor_id) : params.motor_id_str;
        size_t dash_pos = motor_str.find('-');

        if (dash_pos != std::string::npos) {
            // 范围模式: 1-30
            int start_id = std::atoi(motor_str.substr(0, dash_pos).c_str());
            int end_id = std::atoi(motor_str.substr(dash_pos + 1).c_str());
            for (int motor_id = start_id; motor_id <= end_id; motor_id++) {
                motor_ids.push_back(motor_id);
            }
        } else {
            // 单个电机模式
            motor_ids.push_back(params.direct_motor_id);
        }

        // ==================== 确定需要初始化哪些端口 ====================
        std::set<int> ports_needed;
        std::map<int, std::vector<int>> port_motors;  // port -> motor_ids

        for (int motor_id : motor_ids) {
            int port_idx = GetPortIndexForMotor(motor_id);
            if (port_idx >= 0) {
                int port = PORT_CONFIGS[port_idx].port;
                int channel = PORT_CONFIGS[port_idx].channel;
                ports_needed.insert(port);
                port_motors[port].push_back(motor_id);
            }
        }

        std::cout << "Motors to operate: ";
        for (int motor_id : motor_ids) {
            std::cout << motor_id << " ";
        }
        std::cout << std::endl;
        std::cout << "Ports to be used: ";
        for (int port : ports_needed) {
            std::cout << port << " ";
        }
        std::cout << std::endl;

        // ==================== 多端口初始化 ====================
        // 如果只需要一个端口，使用单端口模式
        if (ports_needed.size() == 1) {
            int single_port = *ports_needed.begin();
            int single_channel = PORT_CONFIGS[GetPortIndexForMotor(motor_ids[0])].channel;

            std::cout << "Using single-port mode: Port " << single_port << ", CAN" << single_channel << std::endl;

            if (!InitializeDirectZCAN(params.zlg_ip, single_port, single_channel, params.arb_baud, params.data_baud)) {
                std::cerr << "Failed to initialize ZCAN connection" << std::endl;
                return 1;
            }

            std::cout << "Sending " << (params.direct_enable ? "ENABLE" : "DISABLE") << " command..." << std::endl;

            // 发送使能/失能命令
            for (int motor_id : motor_ids) {
                if (params.direct_enable) {
                    SendDirectEnableCommand(motor_id);
                } else {
                    SendDirectDisableCommand(motor_id);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }

            CloseDirectZCAN();
        } else {
            // 多端口模式：使用MultiPortMotorManager
            std::cout << "Using multi-port mode for " << ports_needed.size() << " ports" << std::endl;

            // 创建临时多端口管理器
            auto temp_multi_manager = std::make_unique<MultiPortMotorManager>(
                params.zlg_ip, params.arb_baud, params.data_baud);

            if (!temp_multi_manager->Initialize()) {
                std::cerr << "Failed to initialize multi-port manager" << std::endl;
                return 1;
            }

            std::cout << "Sending " << (params.direct_enable ? "ENABLE" : "DISABLE") << " command..." << std::endl;

            // 为每个端口发送使能/失能命令
            for (size_t i = 0; i < 4; i++) {
                if (ports_needed.count(PORT_CONFIGS[i].port)) {
                    for (int motor_id : port_motors[PORT_CONFIGS[i].port]) {
                        if (params.direct_enable) {
                            temp_multi_manager->SendEnableCommand(motor_id);
                        } else {
                            temp_multi_manager->SendDisableCommand(motor_id);
                        }
                        std::cout << "  Sent " << (params.direct_enable ? "ENABLE" : "DISABLE")
                                  << " to motor " << motor_id
                                  << " (Port " << PORT_CONFIGS[i].port << ", CAN" << PORT_CONFIGS[i].channel << ")" << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    }
                }
            }
        }

        std::cout << "\n=== Command Complete ===" << std::endl;
        std::cout << ">>> Total " << motor_ids.size() << " motors " << (params.direct_enable ? "enabled" : "disabled") << " <<<" << std::endl;
        return 0;
    }

    // ROS2 桥接模式
    rclcpp::init(argc, argv);

    try {
        auto bridge = std::make_shared<ROS2_to_TCP_Bridge>(params.verbose, params.motor_cmd_enabled);

        // 初始化多端口管理器 (4-port mode: 8000, 8001, 8002, 8003)
        if (!bridge->InitializeMultiPortManager(params.zlg_ip, params.arb_baud, params.data_baud)) {
            std::cerr << "Failed to initialize multi-port manager" << std::endl;
            rclcpp::shutdown();
            return 1;
        }

        bridge->InitializeROS2();

        // Auto-start mode: bypass /motor_enable requirement
        if (params.auto_start) {
            bridge->SetAutoStart();
        }

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
