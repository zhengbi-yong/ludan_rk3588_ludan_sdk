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

static const std::string ROS2_CMD_TOPIC = "/lowcmd";
const int G1_NUM_MOTOR = 30;

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
    uint16_t motor_id;
    float pos;
    float vel;
    float kp;
    float kd;
    float torq;
};

// ==================== MIT Motor Protocol Helper (使用ZCAN官方封装) ====================
// Convert motor command to CAN data bytes (MIT motor protocol format)
// Bytes 0-1: Position (float to int16, ±12.5 rad -> ±32767)
// Bytes 2-3: Velocity (float to int16, ±30 rad/s -> ±32767)
// Bytes 4-5: Kp (float to int16, 0-500 -> 0-32767)
// Bytes 6-7: Kd (float to int16, 0-50 -> 0-32767)
static inline void MotorCommandToCanData(const MotorCommandCan& cmd, uint8_t* can_data) {
    // Convert position to int16 (scale: ±12.5 rad -> ±32767)
    int16_t pos_int = static_cast<int16_t>(cmd.pos * 32767.0f / 12.5f);
    can_data[0] = pos_int & 0xFF;
    can_data[1] = (pos_int >> 8) & 0xFF;

    // Convert velocity to int16 (scale: ±30 rad/s -> ±32767)
    int16_t vel_int = static_cast<int16_t>(cmd.vel * 32767.0f / 30.0f);
    can_data[2] = vel_int & 0xFF;
    can_data[3] = (vel_int >> 8) & 0xFF;

    // Convert Kp to int16 (scale: 0-500 -> 0-32767)
    int16_t kp_int = static_cast<int16_t>(cmd.kp * 32767.0f / 500.0f);
    can_data[4] = kp_int & 0xFF;
    can_data[5] = (kp_int >> 8) & 0xFF;

    // Convert Kd to int16 (scale: 0-50 -> 0-32767)
    int16_t kd_int = static_cast<int16_t>(cmd.kd * 32767.0f / 50.0f);
    can_data[6] = kd_int & 0xFF;
    can_data[7] = (kd_int >> 8) & 0xFF;
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
static inline int get_can_id_for_motor(int motor_id) {
    if (motor_id >= 1 && motor_id <= 30) {
        return motor_id;
    }
    return 0;
}

// ==================== MIT 电机响应解析 ====================
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

                // 如果是电机响应 (ID 1-30)，解析并保存
                if (can_id >= 1 && can_id <= 30) {
                    MotorResponse resp = {0};
                    resp.motor_id = can_id;
                    resp.timestamp = receive_buffer[i].timestamp;

                    if (receive_buffer[i].frame.len >= 8) {
                        // 打印原始数据流（始终显示，不受 verbose 控制）
                        std::cout << "\n  [MOTOR_ID=" << can_id << "] Raw: ";
                        for (uint8_t j = 0; j < receive_buffer[i].frame.len && j < 8; j++) {
                            std::cout << std::hex << std::setw(2) << std::setfill('0') << std::uppercase
                                      << static_cast<int>(receive_buffer[i].frame.data[j]) << " ";
                        }
                        std::cout << std::dec << std::nouppercase << std::endl;

                        // 解析位置 (int16)
                        resp.raw_pos = static_cast<int16_t>(receive_buffer[i].frame.data[0] |
                                                            (receive_buffer[i].frame.data[1] << 8));
                        resp.pos = static_cast<float>(resp.raw_pos) * 12.5f / 32767.0f;

                        // 解析速度 (int16)
                        resp.raw_vel = static_cast<int16_t>(receive_buffer[i].frame.data[2] |
                                                            (receive_buffer[i].frame.data[3] << 8));
                        resp.vel = static_cast<float>(resp.raw_vel) * 30.0f / 32767.0f;

                        // 解析力矩 (int16)
                        resp.raw_torque = static_cast<int16_t>(receive_buffer[i].frame.data[4] |
                                                              (receive_buffer[i].frame.data[5] << 8));
                        resp.torque = static_cast<float>(resp.raw_torque) * 18.0f / 32767.0f;

                        // 解析温度和状态
                        resp.temp = receive_buffer[i].frame.data[6];
                        resp.status = receive_buffer[i].frame.data[7];

                        resp.valid = true;
                        motor_response_count_++;

                        // 更新最新状态
                        {
                            std::lock_guard<std::mutex> lock(motor_states_mutex_);
                            latest_motor_states_[can_id] = resp;
                        }

                        if (verbose) {
                            PrintMotorResponse(resp);
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

// ROS2 桥接类 (继承自 rclcpp::Node)
class ROS2_to_TCP_Bridge : public rclcpp::Node {
private:
    TcpConfig tcp_config_;
    ZhilgongConfig zhilgong_config_;

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

    struct MotorCommandHistory {
        MotorCommandCan current;
        MotorCommandCan previous;
        bool has_previous = false;
        std::chrono::high_resolution_clock::time_point current_timestamp;
        std::chrono::high_resolution_clock::time_point previous_timestamp;
    };
    std::array<MotorCommandHistory, G1_NUM_MOTOR> command_history_;

public:
    ROS2_to_TCP_Bridge(bool verbose = false, bool motor_cmd_enabled = false) : Node("ros2_to_tcp_bridge") {
        verbose_logging_ = verbose;
        motor_cmd_enabled_ = motor_cmd_enabled;

        for (int i = 0; i < G1_NUM_MOTOR; i++) {
            motor_commands_[i] = {static_cast<uint16_t>(i), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        }

        std::cout << "ROS2-to-TCP Bridge Initializing..." << std::endl;
        std::cout << "  Verbose logging: " << (verbose_logging_ ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "  Motor command: " << (motor_cmd_enabled_ ? "ENABLED" : "DISABLED") << std::endl;
    }

    // 设置 ZLG 配置 (从 main 函数调用)
    void SetZlgConfig(const std::string& ip, int port, int channel) {
        zhilgong_config_.remote_ip = ip;
        zhilgong_config_.remote_port = port;
        zhilgong_config_.channel = channel;

        std::cout << "ZLG Config set: IP=" << ip << ", Port=" << port << ", Channel=" << channel << std::endl;
    }

    // 设置 ZLG 配置 (含波特率)
    void SetZlgConfig(const std::string& ip, int port, int channel, int arb_baud, int data_baud) {
        zhilgong_config_.remote_ip = ip;
        zhilgong_config_.remote_port = port;
        zhilgong_config_.channel = channel;
        zhilgong_config_.arbitration_baud = arb_baud;
        zhilgong_config_.data_baud = data_baud;

        std::cout << "ZLG Config set: IP=" << ip << ", Port=" << port << ", Channel=" << channel << std::endl;
        std::cout << "  Arbitration Baud: " << arb_baud << " bps, Data Baud: " << data_baud << " bps" << std::endl;
    }

    ~ROS2_to_TCP_Bridge() {
        should_stop_ = true;
        if (can_send_thread_.joinable()) {
            can_send_thread_.join();
        }

        if (tcp_config_.initialized && tcp_config_.socket_fd >= 0) {
            close(tcp_config_.socket_fd);
        }

        CloseZhilgongConnection();
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

    // Initialize ZCAN connection (使用ZLG官方封装函数)
    bool InitializeZhilgongConnection() {
        if (zhilgong_config_.initialized) {
            return true;
        }

        std::cout << "\n=== Initializing ZCAN Connection ===" << std::endl;
        std::cout << "Target: " << zhilgong_config_.remote_ip << ":" << zhilgong_config_.remote_port << std::endl;
        std::cout << "Channel: CAN" << zhilgong_config_.channel << std::endl;

        // 1. ZCAN_OpenDevice - 打开设备
        zhilgong_config_.device_handle = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, 0, 0);
        if (zhilgong_config_.device_handle == INVALID_DEVICE_HANDLE) {
            std::cerr << "Failed to open ZCAN device" << std::endl;
            return false;
        }
        std::cout << "[1/5] Device opened (ZCAN_OpenDevice)" << std::endl;

        // 2. ZCAN_InitCAN - 初始化CAN通道 (使用CAN-FD模式，显式配置)
        ZCAN_CHANNEL_INIT_CONFIG init_config;
        memset(&init_config, 0, sizeof(init_config));

        // 使用 CAN-FD 模式，显式设置波特率
        init_config.can_type = TYPE_CANFD;                // CAN-FD模式
        init_config.canfd.acc_code = 0;                  // 接受码
        init_config.canfd.acc_mask = 0;                  // 接受掩码
        init_config.canfd.abit_timing = zhilgong_config_.arbitration_baud;  // 仲裁域波特率: 1000000 = 1Mbps
        init_config.canfd.dbit_timing = zhilgong_config_.data_baud;       // 数据域波特率: 5000000 = 5Mbps
        init_config.canfd.brp = 0;                           // 波特率预分频器
        init_config.canfd.filter = 0;                          // 过滤器
        init_config.canfd.mode = 0;                             // 正常模式

        zhilgong_config_.channel_handle = ZCAN_InitCAN(zhilgong_config_.device_handle, zhilgong_config_.channel, &init_config);
        if (zhilgong_config_.channel_handle == INVALID_CHANNEL_HANDLE) {
            std::cerr << "Failed to initialize CAN channel " << zhilgong_config_.channel << std::endl;
            ZCAN_CloseDevice(zhilgong_config_.device_handle);
            return false;
        }
        std::cout << "[2/5] CAN channel initialized (ZCAN_InitCAN)" << std::endl;

        // 3. ZCAN_SetReference - 设置工作模式和连接参数
        UINT val = 0;  // 0 = TCP client mode
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, zhilgong_config_.channel, CMD_TCP_TYPE, &val);

        // 设置目标 IP
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, zhilgong_config_.channel, CMD_DESIP, (void*)zhilgong_config_.remote_ip.c_str());

        // 设置目标端口
        val = zhilgong_config_.remote_port;
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, zhilgong_config_.channel, CMD_DESPORT, &val);
        std::cout << "[3/5] Connection parameters set (ZCAN_SetReference)" << std::endl;

        // 4. ZCAN_StartCAN - 启动CAN通道
        if (ZCAN_StartCAN(zhilgong_config_.channel_handle) == STATUS_ERR) {
            std::cerr << "Failed to start CAN channel" << std::endl;
            ZCAN_CloseDevice(zhilgong_config_.device_handle);
            return false;
        }
        std::cout << "[4/5] CAN channel started (ZCAN_StartCAN)" << std::endl;

        // 5. 短暂延迟等待连接稳定
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "[5/5] Connection established!" << std::endl;
        std::cout << "=====================================" << std::endl << std::endl;

        zhilgong_config_.initialized = true;
        return true;
    }

    void InitializeROS2() {
        std::cout << "Initializing ROS2 subscriber on topic: " << ROS2_CMD_TOPIC << std::endl;

        lowcmd_subscriber_ = this->create_subscription<xixilowcmd::msg::LowCmd>(
            ROS2_CMD_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
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

        // 使用互斥锁保护 motor_commands_ 和 command_history_
        std::lock_guard<std::mutex> lock(motor_commands_mutex_);

        for (const auto& motor_cmd : msg->motor_cmd) {
            int motor_id = motor_cmd.id;

            if (motor_id >= 0 && motor_id < G1_NUM_MOTOR) {
                motor_commands_[motor_id].motor_id = motor_id;
                motor_commands_[motor_id].pos = motor_cmd.q;
                motor_commands_[motor_id].vel = motor_cmd.dq;
                motor_commands_[motor_id].kp = motor_cmd.kp;
                motor_commands_[motor_id].kd = motor_cmd.kd;
                motor_commands_[motor_id].torq = motor_cmd.tau;

                int can_id = get_can_id_for_motor(motor_id);
                if (can_id > 0) {
                    motor_commands_[motor_id].motor_id = can_id;

                    auto& history = command_history_[motor_id];

                    if (history.has_previous) {
                        history.previous = history.current;
                        history.previous_timestamp = history.current_timestamp;
                    } else {
                        history.previous = motor_commands_[motor_id];
                        history.previous_timestamp = current_time;
                        history.has_previous = true;
                    }

                    history.current = motor_commands_[motor_id];
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

        if (motor_id < 0 || motor_id > 29) {
            std::cerr << "Invalid motor ID: " << motor_id << " (range: 0-29)" << std::endl;
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

    // Send enable command using ZLG network packet format
    void SendEnableCommandToZhilgong(int motor_id) {
        if (!InitializeZhilgongConnection()) {
            return;
        }

        int can_id = get_can_id_for_motor(motor_id);
        if (can_id <= 0) {
            std::cerr << "Invalid motor ID: " << motor_id << std::endl;
            return;
        }

        // Enable frame: FF FF FF FF FF FF FF FC
        uint8_t enable_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

        std::cout << "\n>>> Sending ENABLE Command via ZLG Network Packet <<<" << std::endl;
        std::cout << "  Motor ID: " << motor_id << std::endl;
        std::cout << "  CAN ID:   0x" << std::hex << can_id << std::dec << std::endl;
        std::cout << "  Command:  ENABLE (0xFC)" << std::endl;
        std::cout << "  Data:     FF FF FF FF FF FF FF FC" << std::endl;

        SendZlgNetworkPacket(can_id, enable_frame, sizeof(enable_frame));
        std::cout << ">>> Enable command sent <<<" << std::endl;
    }

    // Send disable command using ZLG network packet format
    void SendDisableCommandToZhilgong(int motor_id) {
        if (!InitializeZhilgongConnection()) {
            return;
        }

        int can_id = get_can_id_for_motor(motor_id);
        if (can_id <= 0) {
            std::cerr << "Invalid motor ID: " << motor_id << std::endl;
            return;
        }

        // Disable frame: FF FF FF FF FF FF FF FD
        uint8_t disable_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

        std::cout << "\n>>> Sending DISABLE Command via ZLG Network Packet <<<" << std::endl;
        std::cout << "  Motor ID: " << motor_id << std::endl;
        std::cout << "  CAN ID:   0x" << std::hex << can_id << std::dec << std::endl;
        std::cout << "  Command:  DISABLE (0xFD)" << std::endl;
        std::cout << "  Data:     FF FF FF FF FF FF FF FD" << std::endl;

        SendZlgNetworkPacket(can_id, disable_frame, sizeof(disable_frame));
        std::cout << ">>> Disable command sent <<<" << std::endl;
    }

    // Send custom enable command using ZLG network packet format
    void SendCustomEnableCommandToZhilgong(int motor_id, uint8_t command) {
        if (!InitializeZhilgongConnection()) {
            return;
        }

        int can_id = get_can_id_for_motor(motor_id);
        if (can_id <= 0) {
            std::cerr << "Invalid motor ID: " << motor_id << std::endl;
            return;
        }

        // Custom frame: FF FF FF FF FF FF FF [command]
        uint8_t custom_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command};

        std::cout << "\n>>> Sending CUSTOM Command via ZLG Network Packet <<<" << std::endl;
        std::cout << "  Motor ID: " << motor_id << std::endl;
        std::cout << "  CAN ID:   0x" << std::hex << can_id << std::dec << std::endl;
        std::cout << "  Command:  0x" << std::hex << static_cast<int>(command) << std::dec << std::endl;
        std::cout << "  Data:     FF FF FF FF FF FF FF " << std::hex << static_cast<int>(command) << std::dec << std::endl;

        SendZlgNetworkPacket(can_id, custom_frame, sizeof(custom_frame));
        std::cout << ">>> Custom command sent <<<" << std::endl;
    }

    // Send ZLG CAN frame using ZCAN_Transmit (使用ZLG官方封装函数)
    void SendZlgNetworkPacket(int can_id, const uint8_t* data, size_t data_len) {
        if (!zhilgong_config_.initialized || zhilgong_config_.channel_handle == INVALID_CHANNEL_HANDLE) {
            std::cerr << "ZCAN device not connected" << std::endl;
            return;
        }

        // 构建 ZCAN_Transmit_Data 结构
        ZCAN_Transmit_Data transmit_data;
        memset(&transmit_data, 0, sizeof(transmit_data));

        // 设置 CAN ID (使用 MAKE_CAN_ID 宏)
        transmit_data.frame.can_id = MAKE_CAN_ID(can_id, 0, 0, 0);

        // 设置 DLC
        uint8_t dlc = (data_len > 8) ? 8 : data_len;
        transmit_data.frame.can_dlc = dlc;

        // 设置数据
        for (uint8_t i = 0; i < dlc; i++) {
            transmit_data.frame.data[i] = data[i];
        }

        // 发送类型 (0=正常发送)
        transmit_data.transmit_type = 0;

        // ==================== 打印 TCP 字节流 ====================
        std::cout << "\n  >>> Sending ZLG Network Packet <<<" << std::endl;
        std::cout << "  CAN Frame: ID=0x" << std::hex << can_id << std::dec << " Data=";
        for (uint8_t i = 0; i < dlc; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(data[i]) << " ";
        }
        std::cout << std::dec << std::setfill(' ') << std::endl;

        // 打印完整的 TCP 字节流
        PrintZLGRawPacket(can_id, data, dlc, zhilgong_config_.channel, true);

        // 使用 ZCAN_Transmit 发送
        uint32_t ret = ZCAN_Transmit(zhilgong_config_.channel_handle, &transmit_data, 1);

        if (ret == 1) {
            std::cout << "  [OK] Sent via ZCAN_Transmit" << std::endl;
        } else {
            std::cerr << "  [FAIL] Send failed (ret=" << ret << ")" << std::endl;
        }
    }

    void CloseZhilgongConnection() {
        if (zhilgong_config_.initialized) {
            if (zhilgong_config_.channel_handle != INVALID_CHANNEL_HANDLE) {
                ZCAN_ResetCAN(zhilgong_config_.channel_handle);
                zhilgong_config_.channel_handle = INVALID_CHANNEL_HANDLE;
            }
            if (zhilgong_config_.device_handle != INVALID_DEVICE_HANDLE) {
                ZCAN_CloseDevice(zhilgong_config_.device_handle);
                zhilgong_config_.device_handle = INVALID_DEVICE_HANDLE;
            }
            zhilgong_config_.initialized = false;
            std::cout << "ZCAN connection closed" << std::endl;
        }
    }

    // Send motor command using ZCAN_Transmit (使用ZLG官方封装函数)
    void SendMotorCommandTCP(const MotorCommandCan& cmd) {
        send_count_++;

        if (!zhilgong_config_.initialized || zhilgong_config_.channel_handle == INVALID_CHANNEL_HANDLE) {
            send_error_count_++;
            return;
        }

        // Convert motor command to CAN data bytes (MIT motor protocol)
        // 将电机命令转换为 CAN 数据字节 (MIT 电机协议)
        uint8_t can_data[8];
        MotorCommandToCanData(cmd, can_data);

        // Build ZCAN_Transmit_Data structure
        ZCAN_Transmit_Data transmit_data;
        memset(&transmit_data, 0, sizeof(transmit_data));

        // 设置 CAN ID (使用 MAKE_CAN_ID 宏)
        uint32_t can_id = cmd.motor_id;
        transmit_data.frame.can_id = MAKE_CAN_ID(can_id, 0, 0, 0);
        transmit_data.frame.can_dlc = 8;

        // 设置数据
        for (int i = 0; i < 8; i++) {
            transmit_data.frame.data[i] = can_data[i];
        }

        // 发送类型 (0=正常发送)
        transmit_data.transmit_type = 0;

        // 使用 ZCAN_Transmit 发送
        uint32_t ret = ZCAN_Transmit(zhilgong_config_.channel_handle, &transmit_data, 1);

        if (ret == 1) {
            send_success_count_++;
        } else {
            send_error_count_++;
        }
    }

    // ==================== 批量队列发送功能 ====================
    // 使用 ZLG SDK 的队列批量发送功能，一次性发送多个电机命令
    // 这样可以显著提高发送效率，减少 TCP 通信开销
    int SendMotorCommandBatch(const std::vector<MotorCommandCan>& commands, bool verbose = false) {
        if (!zhilgong_config_.initialized || zhilgong_config_.channel_handle == INVALID_CHANNEL_HANDLE) {
            send_error_count_ += commands.size();
            return 0;
        }

        if (commands.empty()) {
            return 0;
        }

        // 准备批量发送的 CAN 帧数组
        std::vector<ZCAN_Transmit_Data> frames;
        frames.reserve(commands.size());

        for (const auto& cmd : commands) {
            ZCAN_Transmit_Data transmit_data;
            memset(&transmit_data, 0, sizeof(transmit_data));

            // 设置 CAN ID
            uint32_t can_id = cmd.motor_id;
            transmit_data.frame.can_id = MAKE_CAN_ID(can_id, 0, 0, 0);
            transmit_data.frame.can_dlc = 8;

            // 转换电机命令为 CAN 数据
            uint8_t can_data[8];
            MotorCommandToCanData(cmd, can_data);

            for (int i = 0; i < 8; i++) {
                transmit_data.frame.data[i] = can_data[i];
            }

            transmit_data.transmit_type = 0;
            frames.push_back(transmit_data);
        }

        // 打印批量发送信息（完整包结构 + TCP 字节流）
        if (verbose) {
            std::cout << "  >>> [BATCH] Sending " << frames.size() << " Motor Commands <<<" << std::endl;

            // 打印 TCP 字节流（前5个帧的详细包）
            PrintZLGRawPacketBatch(frames.data(), frames.size(), zhilgong_config_.channel);

            // 显示前5个帧的 ZCAN_Transmit_Data 结构信息
            std::cout << "\n  [ZCAN_Structure]" << std::endl;
            uint32_t show_count = std::min(static_cast<uint32_t>(frames.size()), static_cast<uint32_t>(5));
            for (uint32_t i = 0; i < show_count; i++) {
                uint32_t can_id = GET_ID(frames[i].frame.can_id);
                std::cout << "    [Frame " << i << "] ";
                std::cout << "can_id_raw=0x" << std::hex << std::setw(8) << std::setfill('0') << frames[i].frame.can_id << std::dec;
                std::cout << " (ID=0x" << std::hex << std::setw(3) << std::setfill('0') << can_id << std::dec << ") ";
                std::cout << "dlc=" << static_cast<int>(frames[i].frame.can_dlc) << " ";
                std::cout << "data=";
                for (uint8_t j = 0; j < frames[i].frame.can_dlc && j < 8; j++) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(frames[i].frame.data[j]);
                    if (j < frames[i].frame.can_dlc - 1) std::cout << " ";
                }
                std::cout << std::dec << std::endl;
            }
            if (frames.size() > 5) {
                std::cout << "    ... and " << (frames.size() - 5) << " more frames" << std::endl;
            }
        }

        // 使用 ZCAN_Transmit 批量发送（一次发送多个帧）
        // ==================== 验证队列发送：测量发送时间 ====================
        auto start_time = std::chrono::high_resolution_clock::now();
        uint32_t ret = ZCAN_Transmit(zhilgong_config_.channel_handle, frames.data(), frames.size());
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

        if (verbose) {
            std::cout << "    [BATCH] Result: " << ret << "/" << frames.size() << " frames sent" << std::endl;
            std::cout << "    [TIME] Single ZCAN_Transmit call took: " << duration_us << " us" << std::endl;
            std::cout << "    [TIME] Average per frame: " << (duration_us / frames.size()) << " us/frame" << std::endl;
            std::cout << "    [VERIFY] Queue send confirmed: " << frames.size() << " frames in ONE API call" << std::endl;
        }

        // 更新统计
        send_count_ += commands.size();
        if (ret > 0) {
            send_success_count_ += ret;
            send_error_count_ += (commands.size() - ret);
        } else {
            send_error_count_ += commands.size();
        }

        return ret;
    }

    // 收集所有需要发送的电机命令到批量数组
    std::vector<MotorCommandCan> CollectMotorCommands(std::chrono::high_resolution_clock::time_point current_time) {
        std::vector<MotorCommandCan> commands;
        commands.reserve(G1_NUM_MOTOR);

        for (int motor_id = 1; motor_id <= 30; motor_id++) {
            int can_id = get_can_id_for_motor(motor_id);
            if (can_id > 0) {
                auto& history = command_history_[motor_id];

                MotorCommandCan cmd_to_send;
                auto current_timestamp = std::chrono::duration<double>(current_time.time_since_epoch()).count();

                if (history.has_previous) {
                    double prev_time = std::chrono::duration<double>(history.previous_timestamp.time_since_epoch()).count();
                    double curr_time = std::chrono::duration<double>(history.current_timestamp.time_since_epoch()).count();

                    cmd_to_send = interpolateCommand(history.previous, history.current,
                                                   prev_time, curr_time, current_timestamp);
                } else {
                    cmd_to_send = history.current;
                }

                cmd_to_send.motor_id = can_id;
                commands.push_back(cmd_to_send);
            }
        }

        return commands;
    }

    MotorCommandCan interpolateCommand(const MotorCommandCan& prev, const MotorCommandCan& curr,
                                       double prev_time, double curr_time, double target_time) {
        MotorCommandCan result = curr;

        if (curr_time > prev_time) {
            double t = (target_time - prev_time) / (curr_time - prev_time);
            t = std::max(0.0, std::min(1.0, t));

            result.pos = prev.pos + t * (curr.pos - prev.pos);
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

                    // 使用批量发送功能一次性发送所有电机命令
                    // 这样可以显著提高效率，从原来的 30*50us = 1500us 降低到单次发送时间
                    if (!motor_commands.empty()) {
                        SendMotorCommandBatch(motor_commands, verbose_logging_);
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
                              << "ZLG: " << (zhilgong_config_.initialized ? "Connected" : "Disconnected")
                              << std::endl;
                } else if (first_log) {
                    // 第一次开始发送，打印标题
                    std::cout << "\n========== Motor Command Sending Started ==========" << std::endl;
                    std::cout << "[STAT] "
                              << "ROS2_msg: " << current_ros2_count << " (+" << ros2_delta << "/s) | "
                              << "Send: " << current_send_count << " (+" << sends_delta << "/s) | "
                              << "Success: " << current_success_count << " (+" << success_delta << "/s) | "
                              << "Error: " << current_error_count << " (+" << error_delta << "/s) | "
                              << "ZLG: " << (zhilgong_config_.initialized ? "Connected" : "Disconnected")
                              << std::endl;
                    first_log = false;
                } else {
                    // 正常发送状态
                    std::cout << "[STAT] "
                              << "ROS2_msg: " << current_ros2_count << " (+" << ros2_delta << "/s) | "
                              << "Send: " << current_send_count << " (+" << sends_delta << "/s) | "
                              << "Success: " << current_success_count << " (+" << success_delta << "/s) | "
                              << "Error: " << current_error_count << " (+" << error_delta << "/s) | "
                              << "ZLG: " << (zhilgong_config_.initialized ? "Connected" : "Disconnected")
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
        int zlg_port = 8003;
        int channel = 2;  // CAN2 (默认改为CAN2)
        bool verbose = false;
        bool motor_cmd_enabled = false;
        bool quiet = true;  // 默认启用 quiet 模式，屏蔽 ZLG [SYS] 输出

        // CAN-FD 波特率配置
        int arb_baud = 1000000;  // 仲裁域波特率 (默认 1M bps)
        int data_baud = 5000000;  // 数据域波特率 (默认 5M bps)

        // 直接命令模式
        bool direct_mode = false;
        int direct_motor_id = -1;
        std::string motor_id_str;  // 电机 ID 字符串（用于范围模式，如 "1-5"）
        bool direct_enable = false;  // true=enable, false=disable

        // 接收模式
        bool receive_only = false;  // 只接收不发送
        bool receive_mode = false;  // 接收模式 (发送 + 接收)
        int receive_duration = 0;   // 接收持续时间 (秒)，0=无限
    } params;

    if (argc < 2) {
        std::cout << "Usage: motor_controller_with_enable [zlg_endpoint] [options]" << std::endl;
        std::cout << "\nPositional Arguments:" << std::endl;
        std::cout << "  zlg_endpoint       ZLG device endpoint (e.g., 192.168.1.5:8002, default: 192.168.1.5:8003)" << std::endl;
        std::cout << "                     If omitted, port is determined by -p option" << std::endl;
        std::cout << "\nOptions:" << std::endl;
        std::cout << "  -p, --port <port>       ZLG device TCP port (default: 8003)" << std::endl;
        std::cout << "  -c, --channel <ch>      CAN channel 0-3 (default: 2)" << std::endl;
        std::cout << "  -v, --verbose           Enable verbose logging" << std::endl;
        std::cout << "  --no-quiet              Show ZLG SDK [SYS] debug output (default: HIDDEN)" << std::endl;
        std::cout << "  --enable-motor-cmd      Enable motor command sending (default: DISABLED)" << std::endl;
        std::cout << "\nCAN-FD Baud Rate Configuration:" << std::endl;
        std::cout << "  --arb-baud <rate>       Arbitration baud rate in bps (default: 1000000 = 1M)" << std::endl;
        std::cout << "  --data-baud <rate>      Data baud rate in bps (default: 5000000 = 5M)" << std::endl;
        std::cout << "\nDirect Command Mode (no ROS2 required):" << std::endl;
        std::cout << "  --enable <motor_id>     Send enable command to motor (1-30)" << std::endl;
        std::cout << "  --disable <motor_id>    Send disable command to motor (1-30)" << std::endl;
        std::cout << "\nCAN Receive Mode (monitor CAN bus):" << std::endl;
        std::cout << "  --receive-only          Receive CAN frames only (no transmission)" << std::endl;
        std::cout << "  --receive-and-enable    Send enable command and then monitor responses" << std::endl;
        std::cout << "  --receive-duration <s>  Receive duration in seconds (0=infinite, default: 0)" << std::endl;
        std::cout << "\nExamples:" << std::endl;
        std::cout << "  # ROS2 mode - CAN2 with motor commands" << std::endl;
        std::cout << "  ./motor_controller_with_enable 192.168.1.5:8002 -c 2 --enable-motor-cmd" << std::endl;
        std::cout << "\n  # ROS2 mode - CAN3 with verbose logging" << std::endl;
        std::cout << "  ./motor_controller_with_enable 192.168.1.5:8003 -c 3 -v --enable-motor-cmd" << std::endl;
        std::cout << "\n  # Custom baud rate (arbitration 1M, data 5M)" << std::endl;
        std::cout << "  ./motor_controller_with_enable 192.168.1.5:8002 -c 2 --arb-baud 1000000 --data-baud 5000000" << std::endl;
        std::cout << "\n  # Direct enable/disable mode" << std::endl;
        std::cout << "  ./motor_controller_with_enable 192.168.1.5:8002 --enable 1" << std::endl;
        std::cout << "  ./motor_controller_with_enable 192.168.1.5:8002 -c 2 --disable 5" << std::endl;
        std::cout << "\n  # Enable all motors (1-10) on CAN2" << std::endl;
        std::cout << "  ./motor_controller_with_enable 192.168.1.5:8002 -c 2 --enable 1-10" << std::endl;
        std::cout << "\n  # Receive mode - monitor CAN bus (verbose)" << std::endl;
        std::cout << "  ./motor_controller_with_enable 192.168.1.5:8002 -c 2 --receive-only -v" << std::endl;
        std::cout << "\n  # Enable motor 1 and monitor responses for 30 seconds" << std::endl;
        std::cout << "  ./motor_controller_with_enable 192.168.1.5:8002 -c 2 --receive-and-enable 1 --receive-duration 30" << std::endl;
        return 1;
    }

    // 解析第一个位置参数 (ZLG endpoint，格式: IP:PORT 或仅 PORT)
    std::string first_arg = argv[1];
    size_t colon_pos = first_arg.find(':');
    if (colon_pos != std::string::npos) {
        // 格式: IP:PORT
        params.zlg_ip = first_arg.substr(0, colon_pos);
        params.zlg_port = std::stoi(first_arg.substr(colon_pos + 1));
    } else {
        // 尝试解析为端口号
        try {
            params.zlg_port = std::stoi(first_arg);
        } catch (...) {
            // 不是数字，当作 IP 处理（使用默认端口）
            params.zlg_ip = first_arg;
        }
    }

    // 解析可选参数
    for (int i = 2; i < argc; i++) {
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
        else if ((arg == "-p" || arg == "--port") && i + 1 < argc) {
            params.zlg_port = std::atoi(argv[++i]);
        }
        else if ((arg == "-c" || arg == "--channel") && i + 1 < argc) {
            params.channel = std::atoi(argv[++i]);
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
        else if (arg == "--receive-duration" && i + 1 < argc) {
            params.receive_duration = std::atoi(argv[++i]);
        }
    }

    std::cout << "========================================" << std::endl;
    std::cout << "ROS2 Motor Controller with ZLG Enable" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "ZLG Device: " << params.zlg_ip << ":" << params.zlg_port << std::endl;
    std::cout << "CAN Channel: " << params.channel << " (CAN" << params.channel << ")" << std::endl;
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

            // 初始化 ZCAN 连接
            if (!InitializeDirectZCAN(params.zlg_ip, params.zlg_port, params.channel, params.arb_baud, params.data_baud)) {
                std::cerr << "Failed to initialize ZCAN connection" << std::endl;
                return 1;
            }

            // 启动接收线程
            StartCANReceiveThread(direct_zlg_config.channel_handle, params.channel, params.verbose);

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

        // ==================== 接收 + 使能模式 ====================
        if (params.receive_mode) {
            std::cout << "\n=== Receive-and-Enable Mode ===" << std::endl;
            std::cout << "Sending enable command and monitoring motor responses..." << std::endl;

            // 初始化 ZCAN 连接
            if (!InitializeDirectZCAN(params.zlg_ip, params.zlg_port, params.channel, params.arb_baud, params.data_baud)) {
                std::cerr << "Failed to initialize ZCAN connection" << std::endl;
                return 1;
            }

            // 先启动接收线程
            StartCANReceiveThread(direct_zlg_config.channel_handle, params.channel, params.verbose);

            // 短暂延迟等待接收线程就绪
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // ==================== 处理范围模式 ====================
            std::vector<int> motor_ids;
            size_t dash_pos = params.motor_id_str.find('-');
            if (dash_pos != std::string::npos) {
                // 范围模式: 1-5
                int start_id = std::atoi(params.motor_id_str.substr(0, dash_pos).c_str());
                int end_id = std::atoi(params.motor_id_str.substr(dash_pos + 1).c_str());

                std::cout << "\n>>> Enabling motors " << start_id << " - " << end_id << " <<<" << std::endl;

                for (int motor_id = start_id; motor_id <= end_id; motor_id++) {
                    motor_ids.push_back(motor_id);
                    SendDirectEnableCommand(motor_id);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                std::cout << ">>> Total " << motor_ids.size() << " motors enabled <<<\n" << std::endl;
            } else {
                // 单个电机模式
                motor_ids.push_back(params.direct_motor_id);
                SendDirectEnableCommand(params.direct_motor_id);
            }

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

            // 停止接收线程（不自动失能电机）
            StopCANReceiveThread();
            CloseDirectZCAN();
            std::cout << "\n=== Monitoring Complete ===" << std::endl;
            std::cout << "Total frames received: " << receive_count_.load() << std::endl;
            std::cout << "Motor responses: " << motor_response_count_.load() << std::endl;
            std::cout << "Motors remain ENABLED (use --disable to turn off)" << std::endl;
            return 0;
        }

        // ==================== 标准直接命令模式 ====================
        std::cout << "\n=== Direct Command Mode ===" << std::endl;

        // 初始化 ZCAN 连接
        if (!InitializeDirectZCAN(params.zlg_ip, params.zlg_port, params.channel, params.arb_baud, params.data_baud)) {
            std::cerr << "Failed to initialize ZCAN connection" << std::endl;
            return 1;
        }

        // 处理单个或多个电机
        std::string motor_str = argv[argc - 1];  // 最后一个参数是 motor_id
        size_t dash_pos = motor_str.find('-');

        if (dash_pos != std::string::npos) {
            // 范围模式: 1-10
            int start_id = std::atoi(motor_str.substr(0, dash_pos).c_str());
            int end_id = std::atoi(motor_str.substr(dash_pos + 1).c_str());

            std::cout << "Sending " << (params.direct_enable ? "ENABLE" : "DISABLE")
                      << " to motors " << start_id << " - " << end_id << std::endl;

            for (int motor_id = start_id; motor_id <= end_id; motor_id++) {
                if (params.direct_enable) {
                    SendDirectEnableCommand(motor_id);
                } else {
                    SendDirectDisableCommand(motor_id);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        } else {
            // 单个电机模式
            std::cout << "Sending " << (params.direct_enable ? "ENABLE" : "DISABLE")
                      << " to motor " << params.direct_motor_id << std::endl;

            if (params.direct_enable) {
                SendDirectEnableCommand(params.direct_motor_id);
            } else {
                SendDirectDisableCommand(params.direct_motor_id);
            }
        }

        CloseDirectZCAN();
        std::cout << "\n=== Command Complete ===" << std::endl;
        return 0;
    }

    // ROS2 桥接模式
    rclcpp::init(argc, argv);

    try {
        auto bridge = std::make_shared<ROS2_to_TCP_Bridge>(params.verbose, params.motor_cmd_enabled);

        // 设置 ZLG 配置 (含波特率)
        bridge->SetZlgConfig(params.zlg_ip, params.zlg_port, params.channel, params.arb_baud, params.data_baud);

        // 初始化 TCP 连接 (用于上位机通信，如果需要)
        // 注意：如果不需要连接到上位机 TCP 服务器，可以跳过
        // if (!bridge->InitializeTCP(params.tcp_endpoint)) {
        //     std::cerr << "Failed to initialize TCP connection" << std::endl;
        //     rclcpp::shutdown();
        //     return 1;
        // }

        // 初始化周立功设备连接 (用于发送 CAN 命令)
        if (!bridge->InitializeZhilgongConnection()) {
            std::cerr << "Failed to initialize ZLG connection" << std::endl;
            rclcpp::shutdown();
            return 1;
        }

        bridge->InitializeROS2();

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
