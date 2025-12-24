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

// Motor ID mapping function
static inline int get_can_id_for_motor(int motor_id) {
    if (motor_id >= 1 && motor_id <= 30) {
        return motor_id;
    }
    return 0;
}

// ==================== 直接命令模式辅助函数 (使用ZCAN官方封装) ====================
// 用于直接命令模式的简化 ZCAN 操作

static ZhilgongConfig direct_zlg_config;

// 读取并显示 CAN 通道状态
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

// 发送 CAN 帧
static bool SendDirectCANFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc, bool verbose = false) {
    if (!direct_zlg_config.initialized || direct_zlg_config.channel_handle == INVALID_CHANNEL_HANDLE) {
        std::cerr << "ZCAN device not connected" << std::endl;
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

    uint32_t ret = ZCAN_Transmit(direct_zlg_config.channel_handle, &transmit_data, 1);

    if (ret == 1) {
        if (verbose) {
            std::cout << "  [OK] CAN_ID=0x" << std::hex << can_id << std::dec
                      << " Data=";
            for (uint8_t i = 0; i < dlc; i++) {
                std::cout << std::hex << std::setw(2) << std::setfill('0')
                          << static_cast<int>(data[i]) << " ";
            }
            std::cout << std::dec << std::setfill(' ') << std::endl;
        }
        ReadAndDisplayCanStatus(direct_zlg_config.channel_handle);
        return true;
    } else {
        std::cerr << "  [FAIL] CAN_ID=0x" << std::hex << can_id << std::dec
                  << " ret=" << ret << std::endl;
        ReadAndDisplayCanStatus(direct_zlg_config.channel_handle);
        return false;
    }
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

// 关闭直接命令模式的 ZCAN 连接
static void CloseDirectZCAN() {
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

        // 打印原始数据
        std::cout << "  CAN Frame: ID=0x" << std::hex << can_id << std::dec << " Data=";
        for (uint8_t i = 0; i < dlc; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(data[i]) << " ";
        }
        std::cout << std::dec << std::setfill(' ') << std::endl;

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

                    for (int motor_id = 1; motor_id <= 30; motor_id++) {
                        int can_id = get_can_id_for_motor(motor_id);
                        if (can_id > 0) {
                            auto& history = command_history_[motor_id];
                            auto current_time = std::chrono::duration<double>(now.time_since_epoch()).count();

                            MotorCommandCan cmd_to_send;

                            if (history.has_previous) {
                                double prev_time = std::chrono::duration<double>(history.previous_timestamp.time_since_epoch()).count();
                                double curr_time = std::chrono::duration<double>(history.current_timestamp.time_since_epoch()).count();

                                cmd_to_send = interpolateCommand(history.previous, history.current,
                                                               prev_time, curr_time, current_time);
                            } else {
                                cmd_to_send = history.current;
                            }

                            cmd_to_send.motor_id = can_id;
                            SendMotorCommandTCP(cmd_to_send);

                            // 添加短暂延迟，避免周立功设备TCP接收缓冲区溢出
                            // 每个电机命令发送后延迟约50微秒
                            std::this_thread::sleep_for(std::chrono::microseconds(50));
                        }
                    }

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
        bool direct_enable = false;  // true=enable, false=disable
    } params;

    if (argc < 2) {
        std::cout << "Usage: motor_controller_with_enable [zlg_endpoint] [options]" << std::endl;
        std::cout << "\nPositional Arguments:" << std::endl;
        std::cout << "  zlg_endpoint       ZLG device endpoint (e.g., 192.168.1.5:8002, default: 192.168.1.5:8003)" << std::endl;
        std::cout << "                     If omitted, port is determined by -p option" << std::endl;
        std::cout << "\nOptions:" << std::endl;
        std::cout << "  -p, --port <port>       ZLG device TCP port (default: 8003)" << std::endl;
        std::cout << "  -c, --channel <ch>      CAN channel 0-3 (default: 3)" << std::endl;
        std::cout << "  -v, --verbose           Enable verbose logging" << std::endl;
        std::cout << "  --no-quiet              Show ZLG SDK [SYS] debug output (default: HIDDEN)" << std::endl;
        std::cout << "  --enable-motor-cmd      Enable motor command sending (default: DISABLED)" << std::endl;
        std::cout << "\nCAN-FD Baud Rate Configuration:" << std::endl;
        std::cout << "  --arb-baud <rate>       Arbitration baud rate in bps (default: 1000000 = 1M)" << std::endl;
        std::cout << "  --data-baud <rate>      Data baud rate in bps (default: 5000000 = 5M)" << std::endl;
        std::cout << "\nDirect Command Mode (no ROS2 required):" << std::endl;
        std::cout << "  --enable <motor_id>     Send enable command to motor (1-30)" << std::endl;
        std::cout << "  --disable <motor_id>    Send disable command to motor (1-30)" << std::endl;
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
