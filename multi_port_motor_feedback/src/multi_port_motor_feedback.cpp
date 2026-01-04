// Receives CAN frames from ZLG device (4 ports) and forwards motor data via UDP
// 30 motors total: 8+8+8+6 across ports 8000-8003
// Modified: Wait for first CAN frame, then wait 2ms before starting UDP transmission

#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <signal.h>
#include <cerrno>

// ZLG CANFDNET SDK
#include "CANFDNET.h"

// UDP socket includes
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// ==================== Configuration ====================
constexpr int NUM_MOTORS = 30;  // Array size: 30 motors (indices 0-29), motor IDs 1-30
constexpr int DATA_BYTES_PER_MOTOR = 6;
constexpr int NUM_PORTS = 4;

// Port configuration: motor counts per port
struct PortConfig {
    int port;         // ZLG device TCP port
    int motor_count;
    int motor_offset; // Starting motor number for this port
    int channel;      // CAN channel number (may differ from port index)
};

// 4 ports with motor distribution
// Each port uses independent CAN channel (0-3)
// CAN ID 1-8 (or 1-6) is LOCAL to each port
constexpr PortConfig PORT_CONFIGS[NUM_PORTS] = {
    {8000, 8, 0, 0},   // Motor 1-8   -> array index 0-7,   channel 0, local CAN ID 1-8
    {8001, 8, 8, 1},   // Motor 9-16  -> array index 8-15,  channel 1, local CAN ID 1-8
    {8002, 8, 16, 2},  // Motor 17-24 -> array index 16-23, channel 2, local CAN ID 1-8
    {8003, 6, 24, 3}   // Motor 25-30 -> array index 24-29, channel 3, local CAN ID 1-6
};

struct CanToUdpConfig {
    // ZLG device configuration
    std::string zlg_ip = "192.168.1.5";
    int arb_baud = 1000000;   // 1M bps
    int data_baud = 5000000;  // 5M bps

    // UDP target configuration
    std::string udp_target_ip = "192.168.1.20";//"192.168.1.20"
    int udp_target_port = 8990;

    // Test mode configuration
    bool test_mode = false;
    int test_interval_ms = 100;
};

// ==================== Motor Data Buffer ====================
// Stores raw 6-byte data for each of the 30 motors
// Data is initialized to 0, and updated when motor feedback arrives
// If a motor hasn't sent new data yet, old data (or 0) is kept
struct MotorDataBuffer {
    std::array<uint8_t, NUM_MOTORS * DATA_BYTES_PER_MOTOR> data;
    mutable std::mutex mutex;  // mutable to allow locking in const methods

    MotorDataBuffer() {
        data.fill(0);  // Initialize all data to 0
    }

    // Update data for a specific motor (0-29)
    void SetMotorData(int motor_id, const uint8_t* raw_data) {
        if (motor_id >= 0 && motor_id < NUM_MOTORS) {
            std::lock_guard<std::mutex> lock(mutex);
            std::memcpy(&data[motor_id * DATA_BYTES_PER_MOTOR], raw_data, DATA_BYTES_PER_MOTOR);
        }
    }

    // Get raw data pointer for UDP sending (caller must hold lock)
    const uint8_t* GetDataPtr() const {
        return data.data();
    }

    // Get data size
    size_t GetDataSize() const {
        return data.size();
    }
};

// ==================== Forward Declarations ====================
class CanToUdpManager;

// ==================== Single Port CAN Receiver ====================
// Handles one ZLG port (e.g., 8000) with its associated motors
// Uses independent device handle from manager
class PortReceiver {
public:
    PortReceiver(const PortConfig& port_config, const CanToUdpConfig& global_config,
                 MotorDataBuffer& buffer, DEVICE_HANDLE device_handle, CanToUdpManager* manager = nullptr)
        : port_config_(port_config), global_config_(global_config),
          data_buffer_(buffer), device_handle_(device_handle), manager_(manager) {}

    ~PortReceiver() {
        stop();
    }

    bool Initialize();
    void StartReceiveThread();
    void StopReceiveThread();
    void stop();
    uint64_t GetReceiveCount() const { return receive_count_; }
    void SendTestFrame(int can_id);  // Send test CAN frame for debugging

private:
    void ReceiveLoop();  // Implemented after CanToUdpManager definition

    PortConfig port_config_;
    CanToUdpConfig global_config_;
    MotorDataBuffer& data_buffer_;
    DEVICE_HANDLE device_handle_;  // Independent device handle from manager

    CHANNEL_HANDLE channel_handle_ = nullptr;
    std::thread receive_thread_;
    std::atomic<bool> receive_thread_running_{false};
    std::atomic<uint64_t> receive_count_{0};
    CanToUdpManager* manager_;  // Pointer to manager for first frame notification
};

// ==================== UDP Forwarder ====================
// 负责将电机数据通过UDP发送到目标地址
// UDP是无连接协议，不需要建立连接，直接发送数据包
class UdpForwarder {
public:
    UdpForwarder(const CanToUdpConfig& config) : config_(config) {}

    ~UdpForwarder() {
        if (sockfd_ >= 0) {
            close(sockfd_);  // 关闭UDP socket
        }
    }

    bool Initialize() {
        // ===== UDP Socket初始化 =====
        // AF_INET: 使用IPv4协议族
        // SOCK_DGRAM: 使用数据报类型（UDP特性，无连接、不可靠）
        // 0: 自动选择协议（这里是UDP）
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            std::cerr << "Failed to create UDP socket" << std::endl;
            return false;
        }

        // ===== 配置目标地址结构体 =====
        memset(&target_addr_, 0, sizeof(target_addr_));
        target_addr_.sin_family = AF_INET;  // IPv4地址族
        target_addr_.sin_port = htons(config_.udp_target_port);  // 目标端口（htons: 主机字节序转网络字节序）
        // inet_pton: 将点分十进制IP字符串转为网络二进制格式
        if (inet_pton(AF_INET, config_.udp_target_ip.c_str(), &target_addr_.sin_addr) <= 0) {
            std::cerr << "Invalid UDP target IP: " << config_.udp_target_ip << std::endl;
            return false;
        }

        std::cout << "[UDP] Will forward to " << config_.udp_target_ip << ":"
                  << config_.udp_target_port << " @ 500Hz (after first frame + 2ms delay)" << std::endl;
        return true;
    }

    void SendMotorData(const MotorDataBuffer& buffer) {
        // 加锁保护数据缓冲区，防止在读取数据时被CAN接收线程修改
        std::lock_guard<std::mutex> lock(buffer.mutex);

        // ===== UDP数据发送核心函数 =====
        // sendto: UDP发送函数，不需要连接，直接向指定地址发送
        // 参数说明：
        //   sockfd_: socket文件描述符
        //   buffer.GetDataPtr(): 要发送的数据指针（30个电机 × 6字节 = 180字节）
        //   buffer.GetDataSize(): 数据大小（180字节）
        //   0: flags参数（0表示无特殊标志）
        //   (struct sockaddr*)&target_addr_: 目标地址结构体
        //   sizeof(target_addr_): 目标地址结构体大小
        ssize_t sent = sendto(sockfd_, buffer.GetDataPtr(), buffer.GetDataSize(), 0,
                             (struct sockaddr*)&target_addr_, sizeof(target_addr_));

        if (sent < 0) {
            // 打印详细错误信息（只在第一次失败时打印，避免刷屏）
            if (!send_error_logged_) {
                std::cerr << "[UDP ERROR] sendto failed: " << strerror(errno)
                          << " (errno=" << errno << ")" << std::endl;
                std::cerr << "[UDP ERROR] Target: " << config_.udp_target_ip << ":"
                          << config_.udp_target_port << std::endl;
                std::cerr << "[UDP ERROR] Socket fd: " << sockfd_ << std::endl;
                std::cerr << "[UDP ERROR] Data size: " << buffer.GetDataSize() << " bytes" << std::endl;
                send_error_logged_ = true;
            }
            send_error_count_++;
        } else if ((size_t)sent != buffer.GetDataSize()) {
            // 部分发送
            if (!partial_send_logged_) {
                std::cerr << "[UDP WARNING] Partial send: " << sent << "/" << buffer.GetDataSize() << " bytes" << std::endl;
                partial_send_logged_ = true;
            }
        } else {
            send_count_++;
            // 成功后重置错误标志
            send_error_logged_ = false;
            // Print first few sends for debugging
            if (send_count_ <= 5) {
                std::cout << "[UDP] Sent packet " << send_count_ << " (" << sent << " bytes) to "
                          << config_.udp_target_ip << ":" << config_.udp_target_port << std::endl;
            }
        }
    }

    uint64_t GetSendCount() const { return send_count_; }

private:
    CanToUdpConfig config_;
    int sockfd_ = -1;
    struct sockaddr_in target_addr_;
    std::atomic<uint64_t> send_count_{0};
    bool send_error_logged_ = false;
    bool partial_send_logged_ = false;
    std::atomic<uint64_t> send_error_count_{0};
};

// ==================== Multi-Port CAN Manager ====================
// Manages all 4 ports and coordinates UDP forwarding
class CanToUdpManager {
public:
    CanToUdpManager(const CanToUdpConfig& config)
        : config_(config), udp_forwarder_(config), first_frame_received_(false) {}

    ~CanToUdpManager() {
        stop();
    }

    // Called by PortReceiver when first CAN frame is received
    void NotifyFirstFrame() {
        if (!first_frame_received_.exchange(true)) {
            std::cout << "[First Frame] Received! Starting 2ms delay before UDP..." << std::endl;
        }
    }

    bool HasFirstFrame() const {
        return first_frame_received_.load();
    }

    bool Initialize();
    void Start();
    void stop();
    void PrintStatus();
    void StartTestThread();
    std::vector<PortReceiver*>& GetPortReceivers() { return port_receivers_raw_; }

private:
    void ForwardLoop();
    void TestLoop();

    CanToUdpConfig config_;
    MotorDataBuffer data_buffer_;
    UdpForwarder udp_forwarder_;
    std::vector<std::unique_ptr<PortReceiver>> port_receivers_;
    std::vector<PortReceiver*> port_receivers_raw_;  // Raw pointers for test access
    DEVICE_HANDLE device_handles_[NUM_PORTS] = {nullptr, nullptr, nullptr, nullptr};  // 4 independent device handles

    std::thread forward_thread_;
    std::thread test_thread_;
    std::atomic<bool> forward_thread_running_{false};
    std::atomic<bool> test_thread_running_{false};
    std::atomic<bool> first_frame_received_;  // True when first CAN frame received
};

// ==================== PortReceiver Method Implementations ====================

bool PortReceiver::Initialize() {
    std::cout << "  [Port " << port_config_.port << "] Initializing..." << std::endl;

    // 1. Check device handle
    if (device_handle_ == nullptr || device_handle_ == INVALID_DEVICE_HANDLE) {
        std::cerr << "  [Port " << port_config_.port << "] ERROR: Invalid device handle!" << std::endl;
        return false;
    }
    std::cout << "  [Port " << port_config_.port << "] Using shared device handle: " << device_handle_ << std::endl;

    // 2. Initialize CAN channel
    ZCAN_CHANNEL_INIT_CONFIG init_config;
    memset(&init_config, 0, sizeof(init_config));
    init_config.can_type = TYPE_CANFD;
    init_config.canfd.acc_code = 0;
    init_config.canfd.acc_mask = 0;
    init_config.canfd.abit_timing = global_config_.arb_baud;
    init_config.canfd.dbit_timing = global_config_.data_baud;
    init_config.canfd.brp = 0;
    init_config.canfd.filter = 0;
    init_config.canfd.mode = 0;

    std::cout << "  [Port " << port_config_.port << "] Initializing CAN channel " << port_config_.channel << std::endl;

    channel_handle_ = ZCAN_InitCAN(device_handle_, port_config_.channel, &init_config);
    if (channel_handle_ == INVALID_CHANNEL_HANDLE) {
        std::cerr << "  [Port " << port_config_.port << "] Failed to initialize CAN channel " << port_config_.channel << std::endl;
        return false;
    }

    std::cout << "  [Port " << port_config_.port << "] CAN channel " << port_config_.channel
              << " initialized, handle: " << channel_handle_ << std::endl;

    // 3. Set IP and port for each device independently
    // Each device has its own connection: same IP, different port (8000-8003)
    // Device index matches channel number: device 0 uses channel 0, device 1 uses channel 1, etc.
    int device_index = port_config_.channel;
    std::cout << "  [Port " << port_config_.port << "] Setting device " << device_index << " IP and port" << std::endl;
    ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, device_index, device_index, CMD_DESIP,
                     (void*)global_config_.zlg_ip.c_str());
    // Use the port number from config for device connection
    uint32_t port_val = port_config_.port;
    ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, device_index, device_index, CMD_DESPORT, &port_val);

    std::cout << "  [Port " << port_config_.port << "] Device IP and port configured: "
              << global_config_.zlg_ip << ":" << port_config_.port << std::endl;

    // 4. Start CAN
    int start_result = ZCAN_StartCAN(channel_handle_);
    std::cout << "  [Port " << port_config_.port << "] ZCAN_StartCAN returned: " << start_result << std::endl;

    if (start_result != STATUS_OK) {
        std::cerr << "  [Port " << port_config_.port << "] Failed to start CAN channel" << std::endl;
        return false;
    }

    std::cout << "  [Port " << port_config_.port << "] Ready (motors " << (port_config_.motor_offset + 1)
              << "-" << (port_config_.motor_offset + port_config_.motor_count)
              << ", local CAN ID: 1-" << port_config_.motor_count << ")" << std::endl;
    return true;
}

void PortReceiver::StartReceiveThread() {
    receive_thread_running_ = true;
    receive_thread_ = std::thread(&PortReceiver::ReceiveLoop, this);
    std::cout << "  [Port " << port_config_.port << "] Receive thread started" << std::endl;
}

void PortReceiver::StopReceiveThread() {
    receive_thread_running_ = false;
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
}

void PortReceiver::stop() {
    StopReceiveThread();
    if (channel_handle_) {
        std::cout << "  [Port " << port_config_.port << "] Resetting CAN channel" << std::endl;
        ZCAN_ResetCAN(channel_handle_);
    }
    // Note: device is managed by CanToUdpManager, not closed here
    std::cout << "  [Port " << port_config_.port << "] Port receiver stopped" << std::endl;
}

void PortReceiver::SendTestFrame(int can_id) {
    if (!channel_handle_) {
        std::cerr << "  [Port " << port_config_.port << "] No channel handle!" << std::endl;
        return;
    }

    ZCAN_TransmitFD_Data test_frame;
    memset(&test_frame, 0, sizeof(test_frame));
    test_frame.frame.can_id = can_id;
    test_frame.frame.len = 8;
    for (int i = 0; i < 8; i++) {
        test_frame.frame.data[i] = i + 1;
    }

    uint32_t result = ZCAN_TransmitFD(channel_handle_, &test_frame, 1);
    if (result == 1) {
        std::cout << "  [Port " << port_config_.port << "] Test frame sent (CAN ID=" << can_id << ")" << std::endl;
    } else {
        std::cerr << "  [Port " << port_config_.port << "] Failed to send test frame" << std::endl;
    }
}

void PortReceiver::ReceiveLoop() {
    ZCAN_ReceiveFD_Data receive_buffer[100];

    std::cout << "[Port " << port_config_.port << "] Receive thread running (Channel "
              << port_config_.channel << ")" << std::endl;

    uint64_t receive_call_count = 0;
    auto last_debug_time = std::chrono::steady_clock::now();

    while (receive_thread_running_) {
        uint32_t received = ZCAN_ReceiveFD(channel_handle_, receive_buffer, 100, 10);
        receive_call_count++;

        // Print receive status every 5 seconds
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_debug_time).count() > 5000) {
            std::cout << "[Port " << port_config_.port << "] Receive calls: " << receive_call_count
                      << ", Total received: " << receive_count_ << std::endl;
            last_debug_time = now;
        }

        if (received > 0) {
            for (uint32_t i = 0; i < received; i++) {
                // 打印所有裸 CAN 帧（不做过滤，用于调试）
                uint32_t can_id = receive_buffer[i].frame.can_id & 0x7FF;
                const uint8_t* d = receive_buffer[i].frame.data;
                uint8_t len = receive_buffer[i].frame.len;

                std::cout << "[Port " << port_config_.port << " RAW] CAN ID=0x"
                          << std::hex << std::setw(3) << std::setfill('0') << can_id << std::dec
                          << " Len=" << (int)len << " Data: ";
                for (uint8_t j = 0; j < len && j < 8; j++) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0')
                              << static_cast<int>(d[j]) << " " << std::dec;
                }
                std::cout << std::endl;

                // CAN ID mapping: received_id = 0x010 + motor_id
                // Motor ID 0x000 -> CAN ID 0x010, Motor ID 0x004 -> 0x014, ..., Motor ID 0x00F -> 0x01F
                // Calculate motor ID (0-based) from received CAN ID
                int motor_id = can_id - 0x010;

                // Check if CAN ID is in valid range (0x010-0x01F for motor ID 0x000-0x00F)
                // and frame has enough data (at least 6 bytes)
                if (can_id >= 0x010 && can_id <= 0x01F && receive_buffer[i].frame.len >= 6) {
                    // Motor ID is already 0-based (0x000-0x00F), use directly as array index
                    int array_index = motor_id;  // Motor ID 0x000 -> index 0, Motor ID 0x00F -> index 15

                    // Check array index bounds
                    if (array_index >= 0 && array_index < NUM_MOTORS) {
                        // Notify manager on first frame
                        if (manager_ && receive_count_ == 0) {
                            manager_->NotifyFirstFrame();
                        }

                        // Extract first 6 bytes (matching DATA_BYTES_PER_MOTOR)
                        data_buffer_.SetMotorData(array_index, d);

                        receive_count_++;
                    }
                }
            }
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    std::cout << "[Port " << port_config_.port << "] Receive thread stopped" << std::endl;
}

// ==================== CanToUdpManager Method Implementations ====================

bool CanToUdpManager::Initialize() {
    std::cout << "=== Initializing CAN-to-UDP Manager ===" << std::endl;
    std::cout << "ZLG IP: " << config_.zlg_ip << std::endl;
    std::cout << "Ports: ";
    for (const auto& pc : PORT_CONFIGS) {
        std::cout << pc.port << "(" << pc.motor_count << " motors, ch" << pc.channel << ") ";
    }
    std::cout << "\nTotal motors: " << NUM_MOTORS << std::endl;

    // 1. Open devices - skip failed ones instead of aborting
    std::cout << "[Device] Opening up to " << NUM_PORTS << " ZLG devices..." << std::endl;
    for (int i = 0; i < NUM_PORTS; i++) {
        device_handles_[i] = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, i, 0);
        if (device_handles_[i] == INVALID_DEVICE_HANDLE) {
            std::cerr << "[Device] WARNING: Device " << i << " (port " << PORT_CONFIGS[i].port
                      << ") not available - will skip" << std::endl;
            device_handles_[i] = nullptr;
        } else {
            std::cout << "[Device] Device " << i << " opened for port " << PORT_CONFIGS[i].port
                      << ", handle: " << device_handles_[i] << std::endl;
        }
    }

    // Check if at least one device opened
    bool has_any_device = false;
    for (int i = 0; i < NUM_PORTS; i++) {
        if (device_handles_[i] != nullptr) {
            has_any_device = true;
            break;
        }
    }
    if (!has_any_device) {
        std::cerr << "[Device] ERROR: No ZLG devices available!" << std::endl;
        return false;
    }

    // Initialize UDP forwarder
    if (!udp_forwarder_.Initialize()) {
        for (int i = 0; i < NUM_PORTS; i++) {
            if (device_handles_[i]) {
                ZCAN_CloseDevice(device_handles_[i]);
                device_handles_[i] = nullptr;
            }
        }
        return false;
    }

    // 2. Initialize all port receivers - skip failed ones
    int initialized_count = 0;
    for (int i = 0; i < NUM_PORTS; i++) {
        if (device_handles_[i] == nullptr) {
            std::cout << "[Port " << PORT_CONFIGS[i].port << "] Skipping (device not available)" << std::endl;
            continue;
        }

        auto receiver = std::make_unique<PortReceiver>(
            PORT_CONFIGS[i], config_, data_buffer_, device_handles_[i], this);
        if (!receiver->Initialize()) {
            std::cerr << "[Port " << PORT_CONFIGS[i].port << "] WARNING: Initialization failed - skipping" << std::endl;
            // Close this device handle
            ZCAN_CloseDevice(device_handles_[i]);
            device_handles_[i] = nullptr;
            continue;
        }
        port_receivers_raw_.push_back(receiver.get());
        port_receivers_.push_back(std::move(receiver));
        initialized_count++;
    }

    if (initialized_count == 0) {
        std::cerr << "[Device] ERROR: No ports initialized successfully!" << std::endl;
        return false;
    }

    std::cout << "=== Initialization Complete (" << initialized_count << "/" << NUM_PORTS
              << " ports active) ===" << std::endl;
    return true;
}

void CanToUdpManager::Start() {
    // Start all receive threads
    for (auto& receiver : port_receivers_) {
        receiver->StartReceiveThread();
    }
    std::cout << ">>> All CAN Receive Threads Started <<<" << std::endl;

    // Start UDP forwarding thread
    forward_thread_running_ = true;
    forward_thread_ = std::thread(&CanToUdpManager::ForwardLoop, this);
    std::cout << ">>> UDP Forward Thread Started (waiting for first CAN frame) <<<" << std::endl;

    // Note: UDP forwarding will start automatically when first CAN frame arrives
    // For testing without real motors, you can manually populate the data buffer
}

void CanToUdpManager::stop() {
    forward_thread_running_ = false;
    test_thread_running_ = false;
    if (forward_thread_.joinable()) {
        forward_thread_.join();
    }
    if (test_thread_.joinable()) {
        test_thread_.join();
    }

    for (auto& receiver : port_receivers_) {
        receiver->stop();
    }

    // Close all 4 devices
    for (int i = 0; i < NUM_PORTS; i++) {
        if (device_handles_[i] && device_handles_[i] != INVALID_DEVICE_HANDLE) {
            std::cout << "[Device] Closing device " << i << "..." << std::endl;
            ZCAN_CloseDevice(device_handles_[i]);
            device_handles_[i] = nullptr;
        }
    }

    std::cout << ">>> All threads stopped <<<" << std::endl;
}

void CanToUdpManager::PrintStatus() {
    std::cout << "\n=== Status ===" << std::endl;
    uint64_t total_rx = 0;
    for (size_t i = 0; i < port_receivers_.size(); i++) {
        uint64_t count = port_receivers_[i]->GetReceiveCount();
        total_rx += count;
        std::cout << "  Port " << PORT_CONFIGS[i].port << ": " << count << " frames" << std::endl;
    }
    std::cout << "UDP sent: " << udp_forwarder_.GetSendCount() << " packets" << std::endl;
    std::cout << "=============" << std::endl;
}

// ===== UDP转发线程的主循环函数 =====
// 这是UDP发送的核心逻辑，运行在独立线程中
void CanToUdpManager::ForwardLoop() {
    constexpr int UDP_INTERVAL_US = 2000;  // UDP发送间隔：2000微秒 = 2ms = 500Hz频率
    using Clock = std::chrono::steady_clock;
    using Microseconds = std::chrono::microseconds;

    // ===== 阶段1：等待首帧CAN数据 =====
    // 这样设计是为了确保在收到有效电机数据后才开始发送UDP
    std::cout << "[UDP] Waiting for first CAN frame..." << std::endl;
    while (forward_thread_running_ && !first_frame_received_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (!forward_thread_running_) {
        return;  // 如果线程被标记为停止，则退出
    }

    // ===== 阶段2：首帧到达后再等待2ms =====
    // 这个额外的2ms延迟是为了让CAN接收线程有足够时间接收更多的初始数据
    std::cout << "[UDP] First frame received, waiting 2ms before starting UDP transmission..." << std::endl;
    std::this_thread::sleep_for(Microseconds(UDP_INTERVAL_US));
    std::cout << "[UDP] Starting UDP transmission @ 500Hz" << std::endl;

    // ===== 阶段3：开始固定频率的UDP发送循环 =====
    auto next_send_time = Clock::now();  // 初始化第一次发送时间点

    while (forward_thread_running_) {
        // 1. 发送UDP数据包（包含所有30个电机的状态数据）
        //    无论数据是否变化，都按固定频率发送
        udp_forwarder_.SendMotorData(data_buffer_);

        // 2. 计算下一次发送的时间点（当前时间点 + 2ms）
        //    使用累加方式而非直接取当前时间，可以避免时间漂移
        next_send_time += Microseconds(UDP_INTERVAL_US);

        // 3. 精确等待到下一次发送时间点
        //    使用混合策略：sleep + 忙等待，实现精确的2ms间隔
        auto now = Clock::now();
        if (now < next_send_time) {
            // 计算剩余时间（微秒）
            auto remaining = std::chrono::duration_cast<Microseconds>(next_send_time - now).count();
            if (remaining > 100) {  // 如果剩余时间 > 100微秒，先sleep降低CPU占用
                std::this_thread::sleep_for(Microseconds(remaining - 50));
            }
            // 最后阶段使用忙等待（busy spin）确保微秒级精度
            // 虽然增加CPU占用，但能保证2ms间隔的精确性
            while (Clock::now() < next_send_time && forward_thread_running_) {
                // 忙等待循环（CPU密集但精确）
            }
        }
    }

    std::cout << ">>> UDP Forward Thread Stopped <<<" << std::endl;
}

void CanToUdpManager::TestLoop() {
    std::cout << "[TEST] Test thread started, injecting test data..." << std::endl;

    uint32_t test_counter = 0;
    uint8_t test_pattern = 0;

    while (test_thread_running_) {
        // Directly inject test data into buffer (bypass CAN receive)
        for (int motor = 0; motor < NUM_MOTORS; motor++) {
            uint8_t test_data[DATA_BYTES_PER_MOTOR];
            for (int i = 0; i < DATA_BYTES_PER_MOTOR; i++) {
                test_data[i] = test_pattern + motor + i;
            }
            data_buffer_.SetMotorData(motor, test_data);
        }

        // Trigger first frame notification on first injection
        if (test_counter == 0) {
            NotifyFirstFrame();
        }

        test_pattern++;
        test_counter++;

        if (test_counter % 50 == 0) {
            std::cout << "[TEST] Injected " << test_counter << " data cycles (pattern=0x"
                      << std::hex << (int)test_pattern << std::dec << ")" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(config_.test_interval_ms));
    }

    std::cout << ">>> Test Thread Stopped <<<" << std::endl;
}

void CanToUdpManager::StartTestThread() {
    test_thread_running_ = true;
    test_thread_ = std::thread(&CanToUdpManager::TestLoop, this);
    std::cout << ">>> Test Thread Started <<<" << std::endl;
}

// ==================== Global Signal Handler ====================
CanToUdpManager* g_manager = nullptr;

void SignalHandler(int signal) {
    if (g_manager) {
        std::cout << "\nReceived signal " << signal << ", stopping..." << std::endl;
        g_manager->stop();
    }
}

// ==================== Main ====================
int main(int argc, char** argv) {
    // Setup signal handler
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    // Create configuration
    CanToUdpConfig config;

    // Parse command line arguments (optional)
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "--zlg-ip" && i + 1 < argc) {
            config.zlg_ip = argv[++i];
        } else if (arg == "--udp-ip" && i + 1 < argc) {
            config.udp_target_ip = argv[++i];
        } else if (arg == "--udp-port" && i + 1 < argc) {
            config.udp_target_port = std::atoi(argv[++i]);
        } else if (arg == "--test") {
            config.test_mode = true;
            std::cout << "[TEST MODE ENABLED] Will send test CAN frames" << std::endl;
        } else if (arg == "--test-interval" && i + 1 < argc) {
            config.test_interval_ms = std::atoi(argv[++i]);
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  --zlg-ip <address>        ZLG device IP (default: 192.168.1.5)" << std::endl;
            std::cout << "  --udp-ip <address>        UDP target IP (default: 192.168.1.20)" << std::endl;
            std::cout << "  --udp-port <port>         UDP target port (default: 8990)" << std::endl;
            std::cout << "  --test                    Enable test mode (send test CAN frames)" << std::endl;
            std::cout << "  --test-interval <ms>      Test frame interval in ms (default: 100)" << std::endl;
            std::cout << "  -h, --help                Show this help message" << std::endl;
            std::cout << "\nCAN ID Mapping (each port has independent CAN ID range):" << std::endl;
            std::cout << "  Port 8000: motors 1-8   (local CAN ID: 1-8)" << std::endl;
            std::cout << "  Port 8001: motors 9-16  (local CAN ID: 1-8)" << std::endl;
            std::cout << "  Port 8002: motors 17-24 (local CAN ID: 1-8)" << std::endl;
            std::cout << "  Port 8003: motors 25-30 (local CAN ID: 1-6)" << std::endl;
            std::cout << "  Note: Each port uses CAN ID 1-8 (or 1-6 for port 8003) independently" << std::endl;
            std::cout << "\nBehavior:" << std::endl;
            std::cout << "  - Waits for first CAN frame before starting UDP transmission" << std::endl;
            std::cout << "  - After first frame, waits 2ms then starts 500Hz UDP transmission" << std::endl;
            std::cout << "  - CAN receiving continues during the 2ms wait period" << std::endl;
            std::cout << "\nTest Mode:" << std::endl;
            std::cout << "  - Directly injects test data into motor buffer (bypasses CAN)" << std::endl;
            std::cout << "  - Data pattern changes each cycle: motor N has bytes (pattern+N, pattern+N+1, ...)" << std::endl;
            std::cout << "  - Use with --udp-ip set to localhost for local testing" << std::endl;
            return 0;
        }
    }

    // Create and run manager
    CanToUdpManager manager(config);
    g_manager = &manager;

    if (!manager.Initialize()) {
        std::cerr << "Failed to initialize CAN-to-UDP manager" << std::endl;
        return 1;
    }

    // Start all threads
    manager.Start();

    // Start test thread if test mode is enabled
    if (config.test_mode) {
        manager.StartTestThread();
    }

    std::cout << "\nPress Ctrl+C to stop..." << std::endl;

    // Main loop - keep running until interrupted
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // Print status every 5 seconds
        // manager.PrintStatus();  // 暂时注释掉，调试 CAN 接收
    }

    std::cout << "Exiting..." << std::endl;
    return 0;
}