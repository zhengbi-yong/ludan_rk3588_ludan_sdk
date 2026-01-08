// cantoudp.cpp
// Receives CAN frames from ZLG device (4 ports) and forwards motor data via UDP
// 30 motors total: global CAN ID 1-30, maps to array index 0-29 (array_index = can_id - 1)
// Wait for first CAN frame, then wait 2ms before starting UDP transmission

#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <signal.h>

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

// Port configuration: TCP port and CAN channel mapping
struct PortConfig {
    int port;         // ZLG device TCP port
    int channel;      // CAN channel number
};

// 4 ports with independent CAN channels
// Each port can receive global CAN ID 1-30 
// Array mapping: CAN ID (1-30) -> array index (0-29), i.e., array_index = can_id - 1
constexpr PortConfig PORT_CONFIGS[NUM_PORTS] = {
    {8000, 0},   // TCP port 8000, channel 0
    {8001, 1},   // TCP port 8001, channel 1
    {8002, 2},   // TCP port 8002, channel 2
    {8003, 3}    // TCP port 8003, channel 3
};

struct CanToUdpConfig {
    // ZLG device configuration
    std::string zlg_ip = "192.168.1.5";
    int arb_baud = 1000000;   // 1M bps
    int data_baud = 5000000;  // 5M bps

    // UDP target configuration
    std::string udp_target_ip = "192.168.1.20";
    int udp_target_port = 8990;
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

    // Update data for a specific motor by array index (0-29, where index = can_id - 1)
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
// Sends motor data array via UDP to target
class UdpForwarder {
public:
    UdpForwarder(const CanToUdpConfig& config) : config_(config) {}

    ~UdpForwarder() {
        if (sockfd_ >= 0) {
            close(sockfd_);
        }
    }

    bool Initialize() {
        // Create UDP socket
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            std::cerr << "Failed to create UDP socket" << std::endl;
            return false;
        }

        // Setup target address
        memset(&target_addr_, 0, sizeof(target_addr_));
        target_addr_.sin_family = AF_INET;
        target_addr_.sin_port = htons(config_.udp_target_port);
        if (inet_pton(AF_INET, config_.udp_target_ip.c_str(), &target_addr_.sin_addr) <= 0) {
            std::cerr << "Invalid UDP target IP: " << config_.udp_target_ip << std::endl;
            return false;
        }

        std::cout << "[UDP] Will forward to " << config_.udp_target_ip << ":"
                  << config_.udp_target_port << " @ 500Hz (after first frame + 2ms delay)" << std::endl;
        return true;
    }

    void SendMotorData(const MotorDataBuffer& buffer) {
        std::lock_guard<std::mutex> lock(buffer.mutex);

        ssize_t sent = sendto(sockfd_, buffer.GetDataPtr(), buffer.GetDataSize(), 0,
                             (struct sockaddr*)&target_addr_, sizeof(target_addr_));

        if (sent < 0) {
            // Silently ignore send errors to avoid spamming
            std::cerr << "[UDP] sendto failed" << std::endl;
            std::cin.get();
        } else {
            send_count_++;
        }
    }

    uint64_t GetSendCount() const { return send_count_; }

private:
    CanToUdpConfig config_;
    int sockfd_ = -1;
    struct sockaddr_in target_addr_;
    std::atomic<uint64_t> send_count_{0};
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

private:
    void ForwardLoop();

    CanToUdpConfig config_;
    MotorDataBuffer data_buffer_;
    UdpForwarder udp_forwarder_;
    std::vector<std::unique_ptr<PortReceiver>> port_receivers_;
    DEVICE_HANDLE device_handles_[NUM_PORTS] = {nullptr, nullptr, nullptr, nullptr};  // 4 independent device handles

    std::thread forward_thread_;
    std::atomic<bool> forward_thread_running_{false};
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

    std::cout << "  [Port " << port_config_.port << "] Ready  " << std::endl;
    return true;
}

void PortReceiver::StartReceiveThread() {
    receive_thread_running_ = true;
    receive_thread_ = std::thread(&PortReceiver::ReceiveLoop, this);
    std::cout << "  [Port " << port_config_.port << "] Receive thread started" << std::endl;
}

void PortReceiver::StopReceiveThread() {
    receive_thread_running_ = false;

    // 显式关闭通道以强制唤醒阻塞在 ZCAN_ReceiveFD 的线程
    if (channel_handle_) {
        ZCAN_ResetCAN(channel_handle_);
        channel_handle_ = nullptr;
    }

    if (receive_thread_.joinable()) {
        // 简单超时：如果 2 秒内没退出就 detach
        std::thread timeout_thread([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            if (receive_thread_.joinable()) {
                std::cerr << "  [Port " << port_config_.port << "] WARNING: Receive thread hang, detaching..." << std::endl;
                receive_thread_.detach();
            }
        });
        timeout_thread.detach();

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

void PortReceiver::ReceiveLoop() {
    ZCAN_ReceiveFD_Data receive_buffer[100];

    std::cout << "[Port " << port_config_.port << "] Receive thread running (Channel "
              << port_config_.channel << ")" << std::endl;

    while (receive_thread_running_) {
        // 检查通道是否有效（可能在停止时被关闭）
        if (channel_handle_ == nullptr || channel_handle_ == INVALID_CHANNEL_HANDLE) {
            break;
        }

        uint32_t received = ZCAN_ReceiveFD(channel_handle_, receive_buffer, 100, 10);

        if (received > 0) {
            for (uint32_t i = 0; i < received; i++) {
                // 打印所有裸 CAN 帧（不做过滤，用于调试）
                uint32_t can_id = receive_buffer[i].frame.can_id & 0x7FF;
                const uint8_t* d = receive_buffer[i].frame.data;
                uint8_t len = receive_buffer[i].frame.len;

                std::cout << "[Port " << port_config_.port << " RAW] CAN ID=0x"
                          << std::hex << std::setw(3) << std::setfill('0') << can_id << std::dec
                          << " Len=" << (int)len << " Data: ";
                for (uint8_t j = 0; j < len; j++) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0')
                              << static_cast<int>(d[j]) << " " << std::dec;
                }
                std::cout << std::endl;

                // Notify manager on first frame (只要收到任何CAN帧就触发，不依赖过滤条件)
                if (manager_ && receive_count_ == 0) {
                    manager_->NotifyFirstFrame();
                }

                // Global CAN ID (1-30) maps directly to array index (0-29)
                int motor_id = can_id & 0x01F;  // Extract global CAN ID (1-30)

                // Check if CAN ID is valid (1-30)
                if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
                    // Map CAN ID to array index: array_index = can_id - 1
                    int array_index = motor_id - 1;
                    data_buffer_.SetMotorData(array_index, d);
                    receive_count_++;
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
        std::cout << pc.port << "(" << ", ch" << pc.channel << ") ";
    }
    std::cout << "\nTotal motors: " << NUM_MOTORS << std::endl;

    // 1. Open 4 independent devices (one for each port)
    std::cout << "[Device] Opening " << NUM_PORTS << " ZLG devices..." << std::endl;
    for (int i = 0; i < NUM_PORTS; i++) {
        device_handles_[i] = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, i, 0);
        if (device_handles_[i] == INVALID_DEVICE_HANDLE) {
            std::cerr << "[Device] ERROR: Failed to open ZCAN device for port " << PORT_CONFIGS[i].port << "!" << std::endl;
            // Close any previously opened devices
            for (int j = 0; j < i; j++) {
                ZCAN_CloseDevice(device_handles_[j]);
                device_handles_[j] = nullptr;
            }
            return false;
        }
        std::cout << "[Device] Device " << i << " opened for port " << PORT_CONFIGS[i].port
                  << ", handle: " << device_handles_[i] << std::endl;
    }

    // Initialize UDP forwarder
    if (!udp_forwarder_.Initialize()) {
        for (int i = 0; i < NUM_PORTS; i++) {
            ZCAN_CloseDevice(device_handles_[i]);
            device_handles_[i] = nullptr;
        }
        return false;
    }

    // 2. Initialize all port receivers with independent device handles
    for (int i = 0; i < NUM_PORTS; i++) {
        auto receiver = std::make_unique<PortReceiver>(
            PORT_CONFIGS[i], config_, data_buffer_, device_handles_[i], this);
        if (!receiver->Initialize()) {
            std::cerr << "[Device] Failed to initialize port " << PORT_CONFIGS[i].port << std::endl;
            // Close all devices
            for (int j = 0; j < NUM_PORTS; j++) {
                ZCAN_CloseDevice(device_handles_[j]);
                device_handles_[j] = nullptr;
            }
            return false;
        }
        port_receivers_.push_back(std::move(receiver));
    }

    std::cout << "=== Initialization Complete ===" << std::endl;
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
    std::cout << ">>> UDP Forward Thread Started (waiting for first frame) <<<" << std::endl;
}

void CanToUdpManager::stop() {
    forward_thread_running_ = false;
    if (forward_thread_.joinable()) {
        forward_thread_.join();
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

void CanToUdpManager::ForwardLoop() {
    constexpr int UDP_INTERVAL_US = 2000;  // 2ms = 500Hz
    using Clock = std::chrono::steady_clock;
    using Microseconds = std::chrono::microseconds;

    // Wait for first CAN frame to arrive before starting UDP transmission
    std::cout << "[UDP] Waiting for first CAN frame..." << std::endl;
    while (forward_thread_running_ && !first_frame_received_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (!forward_thread_running_) {
        return;  // Exit if stopping while waiting
    }

    // First frame received - wait 2ms before starting UDP transmission
    std::cout << "[UDP] First frame received, waiting 2ms before starting UDP transmission..." << std::endl;
    std::this_thread::sleep_for(Microseconds(UDP_INTERVAL_US));
    std::cout << "[UDP] Starting UDP transmission @ 500Hz" << std::endl;

    auto next_send_time = Clock::now();

    while (forward_thread_running_) {
        // Send motor data via UDP (always sends, even if data hasn't changed)
        udp_forwarder_.SendMotorData(data_buffer_);

        // Calculate next send time (precise 2ms interval)
        next_send_time += Microseconds(UDP_INTERVAL_US);

        // Busy-wait until next send time (more precise than sleep)
        auto now = Clock::now();
        if (now < next_send_time) {
            // Spin wait for high precision (for sub-millisecond accuracy)
            auto remaining = std::chrono::duration_cast<Microseconds>(next_send_time - now).count();
            if (remaining > 100) {  // If > 100us, use sleep to reduce CPU
                std::this_thread::sleep_for(Microseconds(remaining - 50));
            }
            // Final spin wait for precision
            while (Clock::now() < next_send_time && forward_thread_running_) {
                // Busy spin (CPU intensive but precise)
            }
        }
    }

    std::cout << ">>> UDP Forward Thread Stopped <<<" << std::endl;
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
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  --zlg-ip <address>     ZLG device IP (default: 192.168.1.5)" << std::endl;
            std::cout << "  --udp-ip <address>     UDP target IP (default: 192.168.1.20)" << std::endl;
            std::cout << "  --udp-port <port>      UDP target port (default: 8990)" << std::endl;
            std::cout << "  -h, --help             Show this help message" << std::endl;
            std::cout << "\nCAN ID Mapping (global CAN ID 1-30):" << std::endl;
            std::cout << "  Motor CAN ID: 1-30 (global)" << std::endl;
            std::cout << "  Array index: 0-29 (i.e., array_index = can_id - 1)" << std::endl;
            std::cout << "\nBehavior:" << std::endl;
            std::cout << "  - Waits for first CAN frame before starting UDP transmission" << std::endl;
            std::cout << "  - After first frame, waits 2ms then starts 500Hz UDP transmission" << std::endl;
            std::cout << "  - CAN receiving continues during the 2ms wait period" << std::endl;
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

    std::cout << "\nPress Ctrl+C/Ctrl+Z to stop..." << std::endl;

    // Main loop - keep running until interrupted
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // Print status every 5 seconds
        // manager.PrintStatus();  // 暂时注释掉，调试 CAN 接收
    }

    std::cout << "Exiting..." << std::endl;
    return 0;
}