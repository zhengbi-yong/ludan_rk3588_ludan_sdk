// motor_heartbeat_monitor.cpp
// Motor Heartbeat Monitor - Sends read commands to 30 motors at 500Hz
//
// Functionality:
// - Sends heartbeat read commands (0x7FF broadcast) to 30 motors every 2ms
// - Motors 1-30 distributed across 4 CAN ports:
//   Port 8000: motors 1-10   (10 motors, local CAN ID 1-10)
//   Port 8001: motors 11-17  (7 motors, local CAN ID 1-7)
//   Port 8002: motors 18-24  (7 motors, local CAN ID 1-7)
//   Port 8003: motors 25-30  (6 motors, local CAN ID 1-6)
//
// Note: This program only sends read commands.
//       The can_motor_feedback_publisher.cpp receives responses and detects timeouts.

#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <signal.h>
#include <atomic>
#include <chrono>
#include <vector>
#include <memory>
#include <iomanip>
#include <sstream>
#include <set>

// ZLG CANFDNET SDK
#include "CANFDNET.h"

// ==================== Configuration ====================
constexpr int NUM_MOTORS = 30;
constexpr int NUM_PORTS = 4;

// Port configuration (matches motor_controller_with_enable.cpp)
struct PortConfig {
    int port;         // TCP port (8000-8003)
    int motor_count;  // Number of motors on this port
    int motor_offset; // Starting motor index (0-based)
    int channel;      // CAN channel number
};

constexpr PortConfig PORT_CONFIGS[NUM_PORTS] = {
    {8000, 10, 0, 0},   // Motors 1-10  -> Local CAN ID 1-10
    {8001, 7, 10, 1},   // Motors 11-17 -> Local CAN ID 1-7
    {8002, 7, 17, 2},   // Motors 18-24 -> Local CAN ID 1-7
    {8003, 6, 24, 3}    // Motors 25-30 -> Local CAN ID 1-6
};

struct HeartbeatConfig {
    std::string zlg_ip = "192.168.1.5";
    int arb_baud = 1000000;   // 1M bps
    int data_baud = 5000000;  // 5M bps
    int send_frequency = 10;  // Hz (10Hz = 100ms interval)
    std::vector<int> enabled_motors;  // Enabled motor IDs (1-30), empty = all
};

// ==================== Heartbeat Read Command Format ====================
// DM Motor Heartbeat Read Command:
// CAN ID: Motor ID (1-30, direct mapping)
// D[0]: motor_id (Motor CAN ID low 8 bits)
// D[1]: 0x00 (CAN ID high 8 bits)
// D[2]: 0xCC (read motor feedback command)
// D[3-7]: 0x00 (padding)

// ==================== Global Variables ====================
std::atomic<bool> g_running(true);

void SignalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", stopping..." << std::endl;
    g_running = false;
}

// ==================== Single Port Heartbeat Sender ====================
class PortHeartbeatSender {
public:
    PortHeartbeatSender(const PortConfig& port_config, const HeartbeatConfig& global_config, const std::vector<bool>& motor_enabled)
        : port_config_(port_config), global_config_(global_config), motor_enabled_(motor_enabled),
          device_handle_(INVALID_DEVICE_HANDLE), channel_handle_(INVALID_CHANNEL_HANDLE),
          initialized_(false), send_count_(0), receive_count_(0) {}

    ~PortHeartbeatSender() {
        stop();
    }

    bool Initialize();
    void SendHeartbeatForAllMotors();
    void CheckReceive();
    void stop();

    uint64_t GetSendCount() const { return send_count_; }
    uint64_t GetReceiveCount() const { return receive_count_; }

private:
    bool SendHeartbeatReadCommand(int local_motor_id);

    PortConfig port_config_;
    HeartbeatConfig global_config_;
    std::vector<bool> motor_enabled_;  // Which motors are enabled (global 1-30)

    DEVICE_HANDLE device_handle_;
    CHANNEL_HANDLE channel_handle_;
    bool initialized_;

    std::atomic<uint64_t> send_count_;
    std::atomic<uint64_t> receive_count_;
};

bool PortHeartbeatSender::Initialize() {
    std::cout << "  [Port " << port_config_.port << "] Initializing Heartbeat Sender..." << std::endl;

    // 1. Open ZLG device (use device_index = channel, like cantoudp.cpp)
    int device_index = port_config_.channel;
    device_handle_ = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, device_index, 0);
    if (device_handle_ == INVALID_DEVICE_HANDLE) {
        std::cerr << "  [Port " << port_config_.port << "] Failed to open ZLG device" << std::endl;
        return false;
    }
    std::cout << "  [Port " << port_config_.port << "] Device opened, device_index=" << device_index << ", handle: " << device_handle_ << std::endl;

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

    channel_handle_ = ZCAN_InitCAN(device_handle_, port_config_.channel, &init_config);
    if (channel_handle_ == INVALID_CHANNEL_HANDLE) {
        std::cerr << "  [Port " << port_config_.port << "] Failed to initialize CAN channel" << std::endl;
        ZCAN_CloseDevice(device_handle_);
        return false;
    }

    std::cout << "  [Port " << port_config_.port << "] CAN channel " << port_config_.channel
              << " initialized, handle: " << channel_handle_ << std::endl;

    // 3. Set IP and port (use device_index = channel)
    ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, device_index, port_config_.channel,
                     CMD_DESIP, (void*)global_config_.zlg_ip.c_str());
    uint32_t port_val = port_config_.port;
    ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, device_index, port_config_.channel,
                     CMD_DESPORT, &port_val);

    std::cout << "  [Port " << port_config_.port << "] Device configured: "
              << global_config_.zlg_ip << ":" << port_config_.port << std::endl;

    // 4. Start CAN
    int start_result = ZCAN_StartCAN(channel_handle_);
    std::cout << "  [Port " << port_config_.port << "] ZCAN_StartCAN returned: " << start_result << std::endl;

    if (start_result != STATUS_OK) {
        std::cerr << "  [Port " << port_config_.port << "] Failed to start CAN channel" << std::endl;
        ZCAN_CloseDevice(device_handle_);
        return false;
    }

    std::cout << "  [Port " << port_config_.port << "] Ready (motors "
              << (port_config_.motor_offset + 1) << "-"
              << (port_config_.motor_offset + port_config_.motor_count) << ")" << std::endl;

    initialized_ = true;
    return true;
}

bool PortHeartbeatSender::SendHeartbeatReadCommand(int motor_id) {
    if (!initialized_) return false;

    // DM Motor Heartbeat Read Command Format:
    // - CAN ID: 0x7FF (broadcast ID, all motors can receive)
    // - D[0]: motor_id (CAN ID low 8 bits) - target motor ID
    // - D[1]: 0x00 (CAN ID high 8 bits)
    // - D[2]: 0xCC (read motor feedback command)
    // - D[3-7]: 0x00 (padding, optional)

    ZCAN_Transmit_Data transmit_data;
    memset(&transmit_data, 0, sizeof(transmit_data));

    transmit_data.frame.can_id = 0x7FF;  // Broadcast ID (all motors receive)
    transmit_data.frame.can_dlc = 8;

    transmit_data.frame.data[0] = static_cast<uint8_t>(motor_id);  // CANID_L (target motor ID)
    transmit_data.frame.data[1] = 0x00;                            // CANID_H
    transmit_data.frame.data[2] = 0xCC;                            // Read command
    transmit_data.frame.data[3] = 0x00;                            // Padding
    transmit_data.frame.data[4] = 0x00;                            // Padding
    transmit_data.frame.data[5] = 0x00;                            // Padding
    transmit_data.frame.data[6] = 0x00;                            // Padding
    transmit_data.frame.data[7] = 0x00;                            // Padding

    transmit_data.transmit_type = 0;  // Normal transmission

    uint32_t ret = ZCAN_Transmit(channel_handle_, &transmit_data, 1);

    if (ret == 1) {
        send_count_++;
        return true;
    }

    return false;
}

void PortHeartbeatSender::SendHeartbeatForAllMotors() {
    if (!initialized_) return;

    // Send heartbeat read command to enabled motors on this port only
    for (int i = 0; i < port_config_.motor_count; i++) {
        int global_motor_id = port_config_.motor_offset + 1 + i;  // Global motor ID (1-30)
        int motor_index = global_motor_id - 1;  // 0-based index (0-29)

        // Check if this motor is enabled
        if (motor_index < motor_enabled_.size() && motor_enabled_[motor_index]) {
            SendHeartbeatReadCommand(global_motor_id);
        }
    }
}

void PortHeartbeatSender::CheckReceive() {
    if (!initialized_) return;

    ZCAN_ReceiveFD_Data receive_buffer[100];
    uint32_t received = ZCAN_ReceiveFD(channel_handle_, receive_buffer, 100, 0);

    if (received > 0) {
        for (uint32_t i = 0; i < received; i++) {
            uint32_t can_id = receive_buffer[i].frame.can_id & 0x7FF;
            const uint8_t* d = receive_buffer[i].frame.data;
            uint8_t len = receive_buffer[i].frame.len;

            // Debug: print ALL received CAN frames (not just motor feedback)
            std::cout << "[Port " << port_config_.port << " RAW] CAN ID=0x"
                      << std::hex << std::setw(3) << std::setfill('0') << can_id << std::dec
                      << " Len=" << (int)len << " Data: ";
            for (uint8_t j = 0; j < len; j++) {
                std::cout << std::hex << std::setw(2) << std::setfill('0')
                          << static_cast<int>(d[j]) << " " << std::dec;
            }
            std::cout << std::endl;

            // Extract motor ID from CAN ID (motor_id = CAN_ID - 0x20)
            int motor_id_from_can = can_id - 0x20;

            // Check if this is a valid motor feedback (1-30)
            if (motor_id_from_can >= 1 && motor_id_from_can <= 30) {
                receive_count_++;
                std::cout << "[Port " << port_config_.port << "] RX Motor "
                          << motor_id_from_can << " (CAN ID=0x"
                          << std::hex << can_id << std::dec << ")" << std::endl;
            }
        }
    }
}

void PortHeartbeatSender::stop() {
    if (channel_handle_ && channel_handle_ != INVALID_CHANNEL_HANDLE) {
        std::cout << "  [Port " << port_config_.port << "] Resetting CAN channel" << std::endl;
        ZCAN_ResetCAN(channel_handle_);
        channel_handle_ = INVALID_CHANNEL_HANDLE;
    }

    if (device_handle_ && device_handle_ != INVALID_DEVICE_HANDLE) {
        std::cout << "  [Port " << port_config_.port << "] Closing device" << std::endl;
        ZCAN_CloseDevice(device_handle_);
        device_handle_ = INVALID_DEVICE_HANDLE;
    }

    initialized_ = false;
    std::cout << "  [Port " << port_config_.port << "] Stopped" << std::endl;
}

// ==================== Multi-Port Heartbeat Manager ====================
class MultiPortHeartbeatManager {
public:
    MultiPortHeartbeatManager(const HeartbeatConfig& config) : config_(config) {}

    ~MultiPortHeartbeatManager() {
        stop();
    }

    bool Initialize();
    void StartHeartbeatThread();
    void stop();

    uint64_t GetTotalSendCount() const {
        uint64_t total = 0;
        for (const auto& sender : port_senders_) {
            total += sender->GetSendCount();
        }
        return total;
    }

    uint64_t GetTotalReceiveCount() const {
        uint64_t total = 0;
        for (const auto& sender : port_senders_) {
            total += sender->GetReceiveCount();
        }
        return total;
    }

private:
    void HeartbeatSendThread();

    HeartbeatConfig config_;
    std::vector<std::unique_ptr<PortHeartbeatSender>> port_senders_;
    std::thread heartbeat_thread_;
    std::atomic<bool> thread_running_{false};
};

bool MultiPortHeartbeatManager::Initialize() {
    std::cout << "\n=== Initializing Multi-Port Heartbeat Manager ===" << std::endl;
    std::cout << "ZLG IP: " << config_.zlg_ip << std::endl;

    // Determine which motors are enabled and which ports they belong to
    std::vector<bool> motor_enabled(NUM_MOTORS, false);  // Track which motors are enabled
    std::vector<int> ports_to_init;

    if (config_.enabled_motors.empty()) {
        // No specific motors specified, enable all motors and all ports
        for (int i = 0; i < NUM_MOTORS; i++) {
            motor_enabled[i] = true;
        }
        for (int i = 0; i < NUM_PORTS; i++) {
            ports_to_init.push_back(i);
        }
        std::cout << "Motors: All (1-30)" << std::endl;
    } else {
        // Use specified motors
        for (int motor_id : config_.enabled_motors) {
            if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
                motor_enabled[motor_id - 1] = true;
            }
        }

        // Find which ports are needed based on enabled motors
        std::set<int> port_set;
        for (int motor_id : config_.enabled_motors) {
            if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
                // Find which port this motor belongs to
                for (int i = 0; i < NUM_PORTS; i++) {
                    if (motor_id >= PORT_CONFIGS[i].motor_offset + 1 &&
                        motor_id <= PORT_CONFIGS[i].motor_offset + PORT_CONFIGS[i].motor_count) {
                        port_set.insert(i);
                        break;
                    }
                }
            }
        }
        ports_to_init.assign(port_set.begin(), port_set.end());

        std::cout << "Motors: ";
        for (int motor_id : config_.enabled_motors) {
            std::cout << motor_id << " ";
        }
        std::cout << std::endl;
    }

    // Print ports being initialized
    std::cout << "Ports: ";
    for (int i : ports_to_init) {
        std::cout << PORT_CONFIGS[i].port << "(ch" << PORT_CONFIGS[i].channel << ") ";
    }
    std::cout << std::endl;

    // Calculate total enabled motors
    int total_motors = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (motor_enabled[i]) total_motors++;
    }
    std::cout << "Total motors: " << total_motors << std::endl;
    std::cout << "Send frequency: " << config_.send_frequency << " Hz ("
              << (1000.0 / config_.send_frequency) << " ms interval)" << std::endl;

    // Create port senders for needed ports only
    std::cout << "\n[Sender] Creating port heartbeat senders..." << std::endl;
    for (int i : ports_to_init) {
        auto sender = std::make_unique<PortHeartbeatSender>(PORT_CONFIGS[i], config_, motor_enabled);
        if (!sender->Initialize()) {
            std::cerr << "[Sender] ERROR: Failed to initialize port " << PORT_CONFIGS[i].port << std::endl;
            return false;
        }
        port_senders_.push_back(std::move(sender));
    }

    std::cout << "[Sender] All " << port_senders_.size() << " port heartbeat senders initialized" << std::endl;
    return true;
}

void MultiPortHeartbeatManager::StartHeartbeatThread() {
    thread_running_ = true;
    heartbeat_thread_ = std::thread(&MultiPortHeartbeatManager::HeartbeatSendThread, this);
    std::cout << "\n[Manager] Heartbeat send thread started" << std::endl;
}

void MultiPortHeartbeatManager::HeartbeatSendThread() {
    const auto interval = std::chrono::milliseconds(1000 / config_.send_frequency);  // 500Hz = 2ms
    const auto log_interval = std::chrono::seconds(1);  // Log every second
    auto next_send_time = std::chrono::high_resolution_clock::now();
    auto next_log_time = next_send_time + log_interval;

    uint64_t last_send_count = 0;
    uint64_t last_receive_count = 0;

    while (thread_running_ && g_running) {
        auto now = std::chrono::high_resolution_clock::now();

        if (now >= next_send_time) {
            // Send heartbeat read commands to all ports
            for (auto& sender : port_senders_) {
                sender->SendHeartbeatForAllMotors();
            }

            next_send_time += interval;

            // Catch up if we fell behind
            if (now > next_send_time + interval) {
                next_send_time = now + interval;
            }
        }

        // Check for received feedback (non-blocking)
        for (auto& sender : port_senders_) {
            sender->CheckReceive();
        }

        // Log statistics every second
        if (now >= next_log_time) {
            uint64_t current_send_count = GetTotalSendCount();
            uint64_t current_receive_count = GetTotalReceiveCount();
            uint64_t sends_delta = current_send_count - last_send_count;
            uint64_t receives_delta = current_receive_count - last_receive_count;

            std::cout << "[STAT] TX: " << current_send_count
                      << " (+" << sends_delta << "/s) | "
                      << "RX: " << current_receive_count
                      << " (+" << receives_delta << "/s)" << std::endl;

            last_send_count = current_send_count;
            last_receive_count = current_receive_count;
            next_log_time = now + log_interval;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    std::cout << "[Manager] Heartbeat send thread stopped" << std::endl;
}

void MultiPortHeartbeatManager::stop() {
    std::cout << "\n[Manager] Stopping..." << std::endl;
    thread_running_ = false;

    if (heartbeat_thread_.joinable()) {
        std::thread timeout_thread([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            if (heartbeat_thread_.joinable()) {
                std::cerr << "[Manager] WARNING: Heartbeat thread hang, detaching..." << std::endl;
                heartbeat_thread_.detach();
            }
        });
        timeout_thread.detach();

        heartbeat_thread_.join();
    }

    port_senders_.clear();
    std::cout << "[Manager] Stopped" << std::endl;
}

// ==================== Main ====================
void PrintUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options]\n";
    std::cout << "Options:\n";
    std::cout << "  --zlg-ip <address>    ZLG device IP (default: 192.168.1.5)\n";
    std::cout << "  --freq <frequency>    Send frequency in Hz (default: 10)\n";
    std::cout << "  --motors <motors>     Enable specific motors (default: all 1-30)\n";
    std::cout << "                        Examples: --motors 1, --motors 1,2,3, --motors 1-5,10,15\n";
    std::cout << "  -h, --help            Show this help message\n";
    std::cout << "\nMotor distribution:\n";
    std::cout << "  Port 8000 (idx=0): motors 1-10   (10 motors)\n";
    std::cout << "  Port 8001 (idx=1): motors 11-17  (7 motors)\n";
    std::cout << "  Port 8002 (idx=2): motors 18-24  (7 motors)\n";
    std::cout << "  Port 8003 (idx=3): motors 25-30  (6 motors)\n";
    std::cout << "\nHeartbeat read command format:\n";
    std::cout << "  CAN ID: 0x7FF (broadcast)\n";
    std::cout << "  Data: [motor_id, 0x00, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00]\n";
    std::cout << "\nExamples:\n";
    std::cout << "  " << program_name << "                 # All motors (1-30), default settings\n";
    std::cout << "  " << program_name << " --motors 1      # Motor 1 only\n";
    std::cout << "  " << program_name << " --motors 1,2,3  # Motors 1, 2, and 3\n";
    std::cout << "  " << program_name << " --motors 18-24  # Motors 18-24 (port 8002)\n";
}

int main(int argc, char** argv) {
    // Setup signal handler
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    // Parse command line arguments
    HeartbeatConfig config;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--zlg-ip" && i + 1 < argc) {
            config.zlg_ip = argv[++i];
        } else if (arg == "--freq" && i + 1 < argc) {
            config.send_frequency = std::atoi(argv[++i]);
        } else if (arg == "--motors" && i + 1 < argc) {
            // Parse motor list (e.g., "1,2,3" or "18-24")
            std::string motors_str = argv[++i];
            std::istringstream ss(motors_str);
            std::string token;
            while (std::getline(ss, token, ',')) {
                // Check if token contains a range (e.g., "18-24")
                size_t dash_pos = token.find('-');
                if (dash_pos != std::string::npos) {
                    // Parse range
                    int start = std::atoi(token.substr(0, dash_pos).c_str());
                    int end = std::atoi(token.substr(dash_pos + 1).c_str());
                    if (start >= 1 && end <= NUM_MOTORS && start <= end) {
                        for (int m = start; m <= end; m++) {
                            config.enabled_motors.push_back(m);
                        }
                    } else {
                        std::cerr << "Invalid motor range: " << token << " (must be 1-30)" << std::endl;
                        PrintUsage(argv[0]);
                        return 1;
                    }
                } else {
                    // Parse single motor
                    int motor_id = std::atoi(token.c_str());
                    if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
                        config.enabled_motors.push_back(motor_id);
                    } else {
                        std::cerr << "Invalid motor ID: " << motor_id << " (must be 1-30)" << std::endl;
                        PrintUsage(argv[0]);
                        return 1;
                    }
                }
            }
        } else if (arg == "-h" || arg == "--help") {
            PrintUsage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            PrintUsage(argv[0]);
            return 1;
        }
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "   Motor Heartbeat Monitor" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "ZLG IP: " << config.zlg_ip << std::endl;
    std::cout << "Frequency: " << config.send_frequency << " Hz" << std::endl;
    if (!config.enabled_motors.empty()) {
        std::cout << "Motors: ";
        for (int m : config.enabled_motors) {
            std::cout << m << " ";
        }
        std::cout << "(specified)" << std::endl;
    } else {
        std::cout << "Motors: All (1-30)" << std::endl;
    }
    std::cout << "========================================\n" << std::endl;

    // Create heartbeat manager
    MultiPortHeartbeatManager manager(config);

    if (!manager.Initialize()) {
        std::cerr << "Failed to initialize heartbeat manager" << std::endl;
        return 1;
    }

    // Start heartbeat thread
    manager.StartHeartbeatThread();

    std::cout << "\nHeartbeat monitor running..." << std::endl;
    std::cout << "Press Ctrl+C to stop\n" << std::endl;

    // Wait for stop signal
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Cleanup
    std::cout << "\nShutting down..." << std::endl;
    manager.stop();

    std::cout << "Heartbeat monitor stopped" << std::endl;
    return 0;
}
