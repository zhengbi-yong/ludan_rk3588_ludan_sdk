// Multi-port CAN frame sender for testing ZLG CANFDNET device
// Sends test CAN frames to 4 ports (8000-8003) simultaneously
// Matches multi_port_motor_feedback.cpp port configuration

#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>
#include <iomanip>
#include <vector>
#include <atomic>
#include <signal.h>

#include "CANFDNET.h"

// Port configuration matching multi_port_motor_feedback.cpp
struct PortConfig {
    int device_index;  // Device index (0-3)
    int channel;       // CAN channel (0-3)
    int port;          // TCP port (8000-8003)
    int motor_count;   // Number of motors (8,8,8,6)
    int motor_offset;  // Starting motor number (0,8,16,24)
};

constexpr int NUM_PORTS = 4;
constexpr PortConfig PORT_CONFIGS[NUM_PORTS] = {
    {0, 0, 8000, 8, 0},   // Motor 1-8   -> array index 0-7
    {1, 1, 8001, 8, 8},   // Motor 9-16  -> array index 8-15
    {2, 2, 8002, 8, 16},  // Motor 17-24 -> array index 16-23
    {3, 3, 8003, 6, 24}   // Motor 25-30 -> array index 24-29
};

// Global flag for signal handler
std::atomic<bool> g_running(true);

void SignalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", stopping..." << std::endl;
    g_running = false;
}

// Single port sender class
class PortSender {
public:
    PortSender(const PortConfig& config, const char* zlg_ip,
               int interval_ms, bool verbose)
        : config_(config), zlg_ip_(zlg_ip),
          interval_ms_(interval_ms), verbose_(verbose) {}

    ~PortSender() {
        stop();
    }

    bool Initialize() {
        std::cout << "[Port " << config_.port << "] Opening device..." << std::endl;

        // Open device
        device_handle_ = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, config_.device_index, 0);
        if (device_handle_ == INVALID_DEVICE_HANDLE) {
            std::cerr << "[Port " << config_.port << "] Failed to open device!" << std::endl;
            return false;
        }

        // Set IP and port
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, config_.device_index,
                         config_.device_index, CMD_DESIP, (void*)zlg_ip_);
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, config_.device_index,
                         config_.device_index, CMD_DESPORT, &config_.port);

        // Initialize CAN channel
        ZCAN_CHANNEL_INIT_CONFIG init_config;
        memset(&init_config, 0, sizeof(init_config));
        init_config.can_type = TYPE_CANFD;
        init_config.canfd.acc_code = 0;
        init_config.canfd.acc_mask = 0;
        init_config.canfd.abit_timing = 1000000;  // 1M
        init_config.canfd.dbit_timing = 5000000;  // 5M
        init_config.canfd.brp = 0;
        init_config.canfd.filter = 0;
        init_config.canfd.mode = 0;

        channel_handle_ = ZCAN_InitCAN(device_handle_, config_.channel, &init_config);
        if (channel_handle_ == INVALID_CHANNEL_HANDLE) {
            std::cerr << "[Port " << config_.port << "] Failed to initialize channel!" << std::endl;
            ZCAN_CloseDevice(device_handle_);
            return false;
        }

        // Start CAN
        if (ZCAN_StartCAN(channel_handle_) != STATUS_OK) {
            std::cerr << "[Port " << config_.port << "] Failed to start CAN!" << std::endl;
            ZCAN_CloseDevice(device_handle_);
            return false;
        }

        // Enable TX echo
        UINT echo_enable = 1;
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, config_.device_index,
                         config_.channel, SETREF_SET_TX_ECHO_ENABLE, &echo_enable);

        std::cout << "[Port " << config_.port << "] Ready (motors "
                  << (config_.motor_offset + 1) << "-"
                  << (config_.motor_offset + config_.motor_count)
                  << ", CAN ID: 1-" << config_.motor_count << ")" << std::endl;
        return true;
    }

    void StartSendThread() {
        running_ = true;
        send_thread_ = std::thread(&PortSender::SendLoop, this);
    }

    void StopSendThread() {
        running_ = false;
        if (send_thread_.joinable()) {
            send_thread_.join();
        }
    }

    void stop() {
        StopSendThread();
        if (channel_handle_) {
            ZCAN_ResetCAN(channel_handle_);
        }
        if (device_handle_) {
            ZCAN_CloseDevice(device_handle_);
        }
    }

    uint64_t GetSendCount() const { return send_count_; }

private:
    void SendLoop() {
        // Prepare frames for all motors on this port
        // Motor ID 1-30 directly as CAN ID (matches motor_feedback_publisher expectations)
        std::vector<ZCAN_TransmitFD_Data> frames(config_.motor_count);
        for (int i = 0; i < config_.motor_count; i++) {
            memset(&frames[i], 0, sizeof(frames[i]));
            // Calculate motor_id (1-based global motor index)
            int motor_id = config_.motor_offset + i + 1;  // 1-based motor ID
            // CAN ID = motor ID (1-30)
            frames[i].frame.can_id = motor_id;
            frames[i].frame.len = 8;
            // Initial data: motor number in each byte
            for (int j = 0; j < 8; j++) {
                frames[i].frame.data[j] = (motor_id) & 0xFF;
            }
        }

        while (running_ && g_running) {
            // Send all frames for this port
            for (int i = 0; i < config_.motor_count; i++) {
                uint32_t result = ZCAN_TransmitFD(channel_handle_, &frames[i], 1);
                if (result == 1) {
                    send_count_++;
                    if (verbose_ && send_count_ <= 5) {
                        int motor_id = config_.motor_offset + i;
                        std::cout << "[Port " << config_.port
                                  << "] Motor ID 0x" << std::hex << std::setfill('0') << std::setw(3) << motor_id << std::dec
                                  << " (CAN ID=0x" << std::hex << std::setfill('0') << std::setw(3) << (int)frames[i].frame.can_id << std::dec << ") sent" << std::endl;
                    }
                }
                // Update data to make it more interesting
                for (int j = 0; j < 8; j++) {
                    frames[i].frame.data[j] = (frames[i].frame.data[j] + 1) & 0xFF;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms_));
        }
    }

    PortConfig config_;
    const char* zlg_ip_;
    int interval_ms_;
    bool verbose_;

    DEVICE_HANDLE device_handle_ = nullptr;
    CHANNEL_HANDLE channel_handle_ = nullptr;
    std::thread send_thread_;
    std::atomic<bool> running_{false};
    std::atomic<uint64_t> send_count_{0};
};

int main(int argc, char** argv) {
    // Setup signal handler
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    // Configuration
    const char* zlg_ip = "192.168.1.5";
    int send_interval_ms = 100;  // Send every 100ms
    bool verbose = true;

    // Parse optional arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--interval" && i + 1 < argc) {
            send_interval_ms = std::atoi(argv[++i]);
        } else if (arg == "--quiet") {
            verbose = false;
        } else if (arg == "--ip" && i + 1 < argc) {
            zlg_ip = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n";
            std::cout << "Sends test CAN frames to 4 ports simultaneously\n\n";
            std::cout << "Options:\n";
            std::cout << "  --interval <n>  Send interval in ms (default: 100)\n";
            std::cout << "  --ip <address>  ZLG device IP (default: 192.168.1.5)\n";
            std::cout << "  --quiet         Don't print each frame\n";
            std::cout << "  -h, --help      Show this help message\n";
            std::cout << "\nPort configuration:\n";
            std::cout << "  Port 8000: motors 1-8   (device 0, channel 0)\n";
            std::cout << "  Port 8001: motors 9-16  (device 1, channel 1)\n";
            std::cout << "  Port 8002: motors 17-24 (device 2, channel 2)\n";
            std::cout << "  Port 8003: motors 25-30 (device 3, channel 3)\n";
            return 0;
        }
    }

    std::cout << "=== ZLG Multi-Port CAN Send Test ===" << std::endl;
    std::cout << "ZLG IP: " << zlg_ip << std::endl;
    std::cout << "Ports: 8000, 8001, 8002, 8003" << std::endl;
    std::cout << "Total motors: 30 (8+8+8+6)" << std::endl;
    std::cout << "Send interval: " << send_interval_ms << "ms" << std::endl;
    std::cout << "=====================================" << std::endl;

    // Create port senders
    std::vector<std::unique_ptr<PortSender>> senders;
    for (int i = 0; i < NUM_PORTS; i++) {
        auto sender = std::make_unique<PortSender>(PORT_CONFIGS[i], zlg_ip, send_interval_ms, verbose);
        if (!sender->Initialize()) {
            std::cerr << "[ERROR] Failed to initialize port " << PORT_CONFIGS[i].port << std::endl;
            // Continue with other ports
        } else {
            senders.push_back(std::move(sender));
        }
    }

    if (senders.empty()) {
        std::cerr << "[ERROR] No ports initialized!" << std::endl;
        return 1;
    }

    std::cout << "\n=== Initialized " << senders.size() << "/" << NUM_PORTS << " ports ===" << std::endl;

    // Start all send threads
    std::cout << "\n[Send] Starting all threads (Ctrl+C to stop)..." << std::endl;
    for (auto& sender : senders) {
        sender->StartSendThread();
    }

    // Main loop - keep running until interrupted
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // Print status every 5 seconds
        if (verbose) {
            std::cout << "\n=== Status ===" << std::endl;
            uint64_t total_count = 0;
            for (size_t i = 0; i < senders.size(); i++) {
                uint64_t count = senders[i]->GetSendCount();
                total_count += count;
                std::cout << "  Port " << PORT_CONFIGS[i].port << ": " << count << " frames" << std::endl;
            }
            std::cout << "  Total: " << total_count << " frames" << std::endl;
            std::cout << "=============" << std::endl;
        }
    }

    // Cleanup is handled by destructors
    std::cout << "\nDone!" << std::endl;

    return 0;
}
