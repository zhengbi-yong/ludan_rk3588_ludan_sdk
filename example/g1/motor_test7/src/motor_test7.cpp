// motor_test7.cpp
// Test program for multi-port motor control
// Controls motors on ports 8001 (ID 12), 8002 (ID 21), 8003 (ID 30)
// Command: pos=1, vel=0, kp=1, kd=1, tau=0

#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <atomic>
#include <vector>
#include <signal.h>

// ZLG CANFDNET SDK
#include "CANFDNET.h"

// ==================== Configuration ====================
const std::string ZLG_IP = "192.168.1.5";
const int ARB_BAUD = 1000000;   // 1M bps
const int DATA_BAUD = 1000000;  // 1M bps (DM motor requires same rate, no BRS)

// Motor command structure
struct MotorCommand {
    uint16_t motor_id;  // Global motor ID (1-30)
    float pos;
    float vel;
    float kp;
    float kd;
    float tau;
};

// Port configuration (partial - only ports 8001, 8002, 8003)
struct PortConfig {
    int port;         // TCP port
    int motor_count;  // Number of motors on this port
    int motor_offset; // Starting motor index (0-based)
    int channel;      // CAN channel number
};

constexpr PortConfig PORT_CONFIGS[3] = {
    {8001, 8, 8, 1},   // Motors 9-16  -> Local CAN ID 1-8
    {8002, 8, 16, 2},  // Motors 17-24 -> Local CAN ID 1-8
    {8003, 6, 24, 3}   // Motors 25-30 -> Local CAN ID 1-6
};

// ==================== MIT Motor Protocol ====================
// Convert motor command to CAN data bytes (MIT motor protocol format)
static inline void MotorCommandToCanData(const MotorCommand& cmd, uint8_t* can_data) {
    // Convert position to int16 (scale: +/-12.5 rad -> +/-32767)
    int16_t pos_int = static_cast<int16_t>(cmd.pos * 32767.0f / 12.5f);
    can_data[0] = pos_int & 0xFF;
    can_data[1] = (pos_int >> 8) & 0xFF;

    // Convert velocity to int16 (scale: +/-30 rad/s -> +/-32767)
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

// ==================== Single Port Sender ====================
class PortSender {
public:
    PortSender(const PortConfig& port_config, const std::string& zlg_ip,
               int arb_baud, int data_baud)
        : port_config_(port_config), zlg_ip_(zlg_ip),
          arb_baud_(arb_baud), data_baud_(data_baud) {}

    ~PortSender() {
        if (channel_handle_ && channel_handle_ != INVALID_CHANNEL_HANDLE) {
            ZCAN_ResetCAN(channel_handle_);
        }
        if (device_handle_ && device_handle_ != INVALID_DEVICE_HANDLE) {
            ZCAN_CloseDevice(device_handle_);
        }
    }

    bool Initialize() {
        std::cout << "  [Port " << port_config_.port << "] Initializing..." << std::endl;

        // 1. Open device
        device_handle_ = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP,
                                         port_config_.channel, 0);
        if (device_handle_ == INVALID_DEVICE_HANDLE) {
            std::cerr << "  [Port " << port_config_.port << "] Failed to open device" << std::endl;
            return false;
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

        channel_handle_ = ZCAN_InitCAN(device_handle_, port_config_.channel, &init_config);
        if (channel_handle_ == INVALID_CHANNEL_HANDLE) {
            std::cerr << "  [Port " << port_config_.port << "] Failed to init CAN" << std::endl;
            ZCAN_CloseDevice(device_handle_);
            return false;
        }

        // 3. Set IP and port
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, port_config_.channel, port_config_.channel,
                         CMD_DESIP, (void*)zlg_ip_.c_str());
        uint32_t port_val = port_config_.port;
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, port_config_.channel, port_config_.channel,
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

    int SendMotorCommands(const std::vector<MotorCommand>& commands) {
        if (!initialized_) {
            return 0;
        }

        std::vector<ZCAN_Transmit_Data> frames;
        frames.reserve(commands.size());

        for (const auto& cmd : commands) {
            ZCAN_Transmit_Data transmit_data;
            memset(&transmit_data, 0, sizeof(transmit_data));

            transmit_data.frame.can_id = MAKE_CAN_ID(cmd.motor_id, 0, 0, 0);
            transmit_data.frame.can_dlc = 8;

            uint8_t can_data[8];
            MotorCommandToCanData(cmd, can_data);
            for (int i = 0; i < 8; i++) {
                transmit_data.frame.data[i] = can_data[i];
            }

            transmit_data.transmit_type = 0;
            frames.push_back(transmit_data);
        }

        uint32_t ret = ZCAN_Transmit(channel_handle_, frames.data(), frames.size());
        return ret;
    }

    bool SendEnableCommand(int local_motor_id) {
        if (!initialized_) return false;
        uint8_t enable_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
        ZCAN_Transmit_Data transmit_data;
        memset(&transmit_data, 0, sizeof(transmit_data));
        transmit_data.frame.can_id = MAKE_CAN_ID(local_motor_id, 0, 0, 0);
        transmit_data.frame.can_dlc = 8;
        memcpy(transmit_data.frame.data, enable_frame, 8);
        transmit_data.transmit_type = 0;
        return ZCAN_Transmit(channel_handle_, &transmit_data, 1) == 1;
    }

    bool IsInitialized() const { return initialized_; }

private:
    PortConfig port_config_;
    std::string zlg_ip_;
    int arb_baud_;
    int data_baud_;

    DEVICE_HANDLE device_handle_ = INVALID_DEVICE_HANDLE;
    CHANNEL_HANDLE channel_handle_ = INVALID_CHANNEL_HANDLE;
    bool initialized_ = false;
};

// ==================== Motor Port Manager ====================
class MotorPortManager {
public:
    MotorPortManager(const std::string& zlg_ip, int arb_baud, int data_baud)
        : zlg_ip_(zlg_ip), arb_baud_(arb_baud), data_baud_(data_baud) {}

    ~MotorPortManager() = default;

    bool Initialize() {
        std::cout << "\n=== Initializing Motor Port Manager ===" << std::endl;
        std::cout << "ZLG IP: " << zlg_ip_ << std::endl;
        std::cout << "Arbitration Baud: " << arb_baud_ << " bps" << std::endl;
        std::cout << "Data Baud: " << data_baud_ << " bps" << std::endl;

        for (int i = 0; i < 3; i++) {
            port_senders_[i] = std::make_unique<PortSender>(
                PORT_CONFIGS[i], zlg_ip_, arb_baud_, data_baud_);
            if (!port_senders_[i]->Initialize()) {
                std::cerr << "Failed to initialize port " << PORT_CONFIGS[i].port << std::endl;
                return false;
            }
        }

        std::cout << "=== All 3 Ports Initialized ===" << std::endl;
        return true;
    }

    int SendMotorCommands(const std::vector<MotorCommand>& all_commands) {
        // Group commands by port
        std::array<std::vector<MotorCommand>, 3> port_commands;

        for (const auto& cmd : all_commands) {
            int port_idx = GetPortIndexForMotor(cmd.motor_id);
            if (port_idx >= 0 && port_senders_[port_idx]->IsInitialized()) {
                MotorCommand local_cmd = cmd;
                local_cmd.motor_id = GetLocalCanId(cmd.motor_id);
                port_commands[port_idx].push_back(local_cmd);
            }
        }

        // Send to each port
        int total_sent = 0;
        for (int i = 0; i < 3; i++) {
            if (!port_commands[i].empty()) {
                int sent = port_senders_[i]->SendMotorCommands(port_commands[i]);
                total_sent += sent;
                std::cout << "Port " << PORT_CONFIGS[i].port << ": sent " << sent << " commands" << std::endl;
            }
        }

        return total_sent;
    }

    bool SendEnableCommands(const std::vector<int>& global_motor_ids) {
        std::cout << "\n>>> Sending ENABLE Commands <<<" << std::endl;

        // Group by port and send enable commands
        std::array<std::vector<int>, 3> port_motor_ids;

        for (int motor_id : global_motor_ids) {
            int port_idx = GetPortIndexForMotor(motor_id);
            if (port_idx >= 0 && port_senders_[port_idx]->IsInitialized()) {
                int local_can_id = GetLocalCanId(motor_id);
                port_motor_ids[port_idx].push_back(local_can_id);
            }
        }

        // Send enable commands to each port
        for (int i = 0; i < 3; i++) {
            if (!port_motor_ids[i].empty()) {
                for (int local_id : port_motor_ids[i]) {
                    if (port_senders_[i]->SendEnableCommand(local_id)) {
                        std::cout << "  Port " << PORT_CONFIGS[i].port << ", Local ID " << local_id
                                  << " (Global ID " << (PORT_CONFIGS[i].motor_offset + local_id) << "): ENABLED" << std::endl;
                    } else {
                        std::cerr << "  Port " << PORT_CONFIGS[i].port << ", Local ID " << local_id
                                   << ": FAILED" << std::endl;
                        return false;
                    }
                }
            }
        }

        std::cout << ">>> All motors ENABLED <<<" << std::endl;
        return true;
    }

private:
    int GetPortIndexForMotor(int global_motor_id) const {
        if (global_motor_id >= 9 && global_motor_id <= 16)  return 0;  // Port 8001
        if (global_motor_id >= 17 && global_motor_id <= 24) return 1;  // Port 8002
        if (global_motor_id >= 25 && global_motor_id <= 30) return 2;  // Port 8003
        return -1;
    }

    int GetLocalCanId(int global_motor_id) const {
        int port_idx = GetPortIndexForMotor(global_motor_id);
        if (port_idx < 0) return -1;
        return global_motor_id - PORT_CONFIGS[port_idx].motor_offset;
    }

private:
    std::string zlg_ip_;
    int arb_baud_;
    int data_baud_;
    std::array<std::unique_ptr<PortSender>, 3> port_senders_;
};

// ==================== Global Variables ====================
std::atomic<bool> running{true};

void signal_handler(int signum) {
    std::cout << "\nReceived signal " << signum << ", stopping..." << std::endl;
    running = false;
}

// ==================== Main ====================
int main(int argc, char* argv[]) {
    std::cout << "========================================" << std::endl;
    std::cout << "      Motor Test 7 - Multi-Port Test    " << std::endl;
    std::cout << "========================================" << std::endl;

    // Register signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Create motor port manager
    MotorPortManager manager(ZLG_IP, ARB_BAUD, DATA_BAUD);

    // Initialize ports
    if (!manager.Initialize()) {
        std::cerr << "Failed to initialize motor port manager" << std::endl;
        return 1;
    }

    // Prepare test commands
    // Port 8001, Local ID 4 -> Global ID 12
    // Port 8002, Local ID 5 -> Global ID 21
    // Port 8003, Local ID 6 -> Global ID 30
    std::vector<MotorCommand> commands = {
        {12, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f},  // Port 8001, Local ID 4
        {21, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f},  // Port 8002, Local ID 5
        {30, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f}   // Port 8003, Local ID 6
    };

    std::cout << "\n=== Test Commands ===" << std::endl;
    std::cout << "Motor 12 (Port 8001, Local ID 4): pos=1, vel=0, kp=1, kd=1, tau=0" << std::endl;
    std::cout << "Motor 21 (Port 8002, Local ID 5): pos=1, vel=0, kp=1, kd=1, tau=0" << std::endl;
    std::cout << "Motor 30 (Port 8003, Local ID 6): pos=1, vel=0, kp=1, kd=1, tau=0" << std::endl;
    std::cout << "========================" << std::endl;

    std::cout << "\nNOTE: Motors are NOT automatically enabled." << std::endl;
    std::cout << "Please enable motors separately using:" << std::endl;
    std::cout << "  ./motor_controller_with_enable --enable 12" << std::endl;
    std::cout << "  ./motor_controller_with_enable --enable 21" << std::endl;
    std::cout << "  ./motor_controller_with_enable --enable 30" << std::endl;
    std::cout << "\nStarting 500Hz command loop..." << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl << std::endl;

    // 500Hz loop
    const auto interval = std::chrono::milliseconds(2);
    auto next_send_time = std::chrono::high_resolution_clock::now();
    int count = 0;

    while (running) {
        // Send commands
        int sent = manager.SendMotorCommands(commands);
        count++;

        // Print status every 250 cycles (0.5 second)
        if (count % 250 == 0) {
            std::cout << "[" << count << "] Sent " << sent << " commands (3 ports)" << std::endl;
        }

        // Wait until next send time
        next_send_time += interval;
        std::this_thread::sleep_until(next_send_time);
    }

    std::cout << "\n=== Test Complete ===" << std::endl;
    std::cout << "Total commands sent: " << count * 3 << std::endl;

    return 0;
}
