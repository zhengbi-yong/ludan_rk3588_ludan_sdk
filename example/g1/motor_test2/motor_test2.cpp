// motor_test2.cpp
// Fixed position motor control - sends constant position command at specified rate
// No ROS2 dependency - direct CAN communication only

#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <atomic>
#include <signal.h>

// ZLG CANFDNET SDK
#include "CANFDNET.h"

// ==================== Configuration ====================
struct MotorTestConfig {
    // ZLG device configuration
    std::string zlg_ip = "192.168.1.5";
    int zlg_port = 8002;
    int channel = 2;
    int arb_baud = 1000000;   // 1M bps
    int data_baud = 5000000;  // 5M bps

    // Motor configuration
    int motor_id = 9;          // Motor ID to control (1-30)
    double position = 0.0;     // Target position (radians)
    double speed = 0.0;        // Target velocity (rad/s)
    double kp = 50.0;          // Position gain
    double kd = 1.5;           // Velocity gain
    double torque = 0.0;       // Feedforward torque (Nm)

    // Control loop configuration
    double control_rate = 500.0;  // Control rate in Hz
    double duration = 0.0;        // Duration in seconds (0 = infinite)
    bool enable_motor = true;     // Auto-enable motor on start
};

// ==================== Motor Command Structure ====================
struct MotorCommand {
    uint16_t motor_id;
    float pos;
    float vel;
    float kp;
    float kd;
    float tau;
};

// ==================== MIT Motor Protocol ====================
// Convert motor command to CAN data bytes (MIT motor protocol format)
// Bytes 0-1: Position (float to int16, ±12.5 rad -> ±32767)
// Bytes 2-3: Velocity (float to int16, ±30 rad/s -> ±32767)
// Bytes 4-5: Kp (float to int16, 0-500 -> 0-32767)
// Bytes 6-7: Kd (float to int16, 0-50 -> 0-32767)
inline void MotorCommandToCanData(const MotorCommand& cmd, uint8_t* can_data) {
    // Convert position to int16 (scale: ±12.5 rad -> ±32767)
    int16_t pos_int = static_cast<int16_t>(std::max(-12.5f, std::min(12.5f, cmd.pos)) * 32767.0f / 12.5f);
    can_data[0] = pos_int & 0xFF;
    can_data[1] = (pos_int >> 8) & 0xFF;

    // Convert velocity to int16 (scale: ±30 rad/s -> ±32767)
    int16_t vel_int = static_cast<int16_t>(std::max(-30.0f, std::min(30.0f, cmd.vel)) * 32767.0f / 30.0f);
    can_data[2] = vel_int & 0xFF;
    can_data[3] = (vel_int >> 8) & 0xFF;

    // Convert Kp to int16 (scale: 0-500 -> 0-32767)
    int16_t kp_int = static_cast<int16_t>(std::max(0.0f, std::min(500.0f, cmd.kp)) * 32767.0f / 500.0f);
    can_data[4] = kp_int & 0xFF;
    can_data[5] = (kp_int >> 8) & 0xFF;

    // Convert Kd to int16 (scale: 0-50 -> 0-32767)
    int16_t kd_int = static_cast<int16_t>(std::max(0.0f, std::min(50.0f, cmd.kd)) * 32767.0f / 50.0f);
    can_data[6] = kd_int & 0xFF;
    can_data[7] = (kd_int >> 8) & 0xFF;
}

// ==================== Motor Test Controller ====================
class MotorTestController {
public:
    MotorTestController(const MotorTestConfig& config) : config_(config) {}

    bool Initialize() {
        std::cout << "========================================" << std::endl;
        std::cout << "   Motor Test 2 - Fixed Position" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "ZLG: " << config_.zlg_ip << ":" << config_.zlg_port << std::endl;
        std::cout << "Channel: CAN" << config_.channel << std::endl;
        std::cout << "Motor ID: " << config_.motor_id << std::endl;
        std::cout << "Control Rate: " << config_.control_rate << " Hz" << std::endl;
        std::cout << "Duration: " << (config_.duration > 0 ? std::to_string(config_.duration) + "s" : "Infinite") << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Target Position: " << config_.position << " rad" << std::endl;
        std::cout << "Target Velocity: " << config_.speed << " rad/s" << std::endl;
        std::cout << "Kp: " << config_.kp << std::endl;
        std::cout << "Kd: " << config_.kd << std::endl;
        std::cout << "Torque: " << config_.torque << " Nm" << std::endl;
        std::cout << "========================================" << std::endl;

        // 1. Open ZCAN device
        device_handle_ = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, 0, 0);
        if (device_handle_ == INVALID_DEVICE_HANDLE) {
            std::cerr << "Failed to open ZCAN device" << std::endl;
            return false;
        }
        std::cout << "[1/4] Device opened" << std::endl;

        // 2. Initialize CAN channel
        ZCAN_CHANNEL_INIT_CONFIG init_config;
        memset(&init_config, 0, sizeof(init_config));
        init_config.can_type = TYPE_CANFD;
        init_config.canfd.acc_code = 0;
        init_config.canfd.acc_mask = 0;
        init_config.canfd.abit_timing = config_.arb_baud;
        init_config.canfd.dbit_timing = config_.data_baud;
        init_config.canfd.brp = 0;
        init_config.canfd.filter = 0;
        init_config.canfd.mode = 0;

        channel_handle_ = ZCAN_InitCAN(device_handle_, config_.channel, &init_config);
        if (channel_handle_ == INVALID_CHANNEL_HANDLE) {
            std::cerr << "Failed to initialize CAN channel" << std::endl;
            ZCAN_CloseDevice(device_handle_);
            return false;
        }
        std::cout << "[2/4] CAN channel initialized (CAN-FD " << config_.arb_baud << "/"
                  << config_.data_baud << ")" << std::endl;

        // 3. Set IP and port
        uint32_t val = 0;
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, config_.channel, CMD_DESIP,
                         (void*)config_.zlg_ip.c_str());
        val = config_.zlg_port;
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, config_.channel, CMD_DESPORT, &val);
        std::cout << "[3/4] IP:Port configured" << std::endl;

        // 4. Start CAN
        if (ZCAN_StartCAN(channel_handle_) != STATUS_OK) {
            std::cerr << "Failed to start CAN channel" << std::endl;
            ZCAN_CloseDevice(device_handle_);
            return false;
        }
        std::cout << "[4/4] CAN started" << std::endl;
        std::cout << "========================================" << std::endl;

        // Enable motor if configured
        if (config_.enable_motor) {
            EnableMotor();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        return true;
    }

    void EnableMotor() {
        uint8_t enable_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
        SendCanFrame(config_.motor_id, enable_frame, 8);
        std::cout << ">>> Motor " << config_.motor_id << " ENABLE sent <<<" << std::endl;
    }

    void DisableMotor() {
        uint8_t disable_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
        SendCanFrame(config_.motor_id, disable_frame, 8);
        std::cout << ">>> Motor " << config_.motor_id << " DISABLE sent <<<" << std::endl;
    }

    void RunControlLoop() {
        std::cout << "\n=== Starting Control Loop at " << config_.control_rate << " Hz ===" << std::endl;
        std::cout << "Press Ctrl+C to stop..." << std::endl;

        running_ = true;

        // Timing variables
        auto start_time = std::chrono::steady_clock::now();
        auto next_time = start_time;
        const auto control_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / config_.control_rate));

        uint64_t iteration = 0;
        auto last_print_time = start_time;

        while (running_) {
            // Check duration
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(current_time - start_time).count();

            // Exit if duration reached
            if (config_.duration > 0 && elapsed >= config_.duration) {
                std::cout << "\n>>> Duration " << config_.duration << "s reached, stopping <<<" << std::endl;
                break;
            }

            // Create motor command with fixed values
            MotorCommand cmd;
            cmd.motor_id = config_.motor_id;
            cmd.pos = static_cast<float>(config_.position);
            cmd.vel = static_cast<float>(config_.speed);
            cmd.kp = static_cast<float>(config_.kp);
            cmd.kd = static_cast<float>(config_.kd);
            cmd.tau = static_cast<float>(config_.torque);

            // Send motor command
            SendMotorCommand(cmd);

            // Print status every 0.5 seconds
            if (std::chrono::duration<double>(current_time - last_print_time).count() >= 0.5) {
                std::cout << "[" << std::fixed << std::setprecision(1) << std::setw(6) << elapsed << "s] "
                          << "pos=" << std::setprecision(4) << std::setw(8) << config_.position << " rad, "
                          << "vel=" << std::setprecision(4) << std::setw(8) << config_.speed << " rad/s, "
                          << "kp=" << std::setprecision(1) << std::setw(4) << config_.kp << ", "
                          << "kd=" << std::setprecision(1) << std::setw(4) << config_.kd << ", "
                          << "iter=" << iteration << std::endl;
                last_print_time = current_time;
            }

            iteration++;

            // Sleep until next control cycle
            next_time += control_period;
            std::this_thread::sleep_until(next_time);
        }

        std::cout << "\n=== Control Loop Stopped ===" << std::endl;
        std::cout << "Total iterations: " << iteration << std::endl;
        std::cout << "Total time: " << std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() << "s" << std::endl;
    }

    void Stop() {
        running_ = false;
    }

    ~MotorTestController() {
        DisableMotor();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (channel_handle_) {
            ZCAN_ResetCAN(channel_handle_);
        }
        if (device_handle_) {
            ZCAN_CloseDevice(device_handle_);
        }
    }

private:
    void SendMotorCommand(const MotorCommand& cmd) {
        uint8_t can_data[8];
        MotorCommandToCanData(cmd, can_data);
        SendCanFrame(cmd.motor_id, can_data, 8);
    }

    void SendCanFrame(uint16_t motor_id, const uint8_t* data, uint8_t len) {
        ZCAN_Transmit_Data transmit_data;
        memset(&transmit_data, 0, sizeof(transmit_data));

        transmit_data.frame.can_id = motor_id;
        transmit_data.frame.can_dlc = len;
        memcpy(transmit_data.frame.data, data, len);

        ZCAN_Transmit(channel_handle_, &transmit_data, 1);
    }

    MotorTestConfig config_;
    DEVICE_HANDLE device_handle_ = nullptr;
    CHANNEL_HANDLE channel_handle_ = nullptr;
    std::atomic<bool> running_{false};
};

// ==================== Signal Handler ====================
MotorTestController* g_controller = nullptr;

void SignalHandler(int signal) {
    if (g_controller) {
        std::cout << "\nReceived signal " << signal << ", stopping..." << std::endl;
        g_controller->Stop();
    }
}

// ==================== Main ====================
void PrintUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --ip <address>         ZLG device IP (default: 192.168.1.5)" << std::endl;
    std::cout << "  --port <port>          ZLG device port (default: 8002)" << std::endl;
    std::cout << "  --channel <num>        CAN channel (default: 2)" << std::endl;
    std::cout << "  --motor-id <id>        Motor ID to control (default: 9)" << std::endl;
    std::cout << "  --Position <rad>       Target position in rad (default: 0.0)" << std::endl;
    std::cout << "  --speed <rad/s>        Target velocity in rad/s (default: 0.0)" << std::endl;
    std::cout << "  --kp <value>           Position gain (default: 50.0)" << std::endl;
    std::cout << "  --kd <value>           Velocity gain (default: 1.5)" << std::endl;
    std::cout << "  --torque <Nm>          Feedforward torque (default: 0.0)" << std::endl;
    std::cout << "  --rate <hz>            Control rate in Hz (default: 500)" << std::endl;
    std::cout << "  --duration <s>         Duration in seconds (default: infinite)" << std::endl;
    std::cout << "  --no-enable            Don't auto-enable motor" << std::endl;
    std::cout << "  -h, --help             Show this help message" << std::endl;
    std::cout << "\nExample:" << std::endl;
    std::cout << "  " << program_name << " --motor-id 9 --Position 1 --speed 0 --kp 5 --kd 1" << std::endl;
}

int main(int argc, char** argv) {
    MotorTestConfig config;

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            PrintUsage(argv[0]);
            return 0;
        } else if (arg == "--ip" && i + 1 < argc) {
            config.zlg_ip = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            config.zlg_port = std::atoi(argv[++i]);
        } else if (arg == "--channel" && i + 1 < argc) {
            config.channel = std::atoi(argv[++i]);
        } else if (arg == "--motor-id" && i + 1 < argc) {
            config.motor_id = std::atoi(argv[++i]);
        } else if ((arg == "--Position" || arg == "--position") && i + 1 < argc) {
            config.position = std::atof(argv[++i]);
        } else if ((arg == "--speed" || arg == "--Speed") && i + 1 < argc) {
            config.speed = std::atof(argv[++i]);
        } else if ((arg == "--kp" || arg == "--Kp") && i + 1 < argc) {
            config.kp = std::atof(argv[++i]);
        } else if ((arg == "--kd" || arg == "--Kd") && i + 1 < argc) {
            config.kd = std::atof(argv[++i]);
        } else if ((arg == "--torque" || arg == "--Torque") && i + 1 < argc) {
            config.torque = std::atof(argv[++i]);
        } else if (arg == "--rate" && i + 1 < argc) {
            config.control_rate = std::atof(argv[++i]);
        } else if (arg == "--duration" && i + 1 < argc) {
            config.duration = std::atof(argv[++i]);
        } else if (arg == "--no-enable") {
            config.enable_motor = false;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            PrintUsage(argv[0]);
            return 1;
        }
    }

    // Validate motor ID
    if (config.motor_id < 1 || config.motor_id > 30) {
        std::cerr << "Error: Motor ID must be between 1 and 30" << std::endl;
        return 1;
    }

    // Setup signal handler
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    // Create and run controller
    MotorTestController controller(config);
    g_controller = &controller;

    if (!controller.Initialize()) {
        std::cerr << "Failed to initialize controller" << std::endl;
        return 1;
    }

    controller.RunControlLoop();

    std::cout << "Exiting..." << std::endl;
    return 0;
}
