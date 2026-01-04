// motor_test6.cpp
// Motor position control test with incremental angle changes
// Features:
//   - Motor from zero position
//   - Each rotation increases by a fixed angle increment
//   - After each rotation, return to zero position
//   - Configurable: motor_id, kp, kd, angle_increment, rotation_count, time_interval

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <cstring>
#include <signal.h>
#include <atomic>
#include <cmath>

// ZLG CANFDNET SDK
#include "CANFDNET.h"

// ==================== DM4340 Motor Parameters ====================
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -10.0f
#define V_MAX 10.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -28.0f
#define T_MAX 28.0f

// ==================== Configuration ====================
struct MotorTestConfig {
    // ZLG device configuration
    std::string zlg_ip = "192.168.1.5";
    int zlg_port = 8002;
    int channel = 2;
    int arb_baud = 1000000;
    int data_baud = 1000000;

    // Motor configuration
    int motor_id = 4;
    float kp = 50.0f;          // Proportional gain
    float kd = 1.0f;           // Derivative gain
    float angle_increment = 0.1f;  // Angle increment per rotation (radians)
    int rotation_count = 5;    // Number of rotation cycles
    int interval_ms = 1000;    // Time interval between commands (milliseconds)
};

// ==================== Float to Unsigned Int Conversion ====================
int float_to_uint(float x_float, float x_min, float x_max, int bits) {
    if (x_float < x_min) {
        x_float = x_min;
    } else if (x_float > x_max) {
        x_float = x_max;
    }
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

// ==================== Unsigned Int to Float Conversion ====================
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// ==================== Motor Test Controller ====================
class MotorTest6Controller {
public:
    MotorTest6Controller(const MotorTestConfig& config) : config_(config) {}

    bool Initialize() {
        std::cout << "=== Motor Test6 - Incremental Angle Rotation Test ===" << std::endl;
        std::cout << "ZLG: " << config_.zlg_ip << ":" << config_.zlg_port << std::endl;
        std::cout << "Channel: CAN" << config_.channel << std::endl;
        std::cout << "Motor ID: " << config_.motor_id << std::endl;
        std::cout << "Kp: " << config_.kp << ", Kd: " << config_.kd << std::endl;
        std::cout << "Angle Increment: " << config_.angle_increment << " rad ("
                  << (config_.angle_increment * 180.0f / M_PI) << " deg)" << std::endl;
        std::cout << "Rotation Count: " << config_.rotation_count << std::endl;
        std::cout << "Interval: " << config_.interval_ms << " ms" << std::endl;

        // Open ZCAN device
        device_handle_ = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, 0, 0);
        if (device_handle_ == INVALID_DEVICE_HANDLE) {
            std::cerr << "Failed to open ZCAN device" << std::endl;
            return false;
        }
        std::cout << "[1/4] Device opened" << std::endl;

        // Initialize CAN channel
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

        // Set IP and port
        uint32_t val = 0;
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, config_.channel, CMD_DESIP,
                         (void*)config_.zlg_ip.c_str());
        val = config_.zlg_port;
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, config_.channel, CMD_DESPORT, &val);
        std::cout << "[3/4] IP:Port configured" << std::endl;

        // Start CAN
        if (ZCAN_StartCAN(channel_handle_) != STATUS_OK) {
            std::cerr << "Failed to start CAN channel" << std::endl;
            ZCAN_CloseDevice(device_handle_);
            return false;
        }
        std::cout << "[4/4] CAN started" << std::endl;
        std::cout << "=========================================" << std::endl;

        return true;
    }

    void RunIncrementalRotationTest() {
        std::cout << "\n=== Starting Incremental Rotation Test ===" << std::endl;

        float current_angle = 0.0f;  // Start from zero position

        for (int cycle = 0; cycle < config_.rotation_count && running_; cycle++) {
            // Calculate target angle for this cycle
            float target_angle = config_.angle_increment * (cycle + 1);

            std::cout << "\n>>> Cycle " << (cycle + 1) << "/" << config_.rotation_count << " <<<" << std::endl;

            // Step 1: Rotate to target angle
            std::cout << "  [Step 1] Rotating to " << target_angle << " rad ("
                      << (target_angle * 180.0f / M_PI) << " deg)..." << std::endl;
            SendMotorCommand(target_angle, 0.0f, config_.kp, config_.kd, 0.0f);
            std::cout << "  Command sent, waiting " << config_.interval_ms << " ms..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(config_.interval_ms));

            // Step 2: Rotate back to zero position
            std::cout << "  [Step 2] Rotating back to 0 rad (zero position)..." << std::endl;
            SendMotorCommand(0.0f, 0.0f, config_.kp, config_.kd, 0.0f);

            if (cycle < config_.rotation_count - 1) {
                std::cout << "  Command sent, waiting " << config_.interval_ms << " ms..." << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(config_.interval_ms));
            }
        }

        std::cout << "\n=== Test Complete ===" << std::endl;
        std::cout << "Total cycles completed: " << config_.rotation_count << std::endl;
        std::cout << "Final position: 0 rad (zero position)" << std::endl;
    }

    void Stop() {
        running_ = false;
    }

    ~MotorTest6Controller() {
        if (channel_handle_) {
            ZCAN_ResetCAN(channel_handle_);
        }
        if (device_handle_) {
            ZCAN_CloseDevice(device_handle_);
        }
    }

private:
    void SendMotorCommand(float pos, float vel, float kp, float kd, float tor) {
        // Convert parameters to unsigned integers
        uint16_t pos_int = float_to_uint(pos, P_MIN, P_MAX, 16);
        uint16_t vel_int = float_to_uint(vel, V_MIN, V_MAX, 12);
        uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
        uint16_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
        uint16_t tor_int = float_to_uint(tor, T_MIN, T_MAX, 12);

        // Pack data into CAN frame (MIT format)
        uint8_t frame_data[8];
        frame_data[0] = (pos_int >> 8);
        frame_data[1] = pos_int;
        frame_data[2] = (vel_int >> 4);
        frame_data[3] = ((vel_int & 0xF) << 4) | (kp_int >> 8);
        frame_data[4] = kp_int;
        frame_data[5] = (kd_int >> 4);
        frame_data[6] = ((kd_int & 0xF) << 4) | (tor_int >> 8);
        frame_data[7] = tor_int;

        // Print frame info
        std::cout << "    Frame: pos=" << pos << " rad, vel=" << vel << " rad/s, "
                  << "kp=" << kp << ", kd=" << kd << ", tor=" << tor << " Nm" << std::endl;
        std::cout << "    Data: ";
        for (int i = 0; i < 8; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << (int)frame_data[i] << " ";
        }
        std::cout << std::dec << std::endl;

        SendFDCanFrame(config_.motor_id, frame_data, 8);
    }

    void SendFDCanFrame(uint16_t motor_id, const uint8_t* data, uint8_t len) {
        ZCAN_TransmitFD_Data transmit_fd_data;
        memset(&transmit_fd_data, 0, sizeof(transmit_fd_data));

        transmit_fd_data.frame.can_id = motor_id;
        transmit_fd_data.frame.len = len;
        memcpy(transmit_fd_data.frame.data, data, len);
        transmit_fd_data.frame.flags = 0x00;  // BRS disabled
        transmit_fd_data.frame.__res0 = CANFD_FLAG_RES0_FRAME_TYPE_CANFD;
        transmit_fd_data.transmit_type = 0;

        ZCAN_TransmitFD(channel_handle_, &transmit_fd_data, 1);
    }

    MotorTestConfig config_;
    DEVICE_HANDLE device_handle_ = nullptr;
    CHANNEL_HANDLE channel_handle_ = nullptr;
    std::atomic<bool> running_{true};
};

// ==================== Signal Handler ====================
MotorTest6Controller* g_controller = nullptr;

void SignalHandler(int signal) {
    if (g_controller) {
        std::cout << "\nStopping..." << std::endl;
        g_controller->Stop();
    }
}

// ==================== Main ====================
void PrintUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --motor-id <id>        Motor ID to control (default: 4)" << std::endl;
    std::cout << "  --kp <value>           Proportional gain (default: 50.0)" << std::endl;
    std::cout << "  --kd <value>           Derivative gain (default: 1.0)" << std::endl;
    std::cout << "  --angle-inc <value>    Angle increment per rotation in radians (default: 0.1)" << std::endl;
    std::cout << "  --count <n>            Number of rotation cycles (default: 5)" << std::endl;
    std::cout << "  --interval <ms>        Time interval between commands in ms (default: 1000)" << std::endl;
    std::cout << "  --ip <address>         ZLG device IP (default: 192.168.1.5)" << std::endl;
    std::cout << "  --port <port>          ZLG device port (default: 8002)" << std::endl;
    std::cout << "  --channel <num>        CAN channel (default: 2)" << std::endl;
    std::cout << "  -h, --help             Show this help message" << std::endl;
    std::cout << "\nDescription:" << std::endl;
    std::cout << "  This program performs incremental angle rotation test:" << std::endl;
    std::cout << "  - Starts from zero position" << std::endl;
    std::cout << "  - Each cycle rotates to a target angle (increasing each cycle)" << std::endl;
    std::cout << "  - Then rotates back to zero position" << std::endl;
    std::cout << "  - Repeats for specified number of cycles" << std::endl;
    std::cout << "\nExample:" << std::endl;
    std::cout << "  " << program_name << " --motor-id 4 --kp 50 --kd 1 --angle-inc 0.1 --count 5" << std::endl;
}

int main(int argc, char** argv) {
    MotorTestConfig config;

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            PrintUsage(argv[0]);
            return 0;
        } else if (arg == "--motor-id" && i + 1 < argc) {
            config.motor_id = std::atoi(argv[++i]);
        } else if (arg == "--kp" && i + 1 < argc) {
            config.kp = std::atof(argv[++i]);
        } else if (arg == "--kd" && i + 1 < argc) {
            config.kd = std::atof(argv[++i]);
        } else if (arg == "--angle-inc" && i + 1 < argc) {
            config.angle_increment = std::atof(argv[++i]);
        } else if (arg == "--count" && i + 1 < argc) {
            config.rotation_count = std::atoi(argv[++i]);
        } else if (arg == "--interval" && i + 1 < argc) {
            config.interval_ms = std::atoi(argv[++i]);
        } else if (arg == "--ip" && i + 1 < argc) {
            config.zlg_ip = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            config.zlg_port = std::atoi(argv[++i]);
        } else if (arg == "--channel" && i + 1 < argc) {
            config.channel = std::atoi(argv[++i]);
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            PrintUsage(argv[0]);
            return 1;
        }
    }

    // Validate parameters
    if (config.motor_id < 1 || config.motor_id > 30) {
        std::cerr << "Error: Motor ID must be between 1 and 30" << std::endl;
        return 1;
    }
    if (config.kp < KP_MIN || config.kp > KP_MAX) {
        std::cerr << "Error: Kp must be between " << KP_MIN << " and " << KP_MAX << std::endl;
        return 1;
    }
    if (config.kd < KD_MIN || config.kd > KD_MAX) {
        std::cerr << "Error: Kd must be between " << KD_MIN << " and " << KD_MAX << std::endl;
        return 1;
    }
    if (config.angle_increment <= 0 || config.angle_increment * (config.rotation_count + 1) > P_MAX) {
        std::cerr << "Error: Invalid angle increment or would exceed maximum position" << std::endl;
        return 1;
    }
    if (config.rotation_count <= 0) {
        std::cerr << "Error: Rotation count must be positive" << std::endl;
        return 1;
    }

    // Setup signal handler
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    // Create and run controller
    MotorTest6Controller controller(config);
    g_controller = &controller;

    if (!controller.Initialize()) {
        std::cerr << "Failed to initialize controller" << std::endl;
        return 1;
    }

    controller.RunIncrementalRotationTest();

    std::cout << "\nExiting..." << std::endl;
    return 0;
}
