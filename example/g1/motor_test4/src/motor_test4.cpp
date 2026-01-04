// motor_test4.cpp
// Send fixed pre-calculated CAN frames to DM motor
// Based on motor_config.c from ludan_control_board
// Uses UNSIGNED integer encoding (motor_config.c style)

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
// From motor_config.h - DM4340 uses these ranges
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
    int arb_baud = 1000000;   // 1M bps (arbitration phase)
    int data_baud = 1000000;  // 1M bps (data phase, same as arb for DM motor)

    // Motor configuration
    int motor_id = 4;          // Motor ID to control
    int send_interval_ms = 1000;  // Send interval in milliseconds (1 second = 1Hz)

    // Number of frames to send
    int num_frames = 3;
};

// ==================== Float to Unsigned Int Conversion ====================
// This is the same encoding used in motor_config.c
// Formula: int = (x_float - x_min) * (2^bits - 1) / (x_max - x_min)
int float_to_uint(float x_float, float x_min, float x_max, int bits) {
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

// ==================== Unsigned Int to Float Conversion ====================
// This is the same decoding used in motor_config.c
// Formula: float = x_int * (x_max - x_min) / (2^bits - 1) + x_min
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// ==================== Fixed Frame Data ====================
// Frame data: 8A 3C 7F F0 10 33 37 FF
// This frame represents: pos=1.0 rad, vel=0.0 rad/s, kp=471.0, kd=1.0, tor=0.0 Nm
//
// Data packing (from mit_ctrl2 function):
//   data[0] = (pos_tmp >> 8);
//   data[1] = pos_tmp;
//   data[2] = (vel_tmp >> 4);
//   data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
//   data[4] = kp_tmp;
//   data[5] = (kd_tmp >> 4);
//   data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
//   data[7] = tor_tmp;

struct FixedFrame {
    const char* description;
    uint8_t data[8];
};

// Fixed frame data - send the same frame 3 times as specified
const FixedFrame FIXED_FRAMES[] = {
    {
        "Frame 1: pos=1.0 rad, vel=0.0 rad/s, kp=471.0, kd=1.0, tor=0.0 Nm",
        {0x8A, 0x3C, 0x7F, 0xF0, 0x10, 0x33, 0x37, 0xFF}
    },
    {
        "Frame 2: pos=1.0 rad, vel=0.0 rad/s, kp=471.0, kd=1.0, tor=0.0 Nm",
        {0x8A, 0x3C, 0x7F, 0xF0, 0x10, 0x33, 0x37, 0xFF}
    },
    {
        "Frame 3: pos=1.0 rad, vel=0.0 rad/s, kp=471.0, kd=1.0, tor=0.0 Nm",
        {0x8A, 0x3C, 0x7F, 0xF0, 0x10, 0x33, 0x37, 0xFF}
    }
};

// ==================== Motor Test Controller ====================
class MotorTest4Controller {
public:
    MotorTest4Controller(const MotorTestConfig& config) : config_(config) {}

    bool Initialize() {
        std::cout << "=== Initializing Motor Test4 Controller (FDCAN Mode) ===" << std::endl;
        std::cout << "ZLG: " << config_.zlg_ip << ":" << config_.zlg_port << std::endl;
        std::cout << "Channel: CAN" << config_.channel << " (FDCAN mode)" << std::endl;
        std::cout << "Motor ID: " << config_.motor_id << std::endl;
        std::cout << "Send interval: " << config_.send_interval_ms << " ms ("
                  << (1000.0 / config_.send_interval_ms) << " Hz)" << std::endl;
        std::cout << "Number of frames: " << config_.num_frames << std::endl;
        std::cout << "Encoding: UNSIGNED (motor_config.c style)" << std::endl;

        // 1. Open ZCAN device
        device_handle_ = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, 0, 0);
        if (device_handle_ == INVALID_DEVICE_HANDLE) {
            std::cerr << "Failed to open ZCAN device" << std::endl;
            return false;
        }
        std::cout << "[1/4] Device opened" << std::endl;

        // 2. Initialize CAN channel with FDCAN mode
        ZCAN_CHANNEL_INIT_CONFIG init_config;
        memset(&init_config, 0, sizeof(init_config));
        init_config.can_type = TYPE_CANFD;  // Enable CAN FD mode
        init_config.canfd.acc_code = 0;
        init_config.canfd.acc_mask = 0;
        init_config.canfd.abit_timing = config_.arb_baud;  // Arbitration phase baudrate
        init_config.canfd.dbit_timing = config_.data_baud;  // Data phase baudrate
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
                  << config_.data_baud << ", BRS disabled)" << std::endl;

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
        std::cout << "[4/4] CAN started (FDCAN mode enabled)" << std::endl;
        std::cout << "=========================================" << std::endl;

        return true;
    }

    void SendFixedFrames() {
        std::cout << "\n=== Sending Fixed Frames ===" << std::endl;

        int num_to_send = std::min(config_.num_frames, (int)(sizeof(FIXED_FRAMES) / sizeof(FixedFrame)));

        for (int i = 0; i < num_to_send && running_; i++) {
            const FixedFrame& frame = FIXED_FRAMES[i];

            std::cout << "\n>>> Sending Frame " << (i + 1) << "/" << num_to_send << " <<<" << std::endl;
            std::cout << "    " << frame.description << std::endl;
            std::cout << "    Data: ";
            for (int j = 0; j < 8; j++) {
                std::cout << std::hex << std::setw(2) << std::setfill('0')
                          << (int)frame.data[j] << " ";
            }
            std::cout << std::dec << std::endl;

            // Print detailed breakdown
            std::cout << "    Breakdown: " << std::endl;

            // Parse the frame (same as dm4340_fbdata parsing)
            uint16_t pos_int = (frame.data[0] << 8) | frame.data[1];
            uint16_t vel_int = ((frame.data[2] << 4) | (frame.data[3] >> 4));
            uint16_t kp_int = (((frame.data[3] & 0x0F) << 8) | frame.data[4]);
            uint16_t kd_int = ((frame.data[5] << 4) | (frame.data[6] >> 4));
            uint16_t tor_int = (((frame.data[6] & 0x0F) << 8) | frame.data[7]);

            // Convert to float values
            float pos = uint_to_float(pos_int, P_MIN, P_MAX, 16);
            float vel = uint_to_float(vel_int, V_MIN, V_MAX, 12);
            float kp = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
            float kd = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
            float tor = uint_to_float(tor_int, T_MIN, T_MAX, 12);

            std::cout << "      pos_int = 0x" << std::hex << pos_int << std::dec
                      << " (" << pos_int << ") -> pos = " << pos << " rad" << std::endl;
            std::cout << "      vel_int = 0x" << std::hex << vel_int << std::dec
                      << " (" << vel_int << ") -> vel = " << vel << " rad/s" << std::endl;
            std::cout << "      kp_int  = 0x" << std::hex << kp_int << std::dec
                      << " (" << kp_int << ") -> kp  = " << kp << std::endl;
            std::cout << "      kd_int  = 0x" << std::hex << kd_int << std::dec
                      << " (" << kd_int << ") -> kd  = " << kd << std::endl;
            std::cout << "      tor_int = 0x" << std::hex << tor_int << std::dec
                      << " (" << tor_int << ") -> tor = " << tor << " Nm" << std::endl;

            // Send the frame
            SendFDCanFrame(config_.motor_id, frame.data, 8);

            std::cout << "    Frame sent successfully" << std::endl;

            // Wait before sending next frame (except for the last one)
            if (i < num_to_send - 1) {
                std::cout << "    Waiting " << config_.send_interval_ms
                          << " ms before next frame..." << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(config_.send_interval_ms));
            }
        }

        std::cout << "\n=== All Frames Sent ===" << std::endl;
    }

    void Stop() {
        running_ = false;
    }

    ~MotorTest4Controller() {
        if (channel_handle_) {
            ZCAN_ResetCAN(channel_handle_);
        }
        if (device_handle_) {
            ZCAN_CloseDevice(device_handle_);
        }
    }

private:
    // Send FDCAN frame
    void SendFDCanFrame(uint16_t motor_id, const uint8_t* data, uint8_t len) {
        ZCAN_TransmitFD_Data transmit_fd_data;
        memset(&transmit_fd_data, 0, sizeof(transmit_fd_data));

        // Set CAN ID (standard 11-bit ID)
        transmit_fd_data.frame.can_id = motor_id;

        // Set data length
        transmit_fd_data.frame.len = len;

        // Copy data
        memcpy(transmit_fd_data.frame.data, data, len);

        // FDCAN Frame Configuration
        // DM motor requires fixed 1Mbps baudrate (no BRS)
        transmit_fd_data.frame.flags = 0x00;  // Disable BRS - use same baudrate for both phases

        // Set __res0 to explicitly indicate CANFD frame type
        transmit_fd_data.frame.__res0 = CANFD_FLAG_RES0_FRAME_TYPE_CANFD;

        // Set transmit_type (0 = normal transmission)
        transmit_fd_data.transmit_type = 0;

        // Use TransmitFD for CAN-FD frames
        uint32_t result = ZCAN_TransmitFD(channel_handle_, &transmit_fd_data, 1);
        if (result == 0) {
            std::cerr << "    [WARNING] Failed to send frame!" << std::endl;
        }
    }

    MotorTestConfig config_;
    DEVICE_HANDLE device_handle_ = nullptr;
    CHANNEL_HANDLE channel_handle_ = nullptr;
    std::atomic<bool> running_{true};
};

// ==================== Signal Handler ====================
MotorTest4Controller* g_controller = nullptr;

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
    std::cout << "  --ip <address>        ZLG device IP (default: 192.168.1.5)" << std::endl;
    std::cout << "  --port <port>         ZLG device port (default: 8002)" << std::endl;
    std::cout << "  --channel <num>       CAN channel (default: 2)" << std::endl;
    std::cout << "  --motor-id <id>       Motor ID to control (default: 4)" << std::endl;
    std::cout << "  --interval <ms>       Send interval in ms (default: 1000)" << std::endl;
    std::cout << "  --frames <n>          Number of frames to send (default: 3)" << std::endl;
    std::cout << "  -h, --help            Show this help message" << std::endl;
    std::cout << "\nThis program sends: 8A 3C 7F F0 10 33 37 FF (3 times)" << std::endl;
    std::cout << "\nExample:" << std::endl;
    std::cout << "  " << program_name << " --motor-id 4 --interval 1000" << std::endl;
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
        } else if (arg == "--interval" && i + 1 < argc) {
            config.send_interval_ms = std::atoi(argv[++i]);
        } else if (arg == "--frames" && i + 1 < argc) {
            config.num_frames = std::atoi(argv[++i]);
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
    MotorTest4Controller controller(config);
    g_controller = &controller;

    if (!controller.Initialize()) {
        std::cerr << "Failed to initialize controller" << std::endl;
        return 1;
    }

    // Send fixed frames
    controller.SendFixedFrames();

    std::cout << "Exiting..." << std::endl;
    return 0;
}
