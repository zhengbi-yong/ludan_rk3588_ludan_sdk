// motor_test5.cpp
// Send fixed CAN frame: 8A 3C 7F F0 10 33 37 FF (3 times)
// Based on motor_config.c from ludan_control_board

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <cstring>
#include <signal.h>
#include <atomic>

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
    std::string zlg_ip = "192.168.1.5";
    int zlg_port = 8002;
    int channel = 2;
    int arb_baud = 1000000;
    int data_baud = 1000000;
    int motor_id = 4;
    int send_interval_ms = 1000;
    int num_frames = 3;
};

// ==================== Unsigned Int to Float Conversion ====================
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// ==================== Motor Test Controller ====================
class MotorTest5Controller {
public:
    MotorTest5Controller(const MotorTestConfig& config) : config_(config) {}

    bool Initialize() {
        std::cout << "=== Motor Test5 - Send 8A 3C 7F F0 10 33 37 FF (3 times) ===" << std::endl;
        std::cout << "ZLG: " << config_.zlg_ip << ":" << config_.zlg_port << std::endl;
        std::cout << "Channel: CAN" << config_.channel << std::endl;
        std::cout << "Motor ID: " << config_.motor_id << std::endl;

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
                  << config_.data_baud << ", BRS disabled)" << std::endl;

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

    void SendFixedFrames() {
        std::cout << "\n=== Sending Frame: 8A 3C 7F F0 10 33 37 FF ===" << std::endl;

        // The fixed frame data
        uint8_t frame_data[8] = {0x8A, 0x3C, 0x7F, 0xF0, 0x10, 0x33, 0x37, 0xFF};

        // Parse and display frame info
        uint16_t pos_int = (frame_data[0] << 8) | frame_data[1];
        uint16_t vel_int = ((frame_data[2] << 4) | (frame_data[3] >> 4));
        uint16_t kp_int = (((frame_data[3] & 0x0F) << 8) | frame_data[4]);
        uint16_t kd_int = ((frame_data[5] << 4) | (frame_data[6] >> 4));
        uint16_t tor_int = (((frame_data[6] & 0x0F) << 8) | frame_data[7]);

        float pos = uint_to_float(pos_int, P_MIN, P_MAX, 16);
        float vel = uint_to_float(vel_int, V_MIN, V_MAX, 12);
        float kp = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
        float kd = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
        float tor = uint_to_float(tor_int, T_MIN, T_MAX, 12);

        std::cout << "Frame meaning: pos=" << pos << " rad, vel=" << vel << " rad/s, "
                  << "kp=" << kp << ", kd=" << kd << ", tor=" << tor << " Nm" << std::endl;

        // Send 3 times
        for (int i = 0; i < config_.num_frames && running_; i++) {
            std::cout << "\n>>> Sending Frame " << (i + 1) << "/3 <<<" << std::endl;
            std::cout << "    Data: ";
            for (int j = 0; j < 8; j++) {
                std::cout << std::hex << std::setw(2) << std::setfill('0')
                          << (int)frame_data[j] << " ";
            }
            std::cout << std::dec << std::endl;

            SendFDCanFrame(config_.motor_id, frame_data, 8);
            std::cout << "    Frame sent successfully" << std::endl;

            if (i < config_.num_frames - 1) {
                std::cout << "    Waiting " << config_.send_interval_ms << " ms..." << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(config_.send_interval_ms));
            }
        }

        std::cout << "\n=== All 3 Frames Sent ===" << std::endl;
    }

    void Stop() {
        running_ = false;
    }

    ~MotorTest5Controller() {
        if (channel_handle_) {
            ZCAN_ResetCAN(channel_handle_);
        }
        if (device_handle_) {
            ZCAN_CloseDevice(device_handle_);
        }
    }

private:
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
MotorTest5Controller* g_controller = nullptr;

void SignalHandler(int signal) {
    if (g_controller) {
        std::cout << "\nStopping..." << std::endl;
        g_controller->Stop();
    }
}

// ==================== Main ====================
int main(int argc, char** argv) {
    MotorTestConfig config;

    // Parse arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--ip" && i + 1 < argc) config.zlg_ip = argv[++i];
        else if (arg == "--port" && i + 1 < argc) config.zlg_port = std::atoi(argv[++i]);
        else if (arg == "--channel" && i + 1 < argc) config.channel = std::atoi(argv[++i]);
        else if (arg == "--motor-id" && i + 1 < argc) config.motor_id = std::atoi(argv[++i]);
        else if (arg == "--interval" && i + 1 < argc) config.send_interval_ms = std::atoi(argv[++i]);
        else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: motor_test5 [options]" << std::endl;
            std::cout << "Sends: 8A 3C 7F F0 10 33 37 FF (3 times)" << std::endl;
            return 0;
        }
    }

    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    MotorTest5Controller controller(config);
    g_controller = &controller;

    if (!controller.Initialize()) {
        return 1;
    }

    controller.SendFixedFrames();
    return 0;
}
