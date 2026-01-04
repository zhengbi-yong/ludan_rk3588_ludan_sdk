// motor_test.cpp
// Direct motor control with frequency sweep and feedback logging
// No ROS2 dependency - direct CAN communication only

#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <atomic>
#include <vector>
#include <signal.h>
#include <fstream>
#include <sstream>
#include <mutex>
#include <queue>

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
    double kp = 10.0;          // Position gain
    double kd = 1.5;           // Velocity gain
    double torque = 0.0;       // Feedforward torque (Nm)

    // Sine wave configuration
    double amplitude = 0.5;    // Amplitude in radians
    double start_freq = 0.1;   // Start frequency in Hz
    double end_freq = 20.0;    // End frequency in Hz
    double freq_step = 0.05;   // Frequency step in Hz (discrete points)
    double cycles_per_freq = 1.0;  // Number of cycles to hold per frequency
    double offset = 0.0;       // Offset in radians

    // Control loop configuration
    double control_rate = 50.0;   // Control rate in Hz (reduced to 1/10th of original 500 Hz)
    bool enable_motor = true;     // Auto-enable motor on start

    // Logging configuration
    std::string log_dir = "log";
    bool enable_logging = true;
};

// ==================== DM Motor Frame Decoder ====================
// DM motor feedback format:
// D[0] D[1] D[2] D[3] D[4] D[5] D[6] D[7]
// ID|ERR<<4, POS[15:8], POS[7:0], VEL[11:4], VEL[3:0]|T[11:8], T[7:0], T_MOS, T_Rotor
//
// Encoding (float to fixed-point linear mapping):
//   POS_16bit  = position_rad  / (PMAX - PMIN) * 2^16 + 2^15
//   VEL_12bit = velocity_rad_s / (VMAX - VMIN) * 2^12 + 2^11
//   T_12bit   = torque_nm      / (TMAX - TMIN) * 2^12 + 2^11
//
// Decoding (fixed-point to float):
//   position_rad  = (POS_16bit  - 2^15) / 2^16 * (PMAX - PMIN)
//   velocity_rad_s = (VEL_12bit - 2^11) / 2^12 * (VMAX - VMIN)
//   torque_nm      = (T_12bit   - 2^11) / 2^12 * (TMAX - TMIN)

struct DMMotorData {
    // Raw CAN frame data
    uint8_t raw[8];

    // Decoded values (DM motor format, fixed-point)
    uint8_t  motor_id;   // Motor ID (4 bits from D[0] lower)
    uint8_t  error;      // Error code (4 bits from D[0] upper)
    int16_t  position;   // Position raw (16-bit signed from D[1], D[2])
    int16_t  velocity;   // Velocity raw (12-bit signed from D[3], D[4] lower)
    int16_t  torque;     // Torque raw (12-bit signed from D[4] upper, D[5])
    int8_t   temp_mos;   // MOS temperature (8-bit signed from D[6])
    int8_t   temp_rotor; // Rotor temperature (8-bit signed from D[7])

    // Decoded physical values (linear mapping from fixed-point)
    double position_rad;   // Position in radians
    double velocity_rad_s; // Velocity in rad/s
    double torque_nm;      // Torque in Nm
};

class DMMotorFrameDecoder {
public:
    // DM motor range limits (adjust based on your motor specifications)
    static constexpr double PMAX = 3.14159;    // Position max: π rad (180°)
    static constexpr double PMIN = -3.14159;   // Position min: -π rad (-180°)
    static constexpr double VMAX = 45.0;       // Velocity max: 45 rad/s
    static constexpr double VMIN = -45.0;      // Velocity min: -45 rad/s
    static constexpr double TMAX = 20.0;       // Torque max: 20 Nm
    static constexpr double TMIN = -20.0;      // Torque min: -20 Nm

    static DMMotorData DecodeFrameFD(const ZCAN_ReceiveFD_Data& frame) {
        DMMotorData data;
        memset(&data, 0, sizeof(data));

        const uint8_t* d = frame.frame.data;

        // Store raw data
        memcpy(data.raw, d, 8);

        // D[0]: ID[3:0] | ERR[3:0]<<4
        data.motor_id = d[0] & 0x0F;           // Lower 4 bits: Motor ID
        data.error = (d[0] >> 4) & 0x0F;       // Upper 4 bits: Error code

        // D[1-2]: Position (16-bit encoded value, range: [0, 2^16-1])
        // Encoding: POS_16bit = position_rad / (PMAX - PMIN) * 2^16 + 2^15
        // Decoding: position_rad = (POS_16bit - 2^15) / 2^16 * (PMAX - PMIN)
        uint16_t pos_encoded = static_cast<uint16_t>((d[1] << 8) | d[2]);
        data.position = static_cast<int16_t>(pos_encoded);
        data.position_rad = (pos_encoded - 32768.0) / 65536.0 * (PMAX - PMIN);

        // D[3-4]: Velocity (12-bit encoded as unsigned, range: [0, 2^12-1])
        // Encoding: VEL_12bit = velocity_rad_s / (VMAX - VMIN) * 2^12 + 2^11
        // Decoding: velocity_rad_s = (VEL_12bit - 2^11) / 2^12 * (VMAX - VMIN)
        int16_t vel_encoded = ((d[3] & 0xFF) << 4) | (d[4] & 0x0F);
        data.velocity = vel_encoded;
        data.velocity_rad_s = (vel_encoded - 2048.0) / 4096.0 * (VMAX - VMIN);

        // D[4-5]: Torque (12-bit encoded as unsigned, range: [0, 2^12-1])
        // Encoding: T_12bit = torque_nm / (TMAX - TMIN) * 2^12 + 2^11
        // Decoding: torque_nm = (T_12bit - 2^11) / 2^12 * (TMAX - TMIN)
        int16_t torque_encoded = (((d[4] >> 4) & 0x0F) << 8) | d[5];
        data.torque = torque_encoded;
        data.torque_nm = (torque_encoded - 2048.0) / 4096.0 * (TMAX - TMIN);

        // D[6-7]: Temperatures (8-bit signed, directly in Celsius)
        data.temp_mos = static_cast<int8_t>(d[6]);
        data.temp_rotor = static_cast<int8_t>(d[7]);

        return data;
    }
};

// ==================== Motor Feedback Data ====================
struct MotorFeedback {
    double timestamp;       // Timestamp in seconds
    double position;        // Position in radians
    double velocity;        // Velocity in rad/s
    double torque;          // Torque in Nm
    uint8_t  motor_id;
    uint8_t  error;
    int16_t  raw_pos;
    int16_t  raw_vel;
    int16_t  raw_torque;
    int8_t   temp_mos;
    int8_t   temp_rotor;
    double  frequency;      // Command frequency in Hz
    bool valid;
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
inline void MotorCommandToCanData(const MotorCommand& cmd, uint8_t* can_data) {
    int16_t pos_int = static_cast<int16_t>(std::max(-12.5f, std::min(12.5f, cmd.pos)) * 32767.0f / 12.5f);
    can_data[0] = pos_int & 0xFF;
    can_data[1] = (pos_int >> 8) & 0xFF;

    int16_t vel_int = static_cast<int16_t>(std::max(-30.0f, std::min(30.0f, cmd.vel)) * 32767.0f / 30.0f);
    can_data[2] = vel_int & 0xFF;
    can_data[3] = (vel_int >> 8) & 0xFF;

    int16_t kp_int = static_cast<int16_t>(std::max(0.0f, std::min(500.0f, cmd.kp)) * 32767.0f / 500.0f);
    can_data[4] = kp_int & 0xFF;
    can_data[5] = (kp_int >> 8) & 0xFF;

    int16_t kd_int = static_cast<int16_t>(std::max(0.0f, std::min(50.0f, cmd.kd)) * 32767.0f / 50.0f);
    can_data[6] = kd_int & 0xFF;
    can_data[7] = (kd_int >> 8) & 0xFF;
}

// ==================== Motor Test Controller ====================
class MotorTestController {
public:
    MotorTestController(const MotorTestConfig& config) : config_(config) {}

    bool Initialize() {
        std::cout << "=== Initializing Motor Test Controller ===" << std::endl;
        std::cout << "ZLG: " << config_.zlg_ip << ":" << config_.zlg_port << std::endl;
        std::cout << "Channel: CAN" << config_.channel << std::endl;
        std::cout << "Motor ID: " << config_.motor_id << std::endl;
        std::cout << "Control Rate: " << config_.control_rate << " Hz" << std::endl;
        std::cout << "Frequency Sweep: " << config_.start_freq << " -> " << config_.end_freq
                  << " Hz, step=" << config_.freq_step << " Hz" << std::endl;
        std::cout << "Amplitude: " << config_.amplitude << " rad" << std::endl;
        std::cout << "Gains: kp=" << config_.kp << ", kd=" << config_.kd << std::endl;

        // 1. Open ZCAN device
        device_handle_ = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, 0, 0);
        if (device_handle_ == INVALID_DEVICE_HANDLE) {
            std::cerr << "Failed to open ZCAN device" << std::endl;
            return false;
        }
        std::cout << "[1/5] Device opened" << std::endl;

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
        std::cout << "[2/5] CAN channel initialized (CAN-FD " << config_.arb_baud << "/"
                  << config_.data_baud << ")" << std::endl;

        // 3. Configure network settings (before StartCAN)
        // Note: ZCAN_SetReference uses device_type as 1st param, channel_index is always 0
        uint32_t val = 1;
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, 0, SETREF_SET_DATA_RECV_MERGE, &val);

        val = 0;  // 0 = TCP client mode
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, 0, CMD_TCP_TYPE, &val);
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, 0, CMD_DESIP,
                         (void*)config_.zlg_ip.c_str());
        val = config_.zlg_port;
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, 0, CMD_DESPORT, &val);
        std::cout << "[3/5] TCP client mode, IP:Port configured" << std::endl;

        // 4. Start CAN
        if (ZCAN_StartCAN(channel_handle_) != STATUS_OK) {
            std::cerr << "Failed to start CAN channel" << std::endl;
            ZCAN_CloseDevice(device_handle_);
            return false;
        }
        std::cout << "[4/5] CAN started" << std::endl;

        // 5. Setup logging
        if (config_.enable_logging) {
            SetupLogging();
            std::cout << "[5/5] Logging enabled: " << log_file_path_ << std::endl;
        } else {
            std::cout << "[5/5] Logging disabled" << std::endl;
        }

        std::cout << "=========================================" << std::endl;

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

    void StartReceiveThread() {
        receive_thread_running_ = true;
        receive_thread_ = std::thread(&MotorTestController::ReceiveLoop, this);
        std::cout << ">>> CAN Receive Thread Started <<<" << std::endl;
    }

    void RunControlLoop() {
        std::cout << "\n=== Starting Control Loop at " << config_.control_rate << " Hz ===" << std::endl;

        // Generate discrete frequency points
        std::vector<double> freq_points;
        std::vector<double> freq_durations;
        double total_duration = 0.0;

        for (double f = config_.start_freq; f <= config_.end_freq + 1e-9; f += config_.freq_step) {
            freq_points.push_back(f);
            // Hold for at least N complete cycles: duration = cycles / frequency
            double duration = config_.cycles_per_freq / f;
            freq_durations.push_back(duration);
            total_duration += duration;
        }

        std::cout << "Frequency Sweep: " << config_.start_freq << " -> " << config_.end_freq
                  << " Hz, step=" << config_.freq_step << " Hz" << std::endl;
        std::cout << "Number of frequency points: " << freq_points.size() << std::endl;
        std::cout << "Cycles per frequency: " << config_.cycles_per_freq << std::endl;
        std::cout << "Estimated total duration: " << std::fixed << std::setprecision(1)
                  << total_duration << " s (" << total_duration / 60.0 << " min)" << std::endl;
        std::cout << "Press Ctrl+C to stop..." << std::endl;
        std::cout << std::fixed << std::setprecision(4);

        running_ = true;

        // Timing variables
        auto start_time = std::chrono::steady_clock::now();
        auto next_time = start_time;
        const auto control_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / config_.control_rate));

        uint64_t iteration = 0;
        uint64_t print_counter = 0;
        uint64_t feedback_count = 0;
        MotorFeedback last_feedback = {0};
        size_t freq_index = 0;
        double freq_start_time = 0.0;
        double current_freq = freq_points[0];

        // Initialize current command frequency
        current_cmd_frequency_.store(current_freq);

        std::cout << "\n>>> Starting Frequency Point " << (freq_index + 1) << "/" << freq_points.size()
                  << ": " << current_freq << " Hz (duration: " << freq_durations[0] << " s) <<<" << std::endl;

        while (running_ && freq_index < freq_points.size()) {
            // Calculate target position with current discrete frequency
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(current_time - start_time).count();
            double freq_elapsed = elapsed - freq_start_time;

            // Check if it's time to switch to next frequency
            if (freq_elapsed >= freq_durations[freq_index]) {
                freq_index++;
                if (freq_index >= freq_points.size()) {
                    break;  // All frequency points completed
                }
                current_freq = freq_points[freq_index];
                freq_start_time = elapsed;
                std::cout << "\n>>> Switching to Frequency Point " << (freq_index + 1) << "/"
                          << freq_points.size() << ": " << current_freq << " Hz (duration: "
                          << freq_durations[freq_index] << " s) <<<" << std::endl;
            }

            // Calculate phase for current frequency (continuous phase accumulation)
            double phase = current_freq * elapsed;

            // Sine wave: position = A * sin(2π * phase) + offset
            double target_pos = config_.amplitude * std::sin(2.0 * M_PI * phase) + config_.offset;

            // Calculate target velocity
            // velocity = A * 2π * f * cos(2π * phase)
            double target_vel = config_.amplitude * 2.0 * M_PI * current_freq *
                              std::cos(2.0 * M_PI * phase);

            // Create motor command
            MotorCommand cmd;
            cmd.motor_id = config_.motor_id;
            cmd.pos = static_cast<float>(target_pos);
            cmd.vel = static_cast<float>(target_vel);
            cmd.kp = static_cast<float>(config_.kp);
            cmd.kd = static_cast<float>(config_.kd);
            cmd.tau = static_cast<float>(config_.torque);

            // Send motor command
            SendMotorCommand(cmd);

            // Update current command frequency for logging
            current_cmd_frequency_.store(current_freq);

            // Get latest feedback
            {
                std::lock_guard<std::mutex> lock(feedback_mutex_);
                if (latest_feedback_.valid) {
                    last_feedback = latest_feedback_;
                    feedback_count++;
                }
            }

            // Print status every 50 iterations
            if (++print_counter >= 50) {
                double progress = (freq_elapsed / freq_durations[freq_index]) * 100.0;
                std::cout << "[" << std::setw(8) << elapsed << "s] "
                          << "Freq[" << (freq_index + 1) << "/" << freq_points.size() << "] "
                          << std::setw(6) << current_freq << " Hz "
                          << "(" << std::setw(5) << std::setprecision(1) << progress << "%), "
                          << "pos=" << std::setw(8) << target_pos << " rad, "
                          << "vel=" << std::setw(8) << target_vel << " rad/s";
                if (last_feedback.valid) {
                    std::cout << " | act_pos=" << std::setw(8) << last_feedback.position << " rad"
                              << ", act_vel=" << std::setw(8) << last_feedback.velocity << " rad/s";
                }
                std::cout << std::setprecision(4);
                std::cout << ", iter=" << iteration << std::endl;
                print_counter = 0;
            }

            iteration++;

            // Sleep until next control cycle
            next_time += control_period;
            std::this_thread::sleep_until(next_time);
        }

        std::cout << "\n=== Control Loop Stopped ===" << std::endl;
        std::cout << "Total iterations: " << iteration << std::endl;
        std::cout << "Total feedback received: " << feedback_count << std::endl;
        std::cout << "Frequency points completed: " << freq_index << "/" << freq_points.size() << std::endl;
    }

    void Stop() {
        running_ = false;
        receive_thread_running_ = false;
    }

    ~MotorTestController() {
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }

        DisableMotor();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (config_.enable_logging && log_file_.is_open()) {
            log_file_.close();
            std::cout << ">>> Log file saved: " << log_file_path_ << std::endl;
            std::cout << "    Total records: " << log_count_ << std::endl;
        }

        if (channel_handle_) {
            ZCAN_ResetCAN(channel_handle_);
        }
        if (device_handle_) {
            ZCAN_CloseDevice(device_handle_);
        }
    }

private:
    void SetupLogging() {
        // Create log directory if it doesn't exist
        std::string mkdir_cmd = "mkdir -p " + config_.log_dir;
        system(mkdir_cmd.c_str());

        // Generate log filename with timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << config_.log_dir << "/motor_log_" << config_.motor_id << "_"
           << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S") << ".csv";
        log_file_path_ = ss.str();

        // Open log file
        log_file_.open(log_file_path_);
        if (log_file_.is_open()) {
            // Write header
            log_file_ << "timestamp,motor_id,error,position,velocity,torque,raw_pos,raw_vel,raw_torque,temp_mos,temp_rotor,frequency\n";
            log_file_ << std::fixed << std::setprecision(6);
            log_count_ = 0;
        } else {
            std::cerr << "Failed to open log file: " << log_file_path_ << std::endl;
        }
    }

    void ReceiveLoop() {
        ZCAN_ReceiveFD_Data receive_buffer[100];
        uint64_t total_received = 0;
        uint64_t motor_feedback_count = 0;

        while (receive_thread_running_) {
            uint32_t received = ZCAN_ReceiveFD(channel_handle_, receive_buffer, 100, 10);

            if (received > 0) {
                auto now = std::chrono::steady_clock::now();
                double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();

                for (uint32_t i = 0; i < received; i++) {
                    uint32_t can_id = receive_buffer[i].frame.can_id & 0x7FF;

                    // Check if this is our motor's feedback (ID 1-30)
                    if (can_id >= 1 && can_id <= 30 && receive_buffer[i].frame.len >= 8) {
                        // Decode DM motor format
                        DMMotorData dm_data = DMMotorFrameDecoder::DecodeFrameFD(receive_buffer[i]);
                        motor_feedback_count++;

                        // Convert to MotorFeedback
                        MotorFeedback fb;
                        fb.timestamp = timestamp;
                        fb.motor_id = dm_data.motor_id;
                        fb.error = dm_data.error;
                        fb.position = dm_data.position_rad;
                        fb.velocity = dm_data.velocity_rad_s;
                        fb.torque = dm_data.torque_nm;
                        fb.raw_pos = dm_data.position;
                        fb.raw_vel = dm_data.velocity;
                        fb.raw_torque = dm_data.torque;
                        fb.temp_mos = dm_data.temp_mos;
                        fb.temp_rotor = dm_data.temp_rotor;
                        fb.frequency = current_cmd_frequency_.load();  // Get current command frequency
                        fb.valid = true;

                        // Store latest feedback
                        {
                            std::lock_guard<std::mutex> lock(feedback_mutex_);
                            latest_feedback_ = fb;
                        }

                        // Log to file
                        if (config_.enable_logging && log_file_.is_open()) {
                            std::lock_guard<std::mutex> lock(log_mutex_);
                            log_file_ << fb.timestamp << ","
                                     << static_cast<int>(fb.motor_id) << ","
                                     << static_cast<int>(fb.error) << ","
                                     << fb.position << ","
                                     << fb.velocity << ","
                                     << fb.torque << ","
                                     << fb.raw_pos << ","
                                     << fb.raw_vel << ","
                                     << fb.raw_torque << ","
                                     << static_cast<int>(fb.temp_mos) << ","
                                     << static_cast<int>(fb.temp_rotor) << ","
                                     << fb.frequency << "\n";
                            log_count_++;

                            // Flush periodically
                            if (log_count_ % 100 == 0) {
                                log_file_.flush();
                            }
                        }
                    }
                }
                total_received += received;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        std::cout << ">>> Receive Thread Stopped (total: " << total_received
                  << " frames, " << motor_feedback_count << " motor feedback) <<<" << std::endl;
    }

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
    std::atomic<bool> receive_thread_running_{false};

    // Receive thread
    std::thread receive_thread_;

    // Feedback data
    MotorFeedback latest_feedback_{0};
    std::mutex feedback_mutex_;

    // Current command frequency (shared between control and receive threads)
    std::atomic<double> current_cmd_frequency_{0.0};

    // Logging
    std::ofstream log_file_;
    std::string log_file_path_;
    std::mutex log_mutex_;
    uint64_t log_count_ = 0;
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
    std::cout << "  --ip <address>        ZLG device IP (default: 192.168.1.5)" << std::endl;
    std::cout << "  --port <port>         ZLG device port (default: 8002)" << std::endl;
    std::cout << "  --channel <num>       CAN channel (default: 2)" << std::endl;
    std::cout << "  --motor-id <id>       Motor ID to control (default: 9)" << std::endl;
    std::cout << "  --amplitude <rad>     Sine wave amplitude in rad (default: 0.5)" << std::endl;
    std::cout << "  --start-freq <hz>     Start frequency in Hz (default: 0.1)" << std::endl;
    std::cout << "  --end-freq <hz>       End frequency in Hz (default: 20.0)" << std::endl;
    std::cout << "  --freq-step <hz>      Frequency step in Hz (default: 0.05)" << std::endl;
    std::cout << "  --cycles <n>          Cycles per frequency point (default: 1.0)" << std::endl;
    std::cout << "  --offset <rad>        Position offset in rad (default: 0.0)" << std::endl;
    std::cout << "  --kp <value>          Position gain (default: 10.0)" << std::endl;
    std::cout << "  --kd <value>          Velocity gain (default: 1.5)" << std::endl;
    std::cout << "  --rate <hz>           Control rate in Hz (default: 500)" << std::endl;
    std::cout << "  --log-dir <path>      Log directory (default: log)" << std::endl;
    std::cout << "  --no-logging          Disable logging" << std::endl;
    std::cout << "  --no-enable           Don't auto-enable motor" << std::endl;
    std::cout << "  -h, --help            Show this help message" << std::endl;
    std::cout << "\nFrequency Sweep Mode:" << std::endl;
    std::cout << "  Discrete frequency sweep from --start-freq to --end-freq" << std::endl;
    std::cout << "  with step size --freq-step. Each frequency is held for --cycles" << std::endl;
    std::cout << "  complete cycles (duration = cycles / frequency)." << std::endl;
    std::cout << "\nExample:" << std::endl;
    std::cout << "  " << program_name << " --motor-id 9 --amplitude 0.3 --start-freq 0.5 --end-freq 5.0 --freq-step 0.1" << std::endl;
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
        } else if (arg == "--amplitude" && i + 1 < argc) {
            config.amplitude = std::atof(argv[++i]);
        } else if (arg == "--start-freq" && i + 1 < argc) {
            config.start_freq = std::atof(argv[++i]);
        } else if (arg == "--end-freq" && i + 1 < argc) {
            config.end_freq = std::atof(argv[++i]);
        } else if (arg == "--freq-step" && i + 1 < argc) {
            config.freq_step = std::atof(argv[++i]);
        } else if (arg == "--cycles" && i + 1 < argc) {
            config.cycles_per_freq = std::atof(argv[++i]);
        } else if (arg == "--offset" && i + 1 < argc) {
            config.offset = std::atof(argv[++i]);
        } else if (arg == "--kp" && i + 1 < argc) {
            config.kp = std::atof(argv[++i]);
        } else if (arg == "--kd" && i + 1 < argc) {
            config.kd = std::atof(argv[++i]);
        } else if (arg == "--rate" && i + 1 < argc) {
            config.control_rate = std::atof(argv[++i]);
        } else if (arg == "--log-dir" && i + 1 < argc) {
            config.log_dir = argv[++i];
        } else if (arg == "--no-logging") {
            config.enable_logging = false;
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

    // Validate frequency parameters
    if (config.start_freq <= 0 || config.end_freq <= 0 || config.start_freq >= config.end_freq) {
        std::cerr << "Error: Invalid frequency range (start_freq < end_freq, both > 0)" << std::endl;
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

    // Start receive thread
    controller.StartReceiveThread();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Run control loop
    controller.RunControlLoop();

    std::cout << "Exiting..." << std::endl;
    return 0;
}
