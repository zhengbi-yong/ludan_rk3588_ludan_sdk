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
#include <algorithm>
#include <cstdlib>

#include "CANFDNET.h"

// ==================== Config ====================
struct MotorTestConfig
{
    std::string zlg_ip = "192.168.1.5";
    int zlg_port = 8000; // 8000~8003
    int channel = 0;     // 会被固定映射覆盖：channel = zlg_port - 8000
    int arb_baud = 1000000;
    int data_baud = 5000000;

    int motor_id = 1;
    double kp = 10.0;
    double kd = 1.0;
    double torque = 0.0;

    double amplitude = 0.05;
    double start_freq = 1;
    double end_freq = 10;
    double freq_step = 0.1;
    double cycles_per_freq = 1.0;
    double min_hold = 2.0;
    double offset = 0.0;

    double control_rate = 200.0;
    bool enable_motor = true;

    std::string log_dir = "log";
    bool enable_logging = true;
};

// ==================== DM Motor Data ====================
struct DMMotorData
{
    uint8_t raw[8];

    uint8_t motor_id;
    uint8_t error;
    int16_t position;
    int16_t velocity;
    int16_t torque;
    int8_t temp_mos;
    int8_t temp_rotor;

    double position_rad;
    double velocity_rad_s;
    double torque_nm;
};

class DMMotorFrameDecoder
{
public:
    static constexpr double PMAX = 3.14159;
    static constexpr double PMIN = -3.14159;
    static constexpr double VMAX = 45.0;
    static constexpr double VMIN = -45.0;
    static constexpr double TMAX = 20.0;
    static constexpr double TMIN = -20.0;

    static DMMotorData DecodeFrameFD(const ZCAN_ReceiveFD_Data &frame)
    {
        DMMotorData data;
        std::memset(&data, 0, sizeof(data));

        const uint8_t *d = frame.frame.data;
        std::memcpy(data.raw, d, 8);

        data.motor_id = d[0] & 0x0F;
        data.error = (d[0] >> 4) & 0x0F;

        uint16_t pos_encoded = static_cast<uint16_t>((d[1] << 8) | d[2]);
        data.position = static_cast<int16_t>(pos_encoded);
        data.position_rad = (pos_encoded - 32768.0) / 65536.0 * (PMAX - PMIN);

        int16_t vel_encoded = ((d[3] & 0xFF) << 4) | (d[4] & 0x0F);
        data.velocity = vel_encoded;
        data.velocity_rad_s = (vel_encoded - 2048.0) / 4096.0 * (VMAX - VMIN);

        int16_t torque_encoded = (((d[4] >> 4) & 0x0F) << 8) | d[5];
        data.torque = torque_encoded;
        data.torque_nm = (torque_encoded - 2048.0) / 4096.0 * (TMAX - TMIN);

        data.temp_mos = static_cast<int8_t>(d[6]);
        data.temp_rotor = static_cast<int8_t>(d[7]);

        return data;
    }
};

// ==================== Feedback & Command ====================
struct MotorFeedback
{
    double timestamp;
    double position;
    double velocity;
    double torque;
    uint8_t motor_id;
    uint8_t error;
    int16_t raw_pos;
    int16_t raw_vel;
    int16_t raw_torque;
    int8_t temp_mos;
    int8_t temp_rotor;
    double frequency;
    bool valid;
};

struct MotorCommand
{
    uint16_t motor_id;
    float pos;
    float vel;
    float kp;
    float kd;
    float tau;
};

// 保持你原来的打包方式（不改）
inline void MotorCommandToCanData(const MotorCommand &cmd, uint8_t *can_data)
{
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
class MotorTestController
{
public:
    MotorTestController(const MotorTestConfig &config) : config_(config) {}

    bool Initialize()
    {
        std::cout << "=== Initializing Motor Test Controller ===\n";
        std::cout << "ZLG: " << config_.zlg_ip << ":" << config_.zlg_port << "\n";

        // ====== 固定映射：完全对齐 cantoudp.cpp ======
        // port 8000~8003 -> device_index 0~3 -> channel 0~3
        int device_index = config_.zlg_port - 8000;
        if (device_index < 0 || device_index > 3)
        {
            std::cerr << "[ERR] zlg_port must be 8000~8003, got " << config_.zlg_port << "\n";
            return false;
        }

        if (config_.channel != device_index)
        {
            std::cout << "[WARN] channel(" << config_.channel << ") mismatch port(" << config_.zlg_port
                      << "). Force channel=" << device_index << " (cantoudp mapping)\n";
            config_.channel = device_index;
        }

        std::cout << "Channel: CAN" << config_.channel << "\n";
        std::cout << "Motor ID: " << config_.motor_id << "\n";
        std::cout << "Control Rate: " << config_.control_rate << " Hz\n";

        // 1) OpenDevice（按映射的 device_index）
        device_handle_ = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, device_index, 0);
        if (device_handle_ == INVALID_DEVICE_HANDLE)
        {
            std::cerr << "Failed to open ZCAN device\n";
            return false;
        }
        std::cout << "[1/7] Device opened (device_index=" << device_index << ")\n";

        // 2) InitCAN（按映射后的 channel）
        ZCAN_CHANNEL_INIT_CONFIG init_config;
        std::memset(&init_config, 0, sizeof(init_config));
        init_config.can_type = TYPE_CANFD;
        init_config.canfd.acc_code = 0;
        init_config.canfd.acc_mask = 0;
        init_config.canfd.abit_timing = config_.arb_baud;
        init_config.canfd.dbit_timing = config_.data_baud;
        init_config.canfd.brp = 0;
        init_config.canfd.filter = 0;
        init_config.canfd.mode = 0;

        channel_handle_ = ZCAN_InitCAN(device_handle_, config_.channel, &init_config);
        if (channel_handle_ == INVALID_CHANNEL_HANDLE)
        {
            std::cerr << "Failed to initialize CAN channel\n";
            ZCAN_CloseDevice(device_handle_);
            return false;
        }
        std::cout << "[2/7] CAN channel initialized (CAN" << config_.channel << ")\n";

        // 3) SetReference IP/Port（按 cantoudp：对 (device_index, device_index) 设置）
        {
            uint32_t tcp_type = 0; // client
            ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, device_index, device_index, CMD_TCP_TYPE, &tcp_type);

            if (ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, device_index, device_index, CMD_DESIP,
                                  (void *)config_.zlg_ip.c_str()) != STATUS_OK)
            {
                std::cerr << "Failed to set target IP\n";
                ZCAN_CloseDevice(device_handle_);
                return false;
            }
            uint32_t port_val = (uint32_t)config_.zlg_port;
            if (ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, device_index, device_index, CMD_DESPORT, &port_val) != STATUS_OK)
            {
                std::cerr << "Failed to set target Port\n";
                ZCAN_CloseDevice(device_handle_);
                return false;
            }
            std::cout << "[3/7] TCP configured (IP:" << config_.zlg_ip << ", Port:" << config_.zlg_port << ")\n";
        }

        std::cout << "[4/7] Waiting for TCP connection...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "[4/7] TCP wait complete\n";

        // 4) StartCAN
        if (ZCAN_StartCAN(channel_handle_) != STATUS_OK)
        {
            std::cerr << "Failed to start CAN channel\n";
            ZCAN_CloseDevice(device_handle_);
            return false;
        }
        std::cout << "[5/7] CAN started\n";

        // 5) Logging
        if (config_.enable_logging)
        {
            SetupLogging();
            std::cout << "[6/7] Logging enabled: " << log_file_path_ << "\n";
        }
        else
        {
            std::cout << "[6/7] Logging disabled\n";
        }

        std::cout << "=========================================\n";

        // 6) Enable motor（保持原逻辑）
        if (config_.enable_motor)
        {
            EnableMotor();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "[7/7] Init done.\n";
        return true;
    }

    void EnableMotor()
    {
        uint8_t enable_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
        SendCanFrame(config_.motor_id, enable_frame, 8);
        std::cout << ">>> Motor " << config_.motor_id << " ENABLE sent <<<\n";
    }

    void DisableMotor()
    {
        uint8_t disable_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
        SendCanFrame(config_.motor_id, disable_frame, 8);
        std::cout << ">>> Motor " << config_.motor_id << " DISABLE sent <<<\n";
    }

    void StartReceiveThread()
    {
        receive_thread_running_ = true;
        receive_thread_ = std::thread(&MotorTestController::ReceiveLoop, this);
        std::cout << ">>> CAN Receive Thread Started (cantoudp style: ReceiveFD(channel)) <<<\n";
    }

    void RunControlLoop()
    {
        std::cout << "\n=== Starting Control Loop at " << config_.control_rate << " Hz ===\n";

        std::vector<double> freq_points;
        std::vector<double> freq_durations;
        double total_duration = 0.0;

        for (double f = config_.start_freq; f <= config_.end_freq + 1e-9; f += config_.freq_step)
        {
            freq_points.push_back(f);
            double duration = std::max(config_.min_hold, config_.cycles_per_freq / f);
            freq_durations.push_back(duration);
            total_duration += duration;
        }

        std::cout << "Number of frequency points: " << freq_points.size() << "\n";
        std::cout << "Estimated total duration: " << std::fixed << std::setprecision(1)
                  << total_duration << " s (" << total_duration / 60.0 << " min)\n";
        std::cout << "Press Ctrl+C to stop...\n";
        std::cout << std::fixed << std::setprecision(4);

        running_ = true;

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
        double phase_offset = 0.0;

        current_cmd_frequency_.store(current_freq);

        std::cout << "\n>>> Starting Frequency Point " << (freq_index + 1) << "/" << freq_points.size()
                  << ": " << current_freq << " Hz (duration: " << freq_durations[0] << " s) <<<\n";

        while (running_ && freq_index < freq_points.size())
        {
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(current_time - start_time).count();
            double freq_elapsed = elapsed - freq_start_time;

            if (freq_elapsed >= freq_durations[freq_index])
            {
                phase_offset += current_freq * freq_elapsed;
                freq_index++;
                if (freq_index >= freq_points.size())
                    break;

                current_freq = freq_points[freq_index];
                freq_start_time = elapsed;
                std::cout << "\n>>> Switching to Frequency Point " << (freq_index + 1) << "/"
                          << freq_points.size() << ": " << current_freq << " Hz (duration: "
                          << freq_durations[freq_index] << " s) <<<\n";
            }

            double phase = phase_offset + current_freq * (elapsed - freq_start_time);

            double target_pos = config_.amplitude * std::sin(2.0 * M_PI * phase) + config_.offset;
            double target_vel = config_.amplitude * 2.0 * M_PI * current_freq * std::cos(2.0 * M_PI * phase);

            MotorCommand cmd;
            cmd.motor_id = config_.motor_id;
            cmd.pos = static_cast<float>(target_pos);
            cmd.vel = static_cast<float>(target_vel);
            cmd.kp = static_cast<float>(config_.kp);
            cmd.kd = static_cast<float>(config_.kd);
            cmd.tau = static_cast<float>(config_.torque);

            SendMotorCommand(cmd);
            current_cmd_frequency_.store(current_freq);

            {
                std::lock_guard<std::mutex> lock(feedback_mutex_);
                if (latest_feedback_.valid)
                {
                    last_feedback = latest_feedback_;
                    feedback_count++;
                }
            }

            if (++print_counter >= 50)
            {
                double progress = (freq_elapsed / freq_durations[freq_index]) * 100.0;
                std::cout << "[" << std::setw(8) << elapsed << "s] "
                          << "Freq[" << (freq_index + 1) << "/" << freq_points.size() << "] "
                          << std::setw(6) << current_freq << " Hz "
                          << "(" << std::setw(5) << std::setprecision(1) << progress << "%), "
                          << "pos=" << std::setw(8) << target_pos << " rad, "
                          << "vel=" << std::setw(8) << target_vel << " rad/s";

                if (last_feedback.valid)
                {
                    std::cout << " | act_pos=" << std::setw(8) << last_feedback.position
                              << ", act_vel=" << std::setw(8) << last_feedback.velocity
                              << ", err=" << (int)last_feedback.error;
                }
                std::cout << std::setprecision(4) << ", iter=" << iteration
                          << ", rx_total=" << rx_total_.load()
                          << ", rx_target=" << rx_target_.load()
                          << "\n";
                std::cout << std::flush;
                print_counter = 0;
            }

            iteration++;
            next_time += control_period;
            std::this_thread::sleep_until(next_time);
        }

        std::cout << "\n=== Control Loop Stopped ===\n";
        std::cout << "Total iterations: " << iteration << "\n";
        std::cout << "Total feedback received: " << feedback_count << "\n";
        std::cout << "Total rx frames: " << rx_total_.load() << "\n";
        std::cout << "Target rx frames: " << rx_target_.load() << "\n";
        std::cout << "Log records: " << log_count_ << "\n";
        std::cout << "Frequency points completed: " << freq_index << "/" << freq_points.size() << "\n";
    }

    void Stop()
    {
        running_ = false;
        receive_thread_running_ = false;
        if (channel_handle_)
        {
            ZCAN_ResetCAN(channel_handle_);
        }
    }

    ~MotorTestController()
    {
        if (receive_thread_.joinable())
            receive_thread_.join();

        DisableMotor();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (config_.enable_logging && log_file_.is_open())
        {
            log_file_.flush();
            log_file_.close();
            std::cout << ">>> Log file saved: " << log_file_path_ << "\n";
            std::cout << "    Total records: " << log_count_ << "\n";
        }

        if (channel_handle_)
            ZCAN_ResetCAN(channel_handle_);
        if (device_handle_)
            ZCAN_CloseDevice(device_handle_);
    }

private:
    void SetupLogging()
    {
        std::string mkdir_cmd = "mkdir -p " + config_.log_dir;
        int ret = system(mkdir_cmd.c_str());
        std::cout << "[LOG] mkdir -p " << config_.log_dir << " returned: " << ret << "\n";

        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << config_.log_dir << "/motor_log_" << config_.motor_id << "_"
           << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S") << ".csv";
        log_file_path_ = ss.str();

        log_file_.open(log_file_path_);
        if (log_file_.is_open())
        {
            log_file_ << "timestamp,motor_id,error,position,velocity,torque,raw_pos,raw_vel,raw_torque,temp_mos,temp_rotor,frequency\n";
            log_file_ << std::fixed << std::setprecision(6);
            log_file_.flush();
            log_count_ = 0;
            std::cout << "[LOG] Log file opened successfully: " << log_file_path_ << "\n";
        }
        else
        {
            std::cerr << "[LOG ERROR] Failed to open log file: " << log_file_path_ << "\n";
        }
    }

    void HandleOneFrame(uint32_t can_id_full, const uint8_t *data, uint8_t len, double timestamp)
    {
         if (len < 8) return;

    // 用payload判断电机ID（data[0]低4位）
    int payload_motor_id = data[0] & 0x0F;
    if (payload_motor_id != config_.motor_id) return;

        ZCAN_ReceiveFD_Data fake;
        std::memset(&fake, 0, sizeof(fake));
        fake.frame.can_id = can_id_full;
        fake.frame.len = len;

        // CANFD data 최대 64 字节，安全拷贝
        int copy_len = (len > 64) ? 64 : (int)len;
        std::memcpy(fake.frame.data, data, copy_len);

        DMMotorData dm = DMMotorFrameDecoder::DecodeFrameFD(fake);

        MotorFeedback fb;
        fb.timestamp = timestamp;
        fb.motor_id = dm.motor_id;
        fb.error = dm.error;
        fb.position = dm.position_rad;
        fb.velocity = dm.velocity_rad_s;
        fb.torque = dm.torque_nm;
        fb.raw_pos = dm.position;
        fb.raw_vel = dm.velocity;
        fb.raw_torque = dm.torque;
        fb.temp_mos = dm.temp_mos;
        fb.temp_rotor = dm.temp_rotor;
        fb.frequency = current_cmd_frequency_.load();
        fb.valid = true;

        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            latest_feedback_ = fb;
        }

        rx_target_.fetch_add(1);

        if (config_.enable_logging && log_file_.is_open())
        {
            std::lock_guard<std::mutex> lock(log_mutex_);
            log_file_ << fb.timestamp << ","
                      << (int)fb.motor_id << ","
                      << (int)fb.error << ","
                      << fb.position << ","
                      << fb.velocity << ","
                      << fb.torque << ","
                      << fb.raw_pos << ","
                      << fb.raw_vel << ","
                      << fb.raw_torque << ","
                      << (int)fb.temp_mos << ","
                      << (int)fb.temp_rotor << ","
                      << fb.frequency << "\n";
            log_count_++;
            if (log_count_ % 10 == 0)
                log_file_.flush();
        }
    }

    // ===== 接收：完全按 cantoudp 的方式：ZCAN_ReceiveFD(channel_handle,...) =====
    void ReceiveLoop()
    {
        std::cout << ">>> Receive Loop Started (cantoudp style) <<<\n";
        std::cout << "[RECV] Using: ZCAN_ReceiveFD(channel)\n";
        std::cout << "[RECV] Will print first 20 RAW frames for debug\n";

        ZCAN_ReceiveFD_Data buf[200];
        uint64_t printed = 0;

        while (receive_thread_running_)
        {
            if (!channel_handle_ || channel_handle_ == INVALID_CHANNEL_HANDLE)
                break;

            uint32_t n = ZCAN_ReceiveFD(channel_handle_, buf, 200, 10);
            if (n == 0)
                continue;

            rx_total_.fetch_add(n);

            auto now = std::chrono::steady_clock::now();
            double ts = std::chrono::duration<double>(now.time_since_epoch()).count();

            for (uint32_t i = 0; i < n; i++)
            {
                // 打印 RAW（前 20 帧，不影响其它功能）
                if (printed < 20)
                {
                    uint32_t id_full = buf[i].frame.can_id;
                    uint32_t id_std = id_full & 0x7FF;
                    uint8_t len = buf[i].frame.len;

                    std::cout << "[RAW] full_id=0x" << std::hex << std::setw(8) << std::setfill('0') << id_full
                              << " std_id=0x" << std::setw(3) << id_std
                              << std::dec << " len=" << (int)len << " data=";

                    int show = (len > 8) ? 8 : (int)len;
                    for (int j = 0; j < show; j++)
                    {
                        std::cout << std::hex << std::setw(2) << std::setfill('0')
                                  << (int)buf[i].frame.data[j] << " ";
                    }
                    std::cout << std::dec << "\n";
                    printed++;
                }

                HandleOneFrame(buf[i].frame.can_id, buf[i].frame.data, buf[i].frame.len, ts);
            }
        }

        std::cout << ">>> Receive Thread Stopped (rx_total=" << rx_total_.load()
                  << ", rx_target=" << rx_target_.load()
                  << ", log_records=" << log_count_ << ") <<<\n";
    }

    void SendMotorCommand(const MotorCommand &cmd)
    {
        uint8_t can_data[8];
        MotorCommandToCanData(cmd, can_data);
        SendCanFrame(cmd.motor_id, can_data, 8);
    }

    void SendCanFrame(uint16_t motor_id, const uint8_t *data, uint8_t len)
    {
        ZCAN_TransmitFD_Data tx;
        std::memset(&tx, 0, sizeof(tx));
        tx.frame.can_id = motor_id;
        tx.frame.len = len;
        std::memcpy(tx.frame.data, data, len);
        tx.transmit_type = 0;
        ZCAN_TransmitFD(channel_handle_, &tx, 1);
    }

private:
    MotorTestConfig config_;
    DEVICE_HANDLE device_handle_ = nullptr;
    CHANNEL_HANDLE channel_handle_ = nullptr;

    std::atomic<bool> running_{false};
    std::atomic<bool> receive_thread_running_{false};
    std::thread receive_thread_;

    MotorFeedback latest_feedback_{0};
    std::mutex feedback_mutex_;
    std::atomic<double> current_cmd_frequency_{0.0};

    std::ofstream log_file_;
    std::string log_file_path_;
    std::mutex log_mutex_;
    uint64_t log_count_ = 0;

    std::atomic<uint64_t> rx_total_{0};
    std::atomic<uint64_t> rx_target_{0};
};

// ==================== Signal Handler ====================
MotorTestController *g_controller = nullptr;

void SignalHandler(int signal)
{
    if (g_controller)
    {
        std::cout << "\nReceived signal " << signal << ", stopping...\n";
        g_controller->Stop();
    }
}

void PrintUsage(const char *program_name)
{
    std::cout << "Usage: " << program_name << " [options]\n"
              << "  --ip <address>\n"
              << "  --port <8000~8003>\n"
              << "  --channel <num> (will be forced to port-8000)\n"
              << "  --motor-id <id>\n"
              << "  --amplitude <rad>\n"
              << "  --start-freq <hz>\n"
              << "  --end-freq <hz>\n"
              << "  --freq-step <hz>\n"
              << "  --hold-time <sec>\n"
              << "  --offset <rad>\n"
              << "  --kp <v>\n"
              << "  --kd <v>\n"
              << "  --rate <hz>\n"
              << "  --log-dir <path>\n"
              << "  --no-logging\n"
              << "  --no-enable\n";
}

int main(int argc, char **argv)
{
    MotorTestConfig config;

    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];

        if (arg == "-h" || arg == "--help")
        {
            PrintUsage(argv[0]);
            return 0;
        }
        else if (arg == "--ip" && i + 1 < argc)
            config.zlg_ip = argv[++i];
        else if (arg == "--port" && i + 1 < argc)
            config.zlg_port = std::atoi(argv[++i]);
        else if (arg == "--channel" && i + 1 < argc)
            config.channel = std::atoi(argv[++i]); // 允许传，但会被固定映射覆盖
        else if (arg == "--motor-id" && i + 1 < argc)
            config.motor_id = std::atoi(argv[++i]);
        else if (arg == "--amplitude" && i + 1 < argc)
            config.amplitude = std::atof(argv[++i]);
        else if (arg == "--start-freq" && i + 1 < argc)
            config.start_freq = std::atof(argv[++i]);
        else if (arg == "--end-freq" && i + 1 < argc)
            config.end_freq = std::atof(argv[++i]);
        else if (arg == "--freq-step" && i + 1 < argc)
            config.freq_step = std::atof(argv[++i]);
        else if (arg == "--hold-time" && i + 1 < argc)
            config.min_hold = std::atof(argv[++i]);
        else if (arg == "--offset" && i + 1 < argc)
            config.offset = std::atof(argv[++i]);
        else if (arg == "--kp" && i + 1 < argc)
            config.kp = std::atof(argv[++i]);
        else if (arg == "--kd" && i + 1 < argc)
            config.kd = std::atof(argv[++i]);
        else if (arg == "--rate" && i + 1 < argc)
            config.control_rate = std::atof(argv[++i]);
        else if (arg == "--log-dir" && i + 1 < argc)
            config.log_dir = argv[++i];
        else if (arg == "--no-logging")
            config.enable_logging = false;
        else if (arg == "--no-enable")
            config.enable_motor = false;
        else
        {
            std::cerr << "Unknown option: " << arg << "\n";
            PrintUsage(argv[0]);
            return 1;
        }
    }

    if (config.motor_id < 1 || config.motor_id > 30)
    {
        std::cerr << "Error: Motor ID must be between 1 and 30\n";
        return 1;
    }

    if (config.start_freq <= 0 || config.end_freq <= 0 || config.start_freq >= config.end_freq)
    {
        std::cerr << "Error: Invalid frequency range\n";
        return 1;
    }

    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    MotorTestController controller(config);
    g_controller = &controller;

    if (!controller.Initialize())
    {
        std::cerr << "Failed to initialize controller\n";
        return 1;
    }

    controller.StartReceiveThread();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    controller.RunControlLoop();

    std::cout << "Exiting...\n";
    return 0;
}
