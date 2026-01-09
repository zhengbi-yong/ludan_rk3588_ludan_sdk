// can_send_test.cpp
// Sends simulated DM motor CAN data to shared memory
// can_motor_feedback_publisher.cpp reads from shared memory (test mode) or ZLG device (normal mode)
//
// Usage: ./can_send_test [options]
//   --config <file>   JSON configuration file
//   --interval <n>    Send interval in ms (default: 100)
//   --quiet           Don't print each frame
//   -h, --help        Show this help message

#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>
#include <iomanip>
#include <vector>
#include <atomic>
#include <signal.h>
#include <fstream>
#include <cmath>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "cJSON.h"
#include "../include/can_shm.h"

// Port configuration
struct PortConfig {
    int motor_count;         // Number of motors (10,7,7,6)
    int motor_offset;        // Starting motor number (0,10,17,24)
};

constexpr int NUM_PORTS = 4;
constexpr int NUM_MOTORS = 30;

constexpr PortConfig PORT_CONFIGS[NUM_PORTS] = {
    {10, 0},   // Motor 1-10  -> array index 0-9
    {7, 10},   // Motor 11-17 -> array index 10-16
    {7, 17},   // Motor 18-24 -> array index 17-23
    {6, 24}    // Motor 25-30 -> array index 24-29
};

// Error code definitions
enum ErrorCode : uint8_t {
    ERR_DISABLED = 0x0,    // 失能
    ERR_ENABLED = 0x1,     // 使能
    ERR_OVERVOLTAGE = 0x8, // 超压
    ERR_UNDERVOLTAGE = 0x9,// 欠压
    ERR_OVERCURRENT = 0xA, // 过电流
    ERR_MOS_OVERTEMP = 0xB,// MOS过温
    ERR_COIL_OVERTEMP = 0xC,// 电机线圈过温
    ERR_COMM_LOST = 0xD,   // 通讯丢失
    ERR_OVERLOAD = 0xE     // 过载
};

// Motor data configuration
struct MotorDataConfig {
    int motor_id;
    bool enabled;
    float position;         // Position (range depends on P_MAX)
    float velocity;         // Velocity in rad/s (range depends on V_MAX)
    float torque;           // Torque in Nm (range depends on T_MAX)
    float temp_mos;         // MOS temperature in Celsius (0-125)
    float temp_rotor;       // Rotor temperature in Celsius (0-125)
    uint8_t error_code;     // Error code (see ErrorCode enum)
    float pos_increment;    // Position increment per frame (for animation)
    float vel_amplitude;    // Velocity amplitude (for sine wave)
    float vel_frequency;    // Velocity frequency in Hz
    int data_mode;          // 0=fixed, 1=increment, 2=sine, 3=custom

    // Range parameters (can be configured via JSON)
    float p_max;            // Position maximum range (default: 2*PI for radians)
    float p_min;            // Position minimum range (default: -2*PI for radians)
    float v_max;            // Velocity maximum range in rad/s (default: 45.0)
    float v_min;            // Velocity minimum range in rad/s (default: -45.0)
    float t_max;            // Torque maximum range in Nm (default: 18.0)
    float t_min;            // Torque minimum range in Nm (default: -18.0)
};

// Global configuration
struct GlobalConfig {
    int send_interval_ms;
    bool verbose;
    std::vector<MotorDataConfig> motor_configs;

    GlobalConfig() : send_interval_ms(100), verbose(true) {}
};

// Global flag for signal handler
std::atomic<bool> g_running(true);

void SignalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", stopping..." << std::endl;
    g_running = false;
}

// Convert float to 16-bit unsigned integer (position)
int16_t FloatToInt16(float value, float max_range) {
    float result = (value / (2.0f * max_range)) * 65536.0f + 32768.0f;
    if (result > 65535.0f) result = 65535.0f;
    if (result < 0.0f) result = 0.0f;
    return static_cast<int16_t>(result);
}

// Convert float to 12-bit unsigned integer (velocity/torque)
int16_t FloatToInt12(float value, float max_range) {
    float result = (value / (2.0f * max_range)) * 4096.0f + 2048.0f;
    if (result > 4095.0f) result = 4095.0f;
    if (result < 0.0f) result = 0.0f;
    return static_cast<int16_t>(result);
}

// Load JSON configuration file
bool LoadJsonConfig(const char* filepath, GlobalConfig& config) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open config file: " << filepath << std::endl;
        return false;
    }

    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();

    cJSON* root = cJSON_Parse(content.c_str());
    if (root == nullptr) {
        std::cerr << "Failed to parse JSON config" << std::endl;
        return false;
    }

    // Parse global settings
    cJSON* interval = cJSON_GetObjectItem(root, "send_interval_ms");
    if (interval && cJSON_IsNumber(interval)) {
        config.send_interval_ms = interval->valueint;
    }

    cJSON* verbose = cJSON_GetObjectItem(root, "verbose");
    if (verbose && cJSON_IsBool(verbose)) {
        config.verbose = cJSON_IsTrue(verbose);
    }

    // Parse default range parameters
    float default_p_max = 2.0f * M_PI;
    float default_p_min = -2.0f * M_PI;
    float default_v_max = 45.0f;
    float default_v_min = -45.0f;
    float default_t_max = 18.0f;
    float default_t_min = -18.0f;

    cJSON* p_max = cJSON_GetObjectItem(root, "p_max");
    if (p_max && cJSON_IsNumber(p_max)) {
        default_p_max = static_cast<float>(p_max->valuedouble);
    }

    cJSON* p_min = cJSON_GetObjectItem(root, "p_min");
    if (p_min && cJSON_IsNumber(p_min)) {
        default_p_min = static_cast<float>(p_min->valuedouble);
    }

    cJSON* v_max = cJSON_GetObjectItem(root, "v_max");
    if (v_max && cJSON_IsNumber(v_max)) {
        default_v_max = static_cast<float>(v_max->valuedouble);
    }

    cJSON* v_min = cJSON_GetObjectItem(root, "v_min");
    if (v_min && cJSON_IsNumber(v_min)) {
        default_v_min = static_cast<float>(v_min->valuedouble);
    }

    cJSON* t_max = cJSON_GetObjectItem(root, "t_max");
    if (t_max && cJSON_IsNumber(t_max)) {
        default_t_max = static_cast<float>(t_max->valuedouble);
    }

    cJSON* t_min = cJSON_GetObjectItem(root, "t_min");
    if (t_min && cJSON_IsNumber(t_min)) {
        default_t_min = static_cast<float>(t_min->valuedouble);
    }

    // Parse motor configurations
    cJSON* motors = cJSON_GetObjectItem(root, "motors");
    if (motors && cJSON_IsArray(motors)) {
        cJSON* motor = nullptr;
        cJSON_ArrayForEach(motor, motors) {
            MotorDataConfig motor_config;
            memset(&motor_config, 0, sizeof(motor_config));

            motor_config.p_max = default_p_max;
            motor_config.p_min = default_p_min;
            motor_config.v_max = default_v_max;
            motor_config.v_min = default_v_min;
            motor_config.t_max = default_t_max;
            motor_config.t_min = default_t_min;

            cJSON* id = cJSON_GetObjectItem(motor, "id");
            if (id && cJSON_IsNumber(id)) {
                motor_config.motor_id = id->valueint;
            }

            cJSON* enabled = cJSON_GetObjectItem(motor, "enabled");
            if (enabled && cJSON_IsBool(enabled)) {
                motor_config.enabled = cJSON_IsTrue(enabled);
            } else {
                motor_config.enabled = true;
            }

            cJSON* position = cJSON_GetObjectItem(motor, "position");
            if (position && cJSON_IsNumber(position)) {
                motor_config.position = static_cast<float>(position->valuedouble);
            }

            cJSON* velocity = cJSON_GetObjectItem(motor, "velocity");
            if (velocity && cJSON_IsNumber(velocity)) {
                motor_config.velocity = static_cast<float>(velocity->valuedouble);
            }

            cJSON* torque = cJSON_GetObjectItem(motor, "torque");
            if (torque && cJSON_IsNumber(torque)) {
                motor_config.torque = static_cast<float>(torque->valuedouble);
            }

            cJSON* temp_mos = cJSON_GetObjectItem(motor, "temp_mos");
            if (temp_mos && cJSON_IsNumber(temp_mos)) {
                motor_config.temp_mos = static_cast<float>(temp_mos->valuedouble);
            } else {
                motor_config.temp_mos = 25.0f;
            }

            cJSON* temp_rotor = cJSON_GetObjectItem(motor, "temp_rotor");
            if (temp_rotor && cJSON_IsNumber(temp_rotor)) {
                motor_config.temp_rotor = static_cast<float>(temp_rotor->valuedouble);
            } else {
                motor_config.temp_rotor = 25.0f;
            }

            cJSON* error = cJSON_GetObjectItem(motor, "error_code");
            if (error && cJSON_IsNumber(error)) {
                motor_config.error_code = static_cast<uint8_t>(error->valueint);
            } else {
                motor_config.error_code = ERR_ENABLED;
            }

            cJSON* pos_inc = cJSON_GetObjectItem(motor, "pos_increment");
            if (pos_inc && cJSON_IsNumber(pos_inc)) {
                motor_config.pos_increment = static_cast<float>(pos_inc->valuedouble);
            }

            cJSON* vel_amp = cJSON_GetObjectItem(motor, "vel_amplitude");
            if (vel_amp && cJSON_IsNumber(vel_amp)) {
                motor_config.vel_amplitude = static_cast<float>(vel_amp->valuedouble);
            }

            cJSON* vel_freq = cJSON_GetObjectItem(motor, "vel_frequency");
            if (vel_freq && cJSON_IsNumber(vel_freq)) {
                motor_config.vel_frequency = static_cast<float>(vel_freq->valuedouble);
            }

            cJSON* mode = cJSON_GetObjectItem(motor, "data_mode");
            if (mode && cJSON_IsNumber(mode)) {
                motor_config.data_mode = mode->valueint;
            } else {
                motor_config.data_mode = 0;
            }

            cJSON* p_max_motor = cJSON_GetObjectItem(motor, "p_max");
            if (p_max_motor && cJSON_IsNumber(p_max_motor)) {
                motor_config.p_max = static_cast<float>(p_max_motor->valuedouble);
            }

            cJSON* p_min_motor = cJSON_GetObjectItem(motor, "p_min");
            if (p_min_motor && cJSON_IsNumber(p_min_motor)) {
                motor_config.p_min = static_cast<float>(p_min_motor->valuedouble);
            }

            cJSON* v_max_motor = cJSON_GetObjectItem(motor, "v_max");
            if (v_max_motor && cJSON_IsNumber(v_max_motor)) {
                motor_config.v_max = static_cast<float>(v_max_motor->valuedouble);
            }

            cJSON* v_min_motor = cJSON_GetObjectItem(motor, "v_min");
            if (v_min_motor && cJSON_IsNumber(v_min_motor)) {
                motor_config.v_min = static_cast<float>(v_min_motor->valuedouble);
            }

            cJSON* t_max_motor = cJSON_GetObjectItem(motor, "t_max");
            if (t_max_motor && cJSON_IsNumber(t_max_motor)) {
                motor_config.t_max = static_cast<float>(t_max_motor->valuedouble);
            }

            cJSON* t_min_motor = cJSON_GetObjectItem(motor, "t_min");
            if (t_min_motor && cJSON_IsNumber(t_min_motor)) {
                motor_config.t_min = static_cast<float>(t_min_motor->valuedouble);
            }

            config.motor_configs.push_back(motor_config);
        }
    }

    cJSON_Delete(root);
    return true;
}

// Shared memory manager
class SharedMemoryManager {
public:
    SharedMemoryManager() : shm_fd_(-1), shm_ptr_(nullptr) {}

    ~SharedMemoryManager() {
        Cleanup();
    }

    bool Initialize() {
        // Remove existing shared memory
        shm_unlink(SHM_NAME);

        shm_fd_ = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (shm_fd_ == -1) {
            std::cerr << "Failed to create shared memory: " << strerror(errno) << std::endl;
            return false;
        }

        if (ftruncate(shm_fd_, SHM_SIZE) == -1) {
            std::cerr << "Failed to set shared memory size: " << strerror(errno) << std::endl;
            close(shm_fd_);
            shm_unlink(SHM_NAME);
            return false;
        }

        shm_ptr_ = mmap(nullptr, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
        if (shm_ptr_ == MAP_FAILED) {
            std::cerr << "Failed to map shared memory: " << strerror(errno) << std::endl;
            close(shm_fd_);
            shm_unlink(SHM_NAME);
            return false;
        }

        // Initialize shared memory
        new (shm_ptr_) ShmLayout();

        std::cout << "[Shared Memory] Created: " << SHM_NAME
                  << " (size: " << SHM_SIZE << " bytes)" << std::endl;
        return true;
    }

    ShmLayout* GetData() {
        return static_cast<ShmLayout*>(shm_ptr_);
    }

    void Cleanup() {
        if (shm_ptr_ != nullptr && shm_ptr_ != MAP_FAILED) {
            munmap(shm_ptr_, SHM_SIZE);
            shm_ptr_ = nullptr;
        }

        if (shm_fd_ != -1) {
            close(shm_fd_);
            shm_fd_ = -1;
        }

        shm_unlink(SHM_NAME);
    }

private:
    int shm_fd_;
    void* shm_ptr_;
};

// Generate motor CAN frame data
void GenerateMotorFrame(ShmCANFrame& frame, const MotorDataConfig& config, double elapsed, uint64_t frame_counter) {
    float position = config.position;
    float velocity = config.velocity;
    float torque = config.torque;
    float temp_mos = config.temp_mos;
    float temp_rotor = config.temp_rotor;
    uint8_t error = config.error_code;

    // Apply data mode
    switch (config.data_mode) {
        case 0:  // Fixed value
            break;

        case 1:  // Increment position
            position += config.pos_increment * frame_counter;
            break;

        case 2:  // Sine wave velocity
            if (config.vel_amplitude > 0 && config.vel_frequency > 0) {
                velocity = config.vel_amplitude * std::sin(2.0 * M_PI * config.vel_frequency * elapsed);
                position = config.position +
                          (config.vel_amplitude / (2.0 * M_PI * config.vel_frequency)) *
                          (1.0 - std::cos(2.0 * M_PI * config.vel_frequency * elapsed));
            }
            break;

        case 3:  // Custom - combination
            position += config.pos_increment * frame_counter;
            if (config.vel_amplitude > 0 && config.vel_frequency > 0) {
                velocity += config.vel_amplitude * std::sin(2.0 * M_PI * config.vel_frequency * elapsed);
            }
            break;

        default:
            break;
    }

    // Set CAN ID = motor ID
    frame.can_id = config.motor_id;
    frame.len = 8;
    frame.flags = 0;
    frame.valid = true;
    frame.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    // Convert to CAN frame format
    // D[0]: ID | ERR<<4
    uint8_t id = config.motor_id & 0x0F;
    frame.data[0] = id | (error << 4);

    // D[1]: POS[15:8], D[2]: POS[7:0]
    int16_t pos_int = FloatToInt16(position, config.p_max);
    frame.data[1] = (pos_int >> 8) & 0xFF;
    frame.data[2] = pos_int & 0xFF;

    // D[3]: VEL[11:4], D[4]: VEL[3:0] | T[11:8]
    int16_t vel_int = FloatToInt12(velocity, config.v_max);
    int16_t tor_int = FloatToInt12(torque, config.t_max);

    frame.data[3] = (vel_int >> 4) & 0xFF;
    frame.data[4] = ((vel_int & 0x0F) << 4) | ((tor_int >> 8) & 0x0F);
    frame.data[5] = tor_int & 0xFF;

    // Debug: print encoding for motor 7
    if (config.motor_id == 7 && frame_counter <= 3) {
        std::cout << "[DEBUG Motor 7] velocity=" << velocity << " -> vel_int=" << vel_int
                  << " (0x" << std::hex << vel_int << std::dec << ")" << std::endl;
        std::cout << "[DEBUG Motor 7] torque=" << torque << " -> tor_int=" << tor_int
                  << " (0x" << std::hex << tor_int << std::dec << ")" << std::endl;
        std::cout << "[DEBUG Motor 7] CAN data: ";
        for (int i = 0; i < 8; i++) {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)frame.data[i] << " " << std::dec;
        }
        std::cout << std::endl;
    }

    // D[6]: T_MOS
    frame.data[6] = static_cast<uint8_t>(std::max(0.0f, std::min(125.0f, temp_mos)));

    // D[7]: T_Rotor
    frame.data[7] = static_cast<uint8_t>(std::max(0.0f, std::min(125.0f, temp_rotor)));
}

void PrintUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options]\n";
    std::cout << "Sends simulated DM motor CAN data to shared memory\n\n";
    std::cout << "Options:\n";
    std::cout << "  --config <file>   JSON configuration file\n";
    std::cout << "  --interval <n>    Send interval in ms (default: 100)\n";
    std::cout << "  --quiet           Don't print each frame\n";
    std::cout << "  -h, --help        Show this help message\n";
    std::cout << "\nShared Memory:\n";
    std::cout << "  Name: " << SHM_NAME << "\n";
    std::cout << "  Size: " << SHM_SIZE << " bytes\n";
    std::cout << "\nTotal motors: " << NUM_MOTORS << " (10+7+7+6)\n";
}

int main(int argc, char** argv) {
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    const char* config_file = nullptr;
    int send_interval_ms = -1;
    bool verbose = true;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_file = argv[++i];
        } else if (arg == "--interval" && i + 1 < argc) {
            send_interval_ms = std::atoi(argv[++i]);
        } else if (arg == "--quiet") {
            verbose = false;
        } else if (arg == "-h" || arg == "--help") {
            PrintUsage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            PrintUsage(argv[0]);
            return 1;
        }
    }

    GlobalConfig config;
    if (config_file != nullptr) {
        if (!LoadJsonConfig(config_file, config)) {
            std::cerr << "[ERROR] Failed to load config file: " << config_file << std::endl;
            return 1;
        }
        std::cout << "Loaded config from: " << config_file << std::endl;
        std::cout << "  Motor configs: " << config.motor_configs.size() << std::endl;
    }

    if (send_interval_ms > 0) {
        config.send_interval_ms = send_interval_ms;
    }
    if (!verbose) {
        config.verbose = false;
    }

    std::cout << "=== CAN Motor Send Test (Shared Memory) ===" << std::endl;
    std::cout << "Shared Memory: " << SHM_NAME << std::endl;
    std::cout << "Total motors: " << NUM_MOTORS << std::endl;
    std::cout << "Send interval: " << config.send_interval_ms << "ms" << std::endl;
    std::cout << "===========================================" << std::endl;

    // Initialize shared memory
    SharedMemoryManager shm_manager;
    if (!shm_manager.Initialize()) {
        std::cerr << "[ERROR] Failed to initialize shared memory" << std::endl;
        return 1;
    }

    auto start_time = std::chrono::steady_clock::now();
    uint64_t frame_counter = 0;

    std::cout << "\n[Send] Starting motor data generation (Ctrl+C to stop)..." << std::endl;

    while (g_running) {
        frame_counter++;

        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time).count();

        ShmLayout* shm_data = shm_manager.GetData();
        if (shm_data) {
            // Clear previous data
            shm_data->Clear();

            // Generate frames for all motors
            int frame_idx = 0;
            for (int motor_id = 1; motor_id <= NUM_MOTORS; motor_id++) {
                if (frame_idx >= ShmLayout::MAX_FRAMES) break;

                // Find motor config
                MotorDataConfig* motor_config = nullptr;
                for (auto& cfg : config.motor_configs) {
                    if (cfg.motor_id == motor_id) {
                        motor_config = &cfg;
                        break;
                    }
                }

                // Generate frame data
                if (motor_config && motor_config->enabled) {
                    GenerateMotorFrame(shm_data->frames[frame_idx], *motor_config, elapsed, frame_counter);
                    frame_idx++;
                } else {
                    // Default data if motor disabled or not configured
                    shm_data->frames[frame_idx].can_id = motor_id;
                    shm_data->frames[frame_idx].len = 8;
                    shm_data->frames[frame_idx].flags = 0;
                    shm_data->frames[frame_idx].valid = true;
                    shm_data->frames[frame_idx].timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();

                    uint8_t id = motor_id & 0x0F;
                    shm_data->frames[frame_idx].data[0] = id | (ERR_ENABLED << 4);
                    memset(&shm_data->frames[frame_idx].data[1], 0, 7);
                    frame_idx++;
                }
            }

            shm_data->frame_count = frame_idx;
            shm_data->sequence++;
            shm_data->data_ready = true;

            // Print frame data if verbose
            if (verbose && frame_counter <= 5) {
                std::cout << "[TX] Sequence " << shm_data->sequence
                          << " | Frames: " << frame_idx << std::endl;
                for (uint32_t i = 0; i < frame_idx && i < 3; i++) {
                    const ShmCANFrame& frame = shm_data->frames[i];
                    std::cout << "  Motor " << std::setw(2) << frame.can_id
                              << " Data: ";
                    for (int j = 0; j < frame.len; j++) {
                        std::cout << std::hex << std::setfill('0') << std::setw(2)
                                  << (int)frame.data[j] << " " << std::dec;
                    }
                    std::cout << std::endl;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(config.send_interval_ms));

        // Print status every 5 seconds
        if (verbose && frame_counter % (5000 / config.send_interval_ms) == 0) {
            ShmLayout* shm_data = shm_manager.GetData();
            std::cout << "\n=== Status ===" << std::endl;
            std::cout << "  Frames written: " << (shm_data ? shm_data->sequence : 0) << std::endl;
            std::cout << "  Motors active: " << NUM_MOTORS << std::endl;
            std::cout << "===============" << std::endl;
        }
    }

    std::cout << "\nCleaning up shared memory..." << std::endl;
    shm_manager.Cleanup();
    std::cout << "Done!" << std::endl;

    return 0;
}
