// can_motor_feedback_publisher.cpp
// ROS2 node that receives CAN frames from ZLG device (4 ports) and publishes motor feedback
// 30 motors total: 10+7+7+6 distribution
// Uses LowState message format with MotorState[30] array
// Architecture based on cantoudp.cpp: Manager opens devices, passes handles to PortReceivers
//
// TEST MODE: Use --test-mode to read from shared memory instead of ZLG device
// Normal mode: reads from ZLG device
// Test mode: reads from /can_motor_test_shm (written by can_send_test)

#include <rclcpp/rclcpp.hpp>
#include <multi_port_motor_feedback/msg/low_state.hpp>
#include <multi_port_motor_feedback/msg/motor_state.hpp>
#include <chrono>
#include <thread>
#include <cstring>
#include <iomanip>
#include <signal.h>
#include <cmath>
#include <vector>
#include <memory>
#include <atomic>
#include <mutex>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

// ZLG CANFDNET SDK
#include "CANFDNET.h"

// ==================== Shared Memory Structures (for test mode) ====================
struct ShmCANFrame {
    uint32_t can_id;
    uint8_t  data[8];
    uint8_t  len;
    uint32_t flags;
    uint64_t timestamp;
    bool     valid;

    ShmCANFrame() : can_id(0), len(8), flags(0), timestamp(0), valid(false) {
        memset(data, 0, sizeof(data));
    }
};

struct ShmLayout {
    static constexpr int MAX_FRAMES = 100;

    ShmCANFrame frames[MAX_FRAMES];
    uint32_t    frame_count;
    uint64_t    sequence;
    bool        data_ready;

    ShmLayout() : frame_count(0), sequence(0), data_ready(false) {
        for (int i = 0; i < MAX_FRAMES; i++) {
            frames[i].valid = false;
        }
    }
};

constexpr const char* SHM_NAME = "/can_motor_test_shm";
constexpr size_t SHM_SIZE = sizeof(ShmLayout);

// ==================== Configuration ====================
constexpr int NUM_MOTORS = 30;
constexpr int DATA_BYTES_PER_MOTOR = 8;  // DM motors use 8 bytes
constexpr int NUM_PORTS = 4;

// Motor distribution per port (matches 10+7+7+6)
constexpr int MOTOR_COUNTS[NUM_PORTS] = {10, 7, 7, 6};

// Port configuration - simplified like cantoudp.cpp
struct PortConfig {
    int port;         // ZLG device TCP port
    int channel;      // CAN channel number
};

constexpr PortConfig PORT_CONFIGS[NUM_PORTS] = {
    {8000, 0},   // TCP port 8000, channel 0
    {8001, 1},   // TCP port 8001, channel 1
    {8002, 2},   // TCP port 8002, channel 2
    {8003, 3}    // TCP port 8003, channel 3
};

struct CanToRosConfig {
    std::string zlg_ip = "192.168.1.5";
    int arb_baud = 1000000;   // 1M bps
    int data_baud = 5000000;  // 5M bps
};

// ==================== DM Motor Format Decoder ====================
// DM motor feedback format (8 bytes):
// D[0] D[1] D[2] D[3] D[4] D[5] D[6] D[7]
// ID|STATE<<4, POS[15:8], POS[7:0], VEL[11:4], VEL[3:0]|EFFORT[11:8], EFFORT[7:0], T_MOS, T_Rotor

struct DMMotorData {
    uint8_t  raw[8];
    uint8_t  id;           // Motor ID (1-30)
    uint8_t  state;        // Motor state/error code

    // Raw fixed-point values from CAN
    int16_t position_raw;
    int16_t velocity_raw;
    int16_t effort_raw;    // Effort (torque) raw value

    // Physical values (decoded)
    double position;       // Position in radians
    double velocity;       // Velocity in rad/s
    double effort;         // Effort (torque) in Nm

    int8_t  temp_mos;
    int8_t  temp_rotor;
    bool valid;

    DMMotorData() : valid(false) {
        memset(raw, 0, sizeof(raw));
    }
};

// Motor calibration parameters
struct MotorCalibration {
    double p_max;
    double p_min;
    double v_max;
    double v_min;
    double t_max;
    double t_min;
};

class DMMotorFrameDecoder {
public:
    // Default DM motor range limits (fallback values)
    static double PMAX;    // Position max: π rad (180°)
    static double PMIN;    // Position min: -π rad (-180°)
    static double VMAX;    // Velocity max: 30 rad/s
    static double VMIN;    // Velocity min: -30 rad/s
    static double TMAX;    // Torque max: 20 Nm
    static double TMIN;    // Torque min: -20 Nm

    // Per-motor calibration (indexed by motor_id-1, so motor 1 is at index 0)
    static std::array<MotorCalibration, 30> motor_calibration;

    // Initialize motor calibration with default values
    static void InitMotorCalibration() {
        for (int i = 0; i < 30; i++) {
            motor_calibration[i] = {PMAX, PMIN, VMAX, VMIN, TMAX, TMIN};
        }
    }

    static DMMotorData DecodeFrameFD(const ZCAN_ReceiveFD_Data& frame, int motor_id_from_can) {
        DMMotorData data;

        const uint8_t* d = frame.frame.data;

        // Store raw data
        memcpy(data.raw, d, 8);

        // Use motor_id from CAN ID (not from data byte, which only has 4 bits)
        data.id = motor_id_from_can;
        data.state = (d[0] >> 4) & 0x0F;

        // Get calibration for this motor
        int motor_idx = data.id - 1;  // motor 1 -> index 0
        if (motor_idx < 0 || motor_idx >= 30) motor_idx = 0;

        const MotorCalibration& cal = motor_calibration[motor_idx];

        // D[1-2]: Position (16-bit encoded value, range: [0, 2^16-1])
        uint16_t pos_encoded = static_cast<uint16_t>((d[1] << 8) | d[2]);
        data.position_raw = static_cast<int16_t>(pos_encoded);
        data.position = (pos_encoded - 32768.0) / 65536.0 * (cal.p_max - cal.p_min);

        // D[3-4]: Velocity (12-bit encoded as unsigned, range: [0, 2^12-1])
        // D[3] = VEL[11:4], D[4] high nibble = VEL[3:0]
        int16_t vel_encoded = ((d[3] & 0xFF) << 4) | ((d[4] >> 4) & 0x0F);
        data.velocity_raw = vel_encoded;
        data.velocity = (vel_encoded - 2048.0) / 4096.0 * (cal.v_max - cal.v_min);

        // D[4-5]: Effort/Torque (12-bit encoded as unsigned, range: [0, 2^12-1])
        // D[4] low nibble = T[11:8], D[5] = T[7:0]
        int16_t effort_encoded = ((d[4] & 0x0F) << 8) | d[5];
        data.effort_raw = effort_encoded;
        data.effort = (effort_encoded - 2048.0) / 4096.0 * (cal.t_max - cal.t_min);

        // D[6-7]: Temperatures (8-bit signed, directly in Celsius)
        data.temp_mos = static_cast<int8_t>(d[6]);
        data.temp_rotor = static_cast<int8_t>(d[7]);

        data.valid = true;
        return data;
    }

    static std::string GetStateDescription(uint8_t state) {
        switch (state) {
            case 0x0: return "OK/Disabled";
            case 0x1: return "Enabled";
            case 0x8: return "Over-voltage";
            case 0x9: return "Under-voltage";
            case 0xA: return "Over-current";
            case 0xB: return "MOS Over-temp";
            case 0xC: return "Coil Over-temp";
            case 0xD: return "Comms Lost";
            case 0xE: return "Overload";
            default: return "Unknown";
        }
    }

    static void SetRanges(double pmax, double pmin, double vmax, double vmin, double tmax, double tmin) {
        PMAX = pmax;
        PMIN = pmin;
        VMAX = vmax;
        VMIN = vmin;
        TMAX = tmax;
        TMIN = tmin;
    }
};

// Static member definitions
double DMMotorFrameDecoder::PMAX = 3.14159;
double DMMotorFrameDecoder::PMIN = -3.14159;
double DMMotorFrameDecoder::VMAX = 30.0;
double DMMotorFrameDecoder::VMIN = -30.0;
double DMMotorFrameDecoder::TMAX = 20.0;
double DMMotorFrameDecoder::TMIN = -20.0;

// Per-motor calibration array
std::array<MotorCalibration, 30> DMMotorFrameDecoder::motor_calibration;

// ==================== Global Variables ====================
std::atomic<bool> g_running(true);
std::array<DMMotorData, NUM_MOTORS> g_motor_data;
std::mutex g_data_mutex;

void SignalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", stopping..." << std::endl;
    g_running = false;
}

// ==================== Forward Declarations ====================
class CanToRosManager;

// ==================== Single Port CAN Receiver ====================
// Based on cantoudp.cpp PortReceiver
// Handles one ZLG port (e.g., 8000)
// Uses device handle from manager (not opening its own)
class PortReceiver {
public:
    PortReceiver(const PortConfig& port_config, const CanToRosConfig& global_config,
                 DEVICE_HANDLE device_handle)
        : port_config_(port_config), global_config_(global_config), device_handle_(device_handle) {}

    ~PortReceiver() {
        stop();
    }

    bool Initialize();
    void StartReceiveThread();
    void StopReceiveThread();
    void stop();
    uint64_t GetReceiveCount() const { return receive_count_; }

private:
    void ReceiveLoop();

    PortConfig port_config_;
    CanToRosConfig global_config_;
    DEVICE_HANDLE device_handle_;  // Device handle from manager (shared)
    CHANNEL_HANDLE channel_handle_ = nullptr;
    std::thread receive_thread_;
    std::atomic<bool> receive_thread_running_{false};
    std::atomic<uint64_t> receive_count_{0};
};

// ==================== CAN to ROS2 Manager ====================
// Based on cantoudp.cpp CanToUdpManager
// Manages all 4 ports:
// - Opens 4 independent device handles
// - Creates PortReceiver instances with shared device handles
// - Coordinates ROS2 publishing
class CanToRosManager {
public:
    CanToRosManager(const CanToRosConfig& config, rclcpp::Node::SharedPtr node)
        : config_(config), node_(node) {}

    ~CanToRosManager() {
        stop();
    }

    bool Initialize();
    void Start();
    void stop();

    uint64_t GetTotalReceiveCount() const {
        uint64_t total = 0;
        for (const auto& receiver : port_receivers_) {
            total += receiver->GetReceiveCount();
        }
        return total;
    }

private:
    CanToRosConfig config_;
    rclcpp::Node::SharedPtr node_;
    std::vector<std::unique_ptr<PortReceiver>> port_receivers_;
    DEVICE_HANDLE device_handles_[NUM_PORTS] = {nullptr, nullptr, nullptr, nullptr};  // 4 independent device handles
};

// ==================== Test Mode Receiver (Shared Memory) ====================
// Reads CAN frames from shared memory instead of ZLG device
class TestModeReceiver {
public:
    TestModeReceiver(rclcpp::Node::SharedPtr node) : node_(node), shm_fd_(-1), shm_ptr_(nullptr), receive_count_(0) {}

    ~TestModeReceiver() {
        stop();
    }

    bool Initialize();
    void StartReceiveThread();
    void stop();
    uint64_t GetReceiveCount() const { return receive_count_; }

private:
    void ReceiveLoop();

    rclcpp::Node::SharedPtr node_;
    int shm_fd_;
    void* shm_ptr_;
    std::thread receive_thread_;
    std::atomic<bool> receive_thread_running_{false};
    std::atomic<uint64_t> receive_count_{0};
};

// ==================== PortReceiver Method Implementations ====================

bool PortReceiver::Initialize() {
    std::cout << "  [Port " << port_config_.port << "] Initializing..." << std::endl;

    // 1. Check device handle
    if (device_handle_ == nullptr || device_handle_ == INVALID_DEVICE_HANDLE) {
        std::cerr << "  [Port " << port_config_.port << "] ERROR: Invalid device handle!" << std::endl;
        return false;
    }
    std::cout << "  [Port " << port_config_.port << "] Using shared device handle: " << device_handle_ << std::endl;

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

    std::cout << "  [Port " << port_config_.port << "] Initializing CAN channel " << port_config_.channel << std::endl;

    channel_handle_ = ZCAN_InitCAN(device_handle_, port_config_.channel, &init_config);
    if (channel_handle_ == INVALID_CHANNEL_HANDLE) {
        std::cerr << "  [Port " << port_config_.port << "] Failed to initialize CAN channel " << port_config_.channel << std::endl;
        return false;
    }

    std::cout << "  [Port " << port_config_.port << "] CAN channel " << port_config_.channel
              << " initialized, handle: " << channel_handle_ << std::endl;

    // 3. Set IP and port for each device independently
    // Each device has its own connection: same IP, different port (8000-8003)
    // Device index matches channel number: device 0 uses channel 0, device 1 uses channel 1, etc.
    int device_index = port_config_.channel;
    std::cout << "  [Port " << port_config_.port << "] Setting device " << device_index << " IP and port" << std::endl;
    ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, device_index, device_index, CMD_DESIP,
                     (void*)global_config_.zlg_ip.c_str());
    // Use the port number from config for device connection
    uint32_t port_val = port_config_.port;
    ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, device_index, device_index, CMD_DESPORT, &port_val);

    std::cout << "  [Port " << port_config_.port << "] Device IP and port configured: "
              << global_config_.zlg_ip << ":" << port_config_.port << std::endl;

    // 4. Start CAN
    int start_result = ZCAN_StartCAN(channel_handle_);
    std::cout << "  [Port " << port_config_.port << "] ZCAN_StartCAN returned: " << start_result << std::endl;

    if (start_result != STATUS_OK) {
        std::cerr << "  [Port " << port_config_.port << "] Failed to start CAN channel" << std::endl;
        return false;
    }

    std::cout << "  [Port " << port_config_.port << "] Ready" << std::endl;
    return true;
}

void PortReceiver::StartReceiveThread() {
    receive_thread_running_ = true;
    receive_thread_ = std::thread(&PortReceiver::ReceiveLoop, this);
}

void PortReceiver::StopReceiveThread() {
    receive_thread_running_ = false;

    // Explicitly close channel to wake up ZCAN_ReceiveFD
    if (channel_handle_) {
        ZCAN_ResetCAN(channel_handle_);
        channel_handle_ = nullptr;
    }

    if (receive_thread_.joinable()) {
        // Simple timeout: detach if doesn't exit in 2 seconds
        std::thread timeout_thread([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            if (receive_thread_.joinable()) {
                std::cerr << "  [Port " << port_config_.port << "] WARNING: Receive thread hang, detaching..." << std::endl;
                receive_thread_.detach();
            }
        });
        timeout_thread.detach();

        receive_thread_.join();
    }
}

void PortReceiver::stop() {
    StopReceiveThread();
    if (channel_handle_) {
        std::cout << "  [Port " << port_config_.port << "] Resetting CAN channel" << std::endl;
        ZCAN_ResetCAN(channel_handle_);
    }
    // Note: device is managed by CanToRosManager, not closed here
    std::cout << "  [Port " << port_config_.port << "] Port receiver stopped" << std::endl;
}

void PortReceiver::ReceiveLoop() {
    ZCAN_ReceiveFD_Data receive_buffer[100];

    std::cout << "[Port " << port_config_.port << "] Receive thread running (Channel "
              << port_config_.channel << ")" << std::endl;

    while (receive_thread_running_) {
        // Check if channel is valid (may have been closed during stop)
        if (channel_handle_ == nullptr || channel_handle_ == INVALID_CHANNEL_HANDLE) {
            break;
        }

        uint32_t received = ZCAN_ReceiveFD(channel_handle_, receive_buffer, 100, 10);

        if (received > 0) {
            for (uint32_t i = 0; i < received; i++) {
                // Print all raw CAN frames (for debugging, like cantoudp.cpp)
                uint32_t can_id = receive_buffer[i].frame.can_id & 0x7FF;
                const uint8_t* d = receive_buffer[i].frame.data;
                uint8_t len = receive_buffer[i].frame.len;

                std::cout << "[Port " << port_config_.port << " RAW] CAN ID=0x"
                          << std::hex << std::setw(3) << std::setfill('0') << can_id << std::dec
                          << " Len=" << (int)len << " Data: ";
                for (uint8_t j = 0; j < len; j++) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0')
                              << static_cast<int>(d[j]) << " " << std::dec;
                }
                std::cout << std::endl;

                // Motor feedback: extract motor ID from CAN ID
                // Motor ID mapping: motor_id = CAN_ID - 0x20
                // For example: motor 30 has CAN ID = 0x3E (0x1E + 0x20)
                // We need to reverse this: motor_id = 0x3E - 0x20 = 0x1E = 30
                int motor_id_from_can = can_id - 0x20;

                // Check if CAN ID is valid (1-30) - matches cantoudp.cpp
                if (motor_id_from_can >= 1 && motor_id_from_can <= NUM_MOTORS) {
                    // Decode DM format (pass motor_id_from_can to correctly set data.id)
                    DMMotorData dm_data = DMMotorFrameDecoder::DecodeFrameFD(receive_buffer[i], motor_id_from_can);

                    std::lock_guard<std::mutex> lock(g_data_mutex);
                    // Map CAN ID to array index: array_index = motor_id - 1
                    g_motor_data[motor_id_from_can - 1] = dm_data;
                    receive_count_++;
                }
            }
        }
    }

    std::cout << "[Port " << port_config_.port << "] Receive thread stopped" << std::endl;
}

// ==================== CanToRosManager Method Implementations ====================

bool CanToRosManager::Initialize() {
    std::cout << "=== Initializing CAN-to-ROS2 Manager ===" << std::endl;
    std::cout << "ZLG IP: " << config_.zlg_ip << std::endl;
    std::cout << "Ports: ";
    for (const auto& pc : PORT_CONFIGS) {
        std::cout << pc.port << "(ch" << pc.channel << ") ";
    }
    std::cout << "\nTotal motors: " << NUM_MOTORS << std::endl;

    // 1. Open 4 independent devices (one for each port) - like cantoudp.cpp
    std::cout << "[Device] Opening " << NUM_PORTS << " ZLG devices..." << std::endl;
    for (int i = 0; i < NUM_PORTS; i++) {
        device_handles_[i] = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, i, 0);
        if (device_handles_[i] == INVALID_DEVICE_HANDLE) {
            std::cerr << "[Device] ERROR: Failed to open ZCAN device for port " << PORT_CONFIGS[i].port << "!" << std::endl;
            // Close any previously opened devices
            for (int j = 0; j < i; j++) {
                ZCAN_CloseDevice(device_handles_[j]);
                device_handles_[j] = nullptr;
            }
            return false;
        }
        std::cout << "[Device] Device " << i << " opened for port " << PORT_CONFIGS[i].port
                  << ", handle: " << device_handles_[i] << std::endl;
    }

    // 2. Create port receivers with shared device handles
    std::cout << "[Receiver] Creating port receivers..." << std::endl;
    for (int i = 0; i < NUM_PORTS; i++) {
        auto receiver = std::make_unique<PortReceiver>(PORT_CONFIGS[i], config_, device_handles_[i]);
        if (!receiver->Initialize()) {
            std::cerr << "[Receiver] ERROR: Failed to initialize port " << PORT_CONFIGS[i].port << std::endl;
            // Clean up
            for (int j = 0; j < NUM_PORTS; j++) {
                ZCAN_CloseDevice(device_handles_[j]);
                device_handles_[j] = nullptr;
            }
            return false;
        }
        port_receivers_.push_back(std::move(receiver));
    }

    std::cout << "[Receiver] All " << port_receivers_.size() << " port receivers initialized" << std::endl;
    return true;
}

void CanToRosManager::Start() {
    std::cout << "[Manager] Starting all receive threads..." << std::endl;
    for (auto& receiver : port_receivers_) {
        receiver->StartReceiveThread();
    }
    std::cout << "[Manager] All receive threads started" << std::endl;
}

void CanToRosManager::stop() {
    std::cout << "[Manager] Stopping..." << std::endl;
    port_receivers_.clear();

    // Close all device handles
    for (int i = 0; i < NUM_PORTS; i++) {
        if (device_handles_[i] != nullptr) {
            std::cout << "[Device] Closing device " << i << std::endl;
            ZCAN_CloseDevice(device_handles_[i]);
            device_handles_[i] = nullptr;
        }
    }
    std::cout << "[Manager] Stopped" << std::endl;
}

// ==================== TestModeReceiver Method Implementations ====================

bool TestModeReceiver::Initialize() {
    std::cout << "[Test Mode] Opening shared memory: " << SHM_NAME << std::endl;

    shm_fd_ = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (shm_fd_ == -1) {
        std::cerr << "[Test Mode] Failed to open shared memory: " << strerror(errno) << std::endl;
        std::cerr << "[Test Mode] Make sure can_send_test is running!" << std::endl;
        return false;
    }

    shm_ptr_ = mmap(nullptr, SHM_SIZE, PROT_READ, MAP_SHARED, shm_fd_, 0);
    if (shm_ptr_ == MAP_FAILED) {
        std::cerr << "[Test Mode] Failed to map shared memory: " << strerror(errno) << std::endl;
        close(shm_fd_);
        shm_fd_ = -1;
        return false;
    }

    std::cout << "[Test Mode] Connected to shared memory (size: " << SHM_SIZE << " bytes)" << std::endl;
    std::cout << "[Test Mode] Reading from can_send_test..." << std::endl;
    return true;
}

void TestModeReceiver::StartReceiveThread() {
    receive_thread_running_ = true;
    receive_thread_ = std::thread(&TestModeReceiver::ReceiveLoop, this);
}

void TestModeReceiver::stop() {
    receive_thread_running_ = false;

    if (receive_thread_.joinable()) {
        std::thread timeout_thread([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            if (receive_thread_.joinable()) {
                std::cerr << "[Test Mode] WARNING: Receive thread hang, detaching..." << std::endl;
                receive_thread_.detach();
            }
        });
        timeout_thread.detach();

        receive_thread_.join();
    }

    if (shm_ptr_ != nullptr && shm_ptr_ != MAP_FAILED) {
        munmap(shm_ptr_, SHM_SIZE);
        shm_ptr_ = nullptr;
    }

    if (shm_fd_ != -1) {
        close(shm_fd_);
        shm_fd_ = -1;
    }
}

void TestModeReceiver::ReceiveLoop() {
    std::cout << "[Test Mode] Receive thread running..." << std::endl;

    uint64_t print_counter = 0;
    while (receive_thread_running_ && rclcpp::ok()) {
        if (shm_ptr_ == nullptr) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        ShmLayout* shm_data = static_cast<ShmLayout*>(shm_ptr_);

        if (shm_data->data_ready && shm_data->frame_count > 0) {
            for (uint32_t i = 0; i < shm_data->frame_count; i++) {
                const ShmCANFrame& shm_frame = shm_data->frames[i];

                if (!shm_frame.valid) continue;

                // Convert ShmCANFrame to ZCAN_ReceiveFD_Data format (reuse existing decoder)
                ZCAN_ReceiveFD_Data zcan_frame;
                memset(&zcan_frame, 0, sizeof(zcan_frame));
                zcan_frame.frame.can_id = shm_frame.can_id;
                zcan_frame.frame.len = shm_frame.len;
                memcpy(zcan_frame.frame.data, shm_frame.data, 8);

                // Print raw CAN frame (only first few frames)
                print_counter++;
                if (print_counter <= 3 || print_counter % 100 == 0) {
                    std::cout << "[Test Mode RX] Sequence " << shm_data->sequence
                              << " CAN ID=" << zcan_frame.frame.can_id << " Data: ";
                    for (uint8_t j = 0; j < 8; j++) {
                        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)zcan_frame.frame.data[j] << " " << std::dec;
                    }
                    std::cout << std::endl;
                }

                // Use CAN ID as global motor ID (1-30)
                int global_motor_id = zcan_frame.frame.can_id;
                if (global_motor_id >= 1 && global_motor_id <= NUM_MOTORS) {
                    // Decode DM format using existing decoder (pass global_motor_id)
                    DMMotorData dm_data = DMMotorFrameDecoder::DecodeFrameFD(zcan_frame, global_motor_id);

                    std::lock_guard<std::mutex> lock(g_data_mutex);
                    g_motor_data[global_motor_id - 1] = dm_data;
                    receive_count_++;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "[Test Mode] Receive thread stopped" << std::endl;
}

// ==================== Main ====================
int main(int argc, char** argv) {
    // Setup signal handler
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    // Initialize motor calibration
    DMMotorFrameDecoder::InitMotorCalibration();

    // Set per-motor calibration parameters
    // Motors 1-3, 7-10, 14-17: small motors
    for (int i : {0, 1, 2, 6, 7, 8, 9, 13, 14, 15, 16}) {
        DMMotorFrameDecoder::motor_calibration[i] = {12.5, -12.5, 10.0, -10.0, 28.0, -28.0};
    }
    // Motors 4-6, 11-13, 23-24, 29-30: medium motors
    for (int i : {3, 4, 5, 10, 11, 12, 22, 23, 28, 29}) {
        DMMotorFrameDecoder::motor_calibration[i] = {12.566, -12.566, 20.0, -20.0, 120.0, -120.0};
    }
    // Motors 18-22, 25-28: large motors
    for (int i : {17, 18, 19, 20, 21, 24, 25, 26, 27}) {
        DMMotorFrameDecoder::motor_calibration[i] = {12.5, -12.5, 25.0, -25.0, 200.0, -200.0};
    }

    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create ROS2 node
    auto node = rclcpp::Node::make_shared("can_motor_feedback_publisher");

    // Create publisher with QoS profile
    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    qos_profile.reliable();
    qos_profile.durability_volatile();

    auto publisher = node->create_publisher<multi_port_motor_feedback::msg::LowState>(
        "/motor_feedback", qos_profile);

    // Configuration
    CanToRosConfig config;
    bool test_mode = false;

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--test-mode") {
            test_mode = true;
        } else if (arg == "--zlg-ip" && i + 1 < argc) {
            config.zlg_ip = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n";
            std::cout << "Options:\n";
            std::cout << "  --test-mode          Read from shared memory (for testing with can_send_test)\n";
            std::cout << "  --zlg-ip <address>  ZLG device IP (default: 192.168.1.5)\n";
            std::cout << "  -h, --help          Show this help message\n";
            std::cout << "\nNormal mode: reads from ZLG CAN device (4 ports, 30 motors)\n";
            std::cout << "Test mode: reads from shared memory /can_motor_test_shm\n";
            std::cout << "\nMotor distribution:\n";
            std::cout << "  Port 8000: motors 1-10   (10 motors)\n";
            std::cout << "  Port 8001: motors 11-17  (7 motors)\n";
            std::cout << "  Port 8002: motors 18-24  (7 motors)\n";
            std::cout << "  Port 8003: motors 25-30  (6 motors)\n";
            rclcpp::shutdown();
            return 0;
        }
    }

    if (test_mode) {
        RCLCPP_INFO(node->get_logger(), "========================================");
        RCLCPP_INFO(node->get_logger(), "TEST MODE: Reading from shared memory");
        RCLCPP_INFO(node->get_logger(), "Shared memory: %s", SHM_NAME);
        RCLCPP_INFO(node->get_logger(), "Make sure can_send_test is running!");
        RCLCPP_INFO(node->get_logger(), "========================================");
    } else {
        RCLCPP_INFO(node->get_logger(), "CAN Motor Feedback Publisher Started");
        RCLCPP_INFO(node->get_logger(), "ZLG IP: %s", config.zlg_ip.c_str());
    }
    RCLCPP_INFO(node->get_logger(), "Publishing to: /motor_feedback");
    RCLCPP_INFO(node->get_logger(), "Total motors: %d", NUM_MOTORS);
    RCLCPP_INFO(node->get_logger(), "DM format: 8 bytes per motor");

    // Create receiver based on mode
    std::unique_ptr<TestModeReceiver> test_receiver;
    std::unique_ptr<CanToRosManager> manager;

    if (test_mode) {
        // Test mode: use shared memory
        test_receiver = std::make_unique<TestModeReceiver>(node);
        if (!test_receiver->Initialize()) {
            RCLCPP_ERROR(node->get_logger(), "Failed to initialize test mode receiver");
            RCLCPP_ERROR(node->get_logger(), "Make sure can_send_test is running!");
            rclcpp::shutdown();
            return 1;
        }
        test_receiver->StartReceiveThread();
        RCLCPP_INFO(node->get_logger(), "Test mode receiver started");
    } else {
        // Normal mode: use ZLG device
        manager = std::make_unique<CanToRosManager>(config, node);
        if (!manager->Initialize()) {
            RCLCPP_ERROR(node->get_logger(), "Failed to initialize CAN-to-ROS2 manager");
            rclcpp::shutdown();
            return 1;
        }
        manager->Start();
        RCLCPP_INFO(node->get_logger(), "All CAN receive threads started");
    }

    // Publishing loop at 100Hz - publishes LowState with 30 motors in one message
    auto publish_timer = node->create_wall_timer(
        std::chrono::milliseconds(10),
        [&node, &publisher]() {
            // Create LowState message with all 30 motors
            auto msg = multi_port_motor_feedback::msg::LowState();

            // Timestamp
            msg.header.stamp = node->now();

            std::lock_guard<std::mutex> lock(g_data_mutex);

            // Fill motor_state array (index 0 = motor 1, index 29 = motor 30)
            for (int motor_id = 1; motor_id <= NUM_MOTORS; motor_id++) {
                int array_index = motor_id - 1;
                const DMMotorData& dm_data = g_motor_data[array_index];

                auto& motor_msg = msg.motor_state[array_index];

                if (dm_data.valid) {
                    // Debug: print motor 30 data
                    if (motor_id == 30) {
                        std::cout << "[DEBUG] Motor 30: dm_data.id=" << (int)dm_data.id
                                  << " position=" << dm_data.position << std::endl;
                    }
                    motor_msg.id = static_cast<int8_t>(dm_data.id);
                    motor_msg.state = dm_data.state;
                    motor_msg.position = static_cast<float>(dm_data.position);
                    motor_msg.velocity = static_cast<float>(dm_data.velocity);
                    motor_msg.effort = static_cast<float>(dm_data.effort);
                    motor_msg.temp_mos = dm_data.temp_mos;
                    motor_msg.temp_rotor = dm_data.temp_rotor;
                } else {
                    // Default values for invalid motors
                    motor_msg.id = static_cast<int8_t>(motor_id);
                    motor_msg.state = 0;
                    motor_msg.position = 0.0f;
                    motor_msg.velocity = 0.0f;
                    motor_msg.effort = 0.0f;
                    motor_msg.temp_mos = 0;
                    motor_msg.temp_rotor = 0;
                }
            }

            publisher->publish(msg);
        }
    );

    // Status timer (every 5 seconds)
    auto status_timer = node->create_wall_timer(
        std::chrono::seconds(5),
        [&node, &manager, &test_receiver, test_mode]() {
            uint64_t total_count = test_mode ? test_receiver->GetReceiveCount() : manager->GetTotalReceiveCount();
            RCLCPP_INFO(node->get_logger(), "Total CAN frames received: %lu", total_count);
        }
    );

    // Spin the node
    rclcpp::spin(node);

    // Cleanup
    RCLCPP_INFO(node->get_logger(), "Shutting down...");
    if (test_mode) {
        test_receiver.reset();
    } else {
        manager.reset();
    }
    rclcpp::shutdown();

    return 0;
}
