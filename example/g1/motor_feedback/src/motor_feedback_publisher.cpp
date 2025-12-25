// motor_feedback_publisher.cpp
// Publishes DM motor feedback from ZLG CAN device to ROS2 topic

#include <rclcpp/rclcpp.hpp>
#include <motor_feedback/msg/motor_feedback.hpp>
#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <iomanip>
#include <cmath>

// ZLG CANFDNET SDK includes
// Note: CANFDNET.h must be included before zlgcan.h for proper extern "C" linkage
#include "CANFDNET.h"

// ==================== DM Motor Format Decoder ====================
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
    uint8_t  raw[8];
    uint8_t  motor_id;
    uint8_t  error;

    // Raw fixed-point values from CAN
    int16_t position_raw;
    int16_t velocity_raw;
    int16_t torque_raw;

    // Physical values (decoded)
    double position_rad;
    double velocity_rad_s;
    double torque_nm;

    int8_t  temp_mos;
    int8_t  temp_rotor;
};

class DMMotorFrameDecoder {
public:
    // DM motor range limits (adjust based on your motor specifications)
    // Changed from constexpr to allow runtime modification via SetRanges()
    static double PMAX;    // Position max: π rad (180°)
    static double PMIN;    // Position min: -π rad (-180°)
    static double VMAX;    // Velocity max: 45 rad/s
    static double VMIN;    // Velocity min: -45 rad/s
    static double TMAX;    // Torque max: 20 Nm
    static double TMIN;    // Torque min: -20 Nm

    static DMMotorData DecodeFrameFD(const ZCAN_ReceiveFD_Data& frame) {
        DMMotorData data = {};

        const uint8_t* d = frame.frame.data;

        // Store raw data
        memcpy(data.raw, d, 8);

        // D[0]: ID[3:0] | ERR[3:0]<<4
        data.motor_id = d[0] & 0x0F;
        data.error = (d[0] >> 4) & 0x0F;

        // D[1-2]: Position (16-bit encoded value, range: [0, 2^16-1])
        // Encoding: POS_16bit = position_rad / (PMAX - PMIN) * 2^16 + 2^15
        // Decoding: position_rad = (POS_16bit - 2^15) / 2^16 * (PMAX - PMIN)
        uint16_t pos_encoded = static_cast<uint16_t>((d[1] << 8) | d[2]);
        data.position_raw = static_cast<int16_t>(pos_encoded);  // Store as signed for compatibility
        data.position_rad = (pos_encoded - 32768.0) / 65536.0 * (PMAX - PMIN);

        // D[3-4]: Velocity (12-bit encoded as unsigned, range: [0, 2^12-1])
        // Encoding: VEL_12bit = velocity_rad_s / (VMAX - VMIN) * 2^12 + 2^11
        // Decoding: velocity_rad_s = (VEL_12bit - 2^11) / 2^12 * (VMAX - VMIN)
        int16_t vel_encoded = ((d[3] & 0xFF) << 4) | (d[4] & 0x0F);
        data.velocity_raw = vel_encoded;  // Store raw encoded value
        data.velocity_rad_s = (vel_encoded - 2048.0) / 4096.0 * (VMAX - VMIN);

        // D[4-5]: Torque (12-bit encoded as unsigned, range: [0, 2^12-1])
        // Encoding: T_12bit = torque_nm / (TMAX - TMIN) * 2^12 + 2^11
        // Decoding: torque_nm = (T_12bit - 2^11) / 2^12 * (TMAX - TMIN)
        int16_t torque_encoded = (((d[4] >> 4) & 0x0F) << 8) | d[5];
        data.torque_raw = torque_encoded;  // Store raw encoded value
        data.torque_nm = (torque_encoded - 2048.0) / 4096.0 * (TMAX - TMIN);

        // D[6-7]: Temperatures (8-bit signed, directly in Celsius)
        data.temp_mos = static_cast<int8_t>(d[6]);
        data.temp_rotor = static_cast<int8_t>(d[7]);

        return data;
    }

    static std::string GetErrorDescription(uint8_t error) {
        switch (error) {
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

    // Allow custom range configuration
    static void SetRanges(double pmax, double pmin, double vmax, double vmin, double tmax, double tmin) {
        PMAX = pmax;
        PMIN = pmin;
        VMAX = vmax;
        VMIN = vmin;
        TMAX = tmax;
        TMIN = tmin;
    }
};

class MotorFeedbackPublisher : public rclcpp::Node {
public:
    MotorFeedbackPublisher() : Node("motor_feedback_publisher") {
        // Declare parameters
        this->declare_parameter("zlg_ip", "192.168.1.5");
        this->declare_parameter("zlg_port", 8002);
        this->declare_parameter("channel", 2);
        this->declare_parameter("arb_baud", 1000000);
        this->declare_parameter("data_baud", 5000000);
        this->declare_parameter("publish_rate", 100.0);  // Hz

        // Motor range parameters
        this->declare_parameter("pmax", 3.14159);
        this->declare_parameter("pmin", -3.14159);
        this->declare_parameter("vmax", 45.0);
        this->declare_parameter("vmin", -45.0);
        this->declare_parameter("tmax", 20.0);
        this->declare_parameter("tmin", -20.0);

        // Get parameters
        std::string zlg_ip = this->get_parameter("zlg_ip").as_string();
        int zlg_port = this->get_parameter("zlg_port").as_int();
        int channel = this->get_parameter("channel").as_int();
        int arb_baud = this->get_parameter("arb_baud").as_int();
        int data_baud = this->get_parameter("data_baud").as_int();
        double publish_rate = this->get_parameter("publish_rate").as_double();

        // Get motor range parameters
        double pmax = this->get_parameter("pmax").as_double();
        double pmin = this->get_parameter("pmin").as_double();
        double vmax = this->get_parameter("vmax").as_double();
        double vmin = this->get_parameter("vmin").as_double();
        double tmax = this->get_parameter("tmax").as_double();
        double tmin = this->get_parameter("tmin").as_double();

        // Configure decoder with motor ranges
        DMMotorFrameDecoder::SetRanges(pmax, pmin, vmax, vmin, tmax, tmin);

        // Create publisher
        publisher_ = this->create_publisher<motor_feedback::msg::MotorFeedback>(
            "/motor_feedback", 10);

        // Initialize ZLG device
        if (!InitializeZLG(zlg_ip, zlg_port, channel, arb_baud, data_baud)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize ZLG device");
            return;
        }

        // Start receive thread
        receive_thread_running_ = true;
        receive_thread_ = std::thread(&MotorFeedbackPublisher::ReceiveThread, this);

        // Start publish timer
        auto period = std::chrono::duration<double>(1.0 / publish_rate);
        publish_timer_ = this->create_wall_timer(
            period, std::bind(&MotorFeedbackPublisher::PublishTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Motor Feedback Publisher started");
        RCLCPP_INFO(this->get_logger(), "ZLG: %s:%d, Channel: %d", zlg_ip.c_str(), zlg_port, channel);
        RCLCPP_INFO(this->get_logger(), "Motor ranges: P[%.2f,%.2f] V[%.1f,%.1f] T[%.1f,%.1f]",
                    pmin, pmax, vmin, vmax, tmin, tmax);
        RCLCPP_INFO(this->get_logger(), "Publishing to /motor_feedback at %.1f Hz", publish_rate);
    }

    ~MotorFeedbackPublisher() {
        receive_thread_running_ = false;
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
        if (channel_handle_) {
            ZCAN_ResetCAN(channel_handle_);
        }
        if (device_handle_) {
            ZCAN_CloseDevice(device_handle_);
        }
    }

private:
    bool InitializeZLG(const std::string& ip, int port, int channel, int arb_baud, int data_baud) {
        RCLCPP_INFO(this->get_logger(), "=== Initializing ZCAN Connection ===");
        RCLCPP_INFO(this->get_logger(), "Target: %s:%d", ip.c_str(), port);
        RCLCPP_INFO(this->get_logger(), "Channel: CAN%d", channel);

        // 1. ZCAN_OpenDevice - Open device (use device type constant, not IP)
        device_handle_ = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, 0, 0);
        if (device_handle_ == INVALID_DEVICE_HANDLE) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open ZCAN device");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "[1/4] Device opened (ZCAN_OpenDevice)");

        // 2. ZCAN_InitCAN - Initialize CAN channel (CAN-FD mode with explicit config)
        ZCAN_CHANNEL_INIT_CONFIG init_config;
        memset(&init_config, 0, sizeof(init_config));

        // Use CAN-FD mode with explicit baud rate settings
        init_config.can_type = TYPE_CANFD;            // CAN-FD mode
        init_config.canfd.acc_code = 0;              // Accept code
        init_config.canfd.acc_mask = 0;              // Accept mask (0 = accept all)
        init_config.canfd.abit_timing = arb_baud;    // Arbitration baud rate (1M default)
        init_config.canfd.dbit_timing = data_baud;   // Data baud rate (5M default)
        init_config.canfd.brp = 0;                   // Baud rate prescaler
        init_config.canfd.filter = 0;                // Filter
        init_config.canfd.mode = 0;                  // Normal mode

        RCLCPP_INFO(this->get_logger(), "  CAN-FD config: abit_timing=%d, dbit_timing=%d",
                    arb_baud, data_baud);

        channel_handle_ = ZCAN_InitCAN(device_handle_, channel, &init_config);
        if (channel_handle_ == INVALID_CHANNEL_HANDLE) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN channel %d", channel);
            ZCAN_CloseDevice(device_handle_);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "[2/4] CAN channel initialized (CAN-FD %d/%d)",
                    arb_baud, data_baud);

        // 3. ZCAN_SetReference - Set IP and port
        uint32_t val = 0;
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, channel, CMD_DESIP, (void*)ip.c_str());
        val = port;
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, channel, CMD_DESPORT, &val);
        RCLCPP_INFO(this->get_logger(), "[3/4] IP:Port configured");

        // 4. ZCAN_StartCAN - Start CAN
        if (ZCAN_StartCAN(channel_handle_) != STATUS_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start CAN channel %d", channel);
            ZCAN_CloseDevice(device_handle_);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "[4/4] CAN started");
        RCLCPP_INFO(this->get_logger(), "=====================================");

        channel_ = channel;
        return true;
    }

    void ReceiveThread() {
        ZCAN_ReceiveFD_Data receive_buffer[100];
        uint64_t receive_count = 0;
        uint64_t motor_response_count = 0;

        while (receive_thread_running_ && rclcpp::ok()) {
            uint32_t received = ZCAN_ReceiveFD(channel_handle_, receive_buffer, 100, 10);

            if (received > 0) {
                receive_count += received;

                for (uint32_t i = 0; i < received; i++) {
                    // Extract CAN ID (11-bit standard ID)
                    uint32_t can_id = receive_buffer[i].frame.can_id & 0x7FF;

                    // Motor feedback: CAN ID 1-30
                    if (can_id >= 1 && can_id <= 30 && receive_buffer[i].frame.len >= 8) {
                        // Decode DM format
                        DMMotorData dm_data = DMMotorFrameDecoder::DecodeFrameFD(receive_buffer[i]);
                        motor_response_count++;

                        // Store in buffer for publishing
                        std::lock_guard<std::mutex> lock(motor_data_mutex_);
                        motor_data_buffer_[dm_data.motor_id] = dm_data;

                        // Print raw data on console
                        std::cout << "\n[RX] Motor ID=" << static_cast<int>(dm_data.motor_id)
                                  << " | RAW: ";
                        for (int j = 0; j < 8; j++) {
                            std::cout << std::hex << std::setw(2) << std::setfill('0')
                                      << static_cast<int>(dm_data.raw[j]) << " ";
                        }
                        std::cout << std::dec << "\n      Pos=" << dm_data.position_rad << " rad"
                                  << " (raw=" << dm_data.position_raw << ")"
                                  << " | Vel=" << dm_data.velocity_rad_s << " rad/s"
                                  << " (raw=" << dm_data.velocity_raw << ")"
                                  << " | Tau=" << dm_data.torque_nm << " Nm"
                                  << " (raw=" << dm_data.torque_raw << ")"
                                  << " | T_MOS=" << static_cast<int>(dm_data.temp_mos) << "°C"
                                  << " | T_Rotor=" << static_cast<int>(dm_data.temp_rotor) << "°C"
                                  << std::endl;
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        RCLCPP_INFO(this->get_logger(), "Receive thread stopped. Total: %lu frames, %lu motor responses",
                    receive_count, motor_response_count);
    }

    void PublishTimerCallback() {
        std::lock_guard<std::mutex> lock(motor_data_mutex_);

        if (motor_data_buffer_.empty()) {
            return;
        }

        // Publish all motor data in buffer
        for (const auto& pair : motor_data_buffer_) {
            const DMMotorData& dm_data = pair.second;

            auto msg = motor_feedback::msg::MotorFeedback();

            // Timestamp
            msg.header.stamp = this->now();

            // Motor identification
            msg.motor_id = dm_data.motor_id;
            msg.can_id = dm_data.motor_id;

            // Error code
            msg.error = dm_data.error;

            // Raw values (fixed-point)
            msg.position = dm_data.position_raw;
            msg.velocity = dm_data.velocity_raw;
            msg.torque = dm_data.torque_raw;

            // Temperatures
            msg.temp_mos = dm_data.temp_mos;
            msg.temp_rotor = dm_data.temp_rotor;

            // Scaled values (decoded physical units)
            msg.position_rad = static_cast<float>(dm_data.position_rad);
            msg.velocity_rad_s = static_cast<float>(dm_data.velocity_rad_s);
            msg.torque_nm = static_cast<float>(dm_data.torque_nm);

            // Raw CAN data
            for (int i = 0; i < 8; i++) {
                msg.can_data[i] = dm_data.raw[i];
            }

            publisher_->publish(msg);
        }

        // Clear buffer after publishing
        motor_data_buffer_.clear();
    }

    // ZLG device handles
    DEVICE_HANDLE device_handle_ = nullptr;
    CHANNEL_HANDLE channel_handle_ = nullptr;
    int channel_ = 0;

    // ROS2 publisher
    rclcpp::Publisher<motor_feedback::msg::MotorFeedback>::SharedPtr publisher_;

    // Receive thread
    std::thread receive_thread_;
    std::atomic<bool> receive_thread_running_{false};
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // Motor data buffer
    std::map<int, DMMotorData> motor_data_buffer_;
    std::mutex motor_data_mutex_;
};

// Static member definitions
double DMMotorFrameDecoder::PMAX = 3.14159;
double DMMotorFrameDecoder::PMIN = -3.14159;
double DMMotorFrameDecoder::VMAX = 45.0;
double DMMotorFrameDecoder::VMIN = -45.0;
double DMMotorFrameDecoder::TMAX = 20.0;
double DMMotorFrameDecoder::TMIN = -20.0;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MotorFeedbackPublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
