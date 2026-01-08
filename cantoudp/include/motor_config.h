#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include <string>
#include <vector>
#include <array>

// ==================== Motor Calibration Limits ====================
struct MotorLimits {
    double min;
    double max;

    MotorLimits() : min(0.0), max(0.0) {}
    MotorLimits(double min_val, double max_val) : min(min_val), max(max_val) {}

    // Check if value is within limits
    bool IsValid(double value) const {
        return value >= min && value <= max;
    }

    // Clamp value to limits
    double Clamp(double value) const {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
};

// ==================== Single Motor Parameter Set ====================
struct MotorParams {
    int can_id;                    // CAN ID (1-30)

    MotorLimits position;          // Position limits (rad)
    MotorLimits velocity;          // Velocity limits (rad/s)
    MotorLimits torque;            // Torque limits (Nm)
    // Temperature has no min/max limits

    MotorParams() : can_id(0) {}
};

// ==================== Unpacked Motor Data ====================
struct UnpackedMotorData {
    int can_id;
    double position;       // radians
    double velocity;       // rad/s
    double torque;         // Nm
    double temperature;    // Celsius

    // Status flags
    bool position_valid;
    bool velocity_valid;
    bool torque_valid;
    bool temperature_valid;

    UnpackedMotorData()
        : can_id(0), position(0.0), velocity(0.0), torque(0.0), temperature(0.0)
        , position_valid(false), velocity_valid(false), torque_valid(false), temperature_valid(false)
    {}

    void Print() const {
        printf("Motor %d: pos=%.3f rad, vel=%.2f rad/s, trq=%.2f Nm, temp=%.1f C%s\n",
               can_id, position, velocity, torque, temperature,
               (position_valid && velocity_valid && torque_valid && temperature_valid) ? "" : " [INVALID]");
    }
};

// ==================== All Motors Configuration ====================
class MotorConfig {
public:
    static constexpr int NUM_MOTORS = 30;

    MotorConfig();

    // Load configuration from YAML file
    bool LoadFromFile(const std::string& config_path);

    // Get motor parameters by CAN ID (1-30) or array index (0-29)
    const MotorParams& GetMotorParamsByCanId(int can_id) const;
    const MotorParams& GetMotorParamsByIndex(int index) const;

    // Unpack raw CAN data to physical values
    UnpackedMotorData UnpackMotorData(int can_id, const uint8_t* raw_data) const;

    // Validate unpacked data against limits
    bool ValidateMotorData(const UnpackedMotorData& data) const;

    // Set default path for config file
    void SetDefaultConfigPath(const std::string& path) {
        default_config_path_ = path;
    }

private:
    std::array<MotorParams, NUM_MOTORS> motor_params_;  // Index 0-29 = CAN ID 1-30
    MotorParams default_params_;                         // Global defaults
    std::string default_config_path_;

    bool LoadYamlFile(const std::string& path);
    void ApplyDefaults();
};

#endif // MOTOR_CONFIG_H
