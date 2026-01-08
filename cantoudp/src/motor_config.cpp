#include "motor_config.h"
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <cstring>
#include <cmath>

// ==================== Helper Functions ====================

// Extract signed/signed integer from byte array
template<typename T>
T ExtractInt(const uint8_t* data, int offset) {
    T value = 0;
    memcpy(&value, data + offset, sizeof(T));
    return value;
}

// ==================== MotorConfig Implementation ====================

MotorConfig::MotorConfig() {
    // Initialize with default values
    default_params_.can_id = 0;
    default_params_.position = MotorLimits(-3.14159, 3.14159);
    default_params_.velocity = MotorLimits(-30.0, 30.0);
    default_params_.torque = MotorLimits(-50.0, 50.0);

    // Initialize all motors with defaults
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_params_[i] = default_params_;
        motor_params_[i].can_id = i + 1;  // CAN ID 1-30
    }

    // Set default config path
    default_config_path_ = "/home/linaro/ludan_sdk/cantoudp/config/motor_params.yaml";
}

bool MotorConfig::LoadFromFile(const std::string& config_path) {
    std::string path = config_path.empty() ? default_config_path_ : config_path;
    return LoadYamlFile(path);
}

const MotorParams& MotorConfig::GetMotorParamsByCanId(int can_id) const {
    if (can_id >= 1 && can_id <= NUM_MOTORS) {
        return motor_params_[can_id - 1];
    }
    // Return first motor as fallback (or could throw exception)
    std::cerr << "[MotorConfig] Invalid CAN ID: " << can_id << std::endl;
    return motor_params_[0];
}

const MotorParams& MotorConfig::GetMotorParamsByIndex(int index) const {
    if (index >= 0 && index < NUM_MOTORS) {
        return motor_params_[index];
    }
    std::cerr << "[MotorConfig] Invalid index: " << index << std::endl;
    return motor_params_[0];
}

UnpackedMotorData MotorConfig::UnpackMotorData(int can_id, const uint8_t* raw_data) const {
    UnpackedMotorData result;
    result.can_id = can_id;

    const MotorParams& params = GetMotorParamsByCanId(can_id);

    // Hard-coded CAN data format (6 bytes per motor):
    // Byte 0-1: Position (int16)
    // Byte 2-3: Velocity (int16)
    // Byte 4: Torque (int8)
    // Byte 5: Temperature (uint8)

    // Unpack position (int16, offset 0, scale 0.001)
    int16_t raw_pos = ExtractInt<int16_t>(raw_data, 0);
    result.position = raw_pos * 0.001;
    result.position_valid = params.position.IsValid(result.position);

    // Unpack velocity (int16, offset 2, scale 0.01)
    int16_t raw_vel = ExtractInt<int16_t>(raw_data, 2);
    result.velocity = raw_vel * 0.01;
    result.velocity_valid = params.velocity.IsValid(result.velocity);

    // Unpack torque (int8, offset 4, scale 0.1)
    int8_t raw_trq = ExtractInt<int8_t>(raw_data, 4);
    result.torque = raw_trq * 0.1;
    result.torque_valid = params.torque.IsValid(result.torque);

    // Unpack temperature (uint8, offset 5, scale 1.0) - no min/max validation
    uint8_t raw_temp = ExtractInt<uint8_t>(raw_data, 5);
    result.temperature = raw_temp;
    result.temperature_valid = true;  // Temperature has no limits

    return result;
}

bool MotorConfig::ValidateMotorData(const UnpackedMotorData& data) const {
    return data.position_valid && data.velocity_valid &&
           data.torque_valid && data.temperature_valid;
}

// ==================== YAML Loading ====================

bool MotorConfig::LoadYamlFile(const std::string& path) {
    try {
        YAML::Node config = YAML::LoadFile(path);

        std::cout << "[MotorConfig] Loading config from: " << path << std::endl;

        // Load global defaults
        if (config["global_defaults"]) {
            const auto& defaults = config["global_defaults"];

            if (defaults["position"]) {
                default_params_.position.min = defaults["position"]["min"].as<double>();
                default_params_.position.max = defaults["position"]["max"].as<double>();
            }
            if (defaults["velocity"]) {
                default_params_.velocity.min = defaults["velocity"]["min"].as<double>();
                default_params_.velocity.max = defaults["velocity"]["max"].as<double>();
            }
            if (defaults["torque"]) {
                default_params_.torque.min = defaults["torque"]["min"].as<double>();
                default_params_.torque.max = defaults["torque"]["max"].as<double>();
            }

            std::cout << "[MotorConfig] Loaded global defaults" << std::endl;
        }

        // Load individual motor parameters
        if (config["motors"]) {
            const auto& motors = config["motors"];

            for (const auto& motor_entry : motors) {
                std::string key = motor_entry.first.as<std::string>();
                const auto& motor = motor_entry.second;

                if (!motor["can_id"]) {
                    std::cerr << "[MotorConfig] Missing can_id for " << key << std::endl;
                    continue;
                }

                int can_id = motor["can_id"].as<int>();
                if (can_id < 1 || can_id > NUM_MOTORS) {
                    std::cerr << "[MotorConfig] Invalid CAN ID: " << can_id << std::endl;
                    continue;
                }

                int index = can_id - 1;
                motor_params_[index].can_id = can_id;

                // Override limits if specified
                if (motor["position"]) {
                    motor_params_[index].position.min = motor["position"]["min"].as<double>();
                    motor_params_[index].position.max = motor["position"]["max"].as<double>();
                }
                if (motor["velocity"]) {
                    motor_params_[index].velocity.min = motor["velocity"]["min"].as<double>();
                    motor_params_[index].velocity.max = motor["velocity"]["max"].as<double>();
                }
                if (motor["torque"]) {
                    motor_params_[index].torque.min = motor["torque"]["min"].as<double>();
                    motor_params_[index].torque.max = motor["torque"]["max"].as<double>();
                }
            }

            std::cout << "[MotorConfig] Loaded " << motors.size() << " motor configurations" << std::endl;
        }

        ApplyDefaults();
        std::cout << "[MotorConfig] Configuration loaded successfully" << std::endl;
        return true;

    } catch (const YAML::Exception& e) {
        std::cerr << "[MotorConfig] YAML Error: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "[MotorConfig] Error: " << e.what() << std::endl;
        return false;
    }
}

void MotorConfig::ApplyDefaults() {
    // Apply default limits to any motor that doesn't have explicit overrides
    for (int i = 0; i < NUM_MOTORS; i++) {
        // If a motor's limits are still at initial values, apply defaults
        // (This is handled by initialization, but this ensures consistency)
        if (motor_params_[i].position.min == 0 && motor_params_[i].position.max == 0) {
            motor_params_[i].position = default_params_.position;
        }
        if (motor_params_[i].velocity.min == 0 && motor_params_[i].velocity.max == 0) {
            motor_params_[i].velocity = default_params_.velocity;
        }
        if (motor_params_[i].torque.min == 0 && motor_params_[i].torque.max == 0) {
            motor_params_[i].torque = default_params_.torque;
        }
    }
}
