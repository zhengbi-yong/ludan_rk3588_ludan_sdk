#include <iostream>
#include <thread>
#include <chrono>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include "xixilowcmd/msg/motor_cmd.hpp"

static const std::string MOTOR_ENABLE_TOPIC = "/motor_enable";

class MotorEnablePublisher : public rclcpp::Node {
private:
    rclcpp::Publisher<xixilowcmd::msg::MotorCmd>::SharedPtr enable_publisher_;

public:
    MotorEnablePublisher() : Node("motor_enable_publisher") {
        // Create publisher for motor enable topic
        // Note: Uses MotorCmd message type, not Bool!
        enable_publisher_ = this->create_publisher<xixilowcmd::msg::MotorCmd>(
            MOTOR_ENABLE_TOPIC, 10);

        RCLCPP_INFO(this->get_logger(), "Motor Enable Publisher started");
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", MOTOR_ENABLE_TOPIC.c_str());
        RCLCPP_INFO(this->get_logger(), "Message type: xixilowcmd::msg::MotorCmd");
    }

    void publish_enable(int motor_id, bool enable) {
        xixilowcmd::msg::MotorCmd msg;
        msg.id = motor_id;
        msg.mode = 0;  // Not used by handler
        // IMPORTANT: Handler checks msg->q for command value!
        // q=1 means enable, q=0 means disable
        msg.q = enable ? 1.0f : 0.0f;
        msg.dq = 0.0f;
        msg.kp = 0.0f;
        msg.kd = 0.0f;
        msg.tau = 0.0f;

        enable_publisher_->publish(msg);

        if (enable) {
            RCLCPP_INFO(this->get_logger(), "Published: ENABLE motor %d (q=1)", motor_id);
        } else {
            RCLCPP_INFO(this->get_logger(), "Published: DISABLE motor %d (q=0)", motor_id);
        }
    }
};

int main(int argc, char** argv) {
    // Parse command line
    int motor_id = 0;  // 0 = all motors
    bool enable = true;  // default: enable
    int repeat = 1;      // default: send once

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if ((arg == "--id") && i + 1 < argc) {
            motor_id = std::atoi(argv[++i]);
        } else if (arg == "--disable" || arg == "-d") {
            enable = false;
        } else if (arg == "--repeat" && i + 1 < argc) {
            repeat = std::atoi(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << "\nOptions:" << std::endl;
            std::cout << "  --id <n>         Motor ID to enable (0=all motors, default: 0)" << std::endl;
            std::cout << "  --disable, -d    Send disable command (default: enable)" << std::endl;
            std::cout << "  --repeat <n>     Repeat the command n times (default: 1)" << std::endl;
            std::cout << "\nExample:" << std::endl;
            std::cout << "  " << argv[0] << "                    # Enable all motors" << std::endl;
            std::cout << "  " << argv[0] << " --id 9            # Enable motor 9" << std::endl;
            std::cout << "  " << argv[0] << " --disable         # Disable all motors" << std::endl;
            std::cout << "  " << argv[0] << " --id 9 --repeat 5 # Enable motor 9, 5 times" << std::endl;
            return 0;
        }
    }

    std::cout << "========================================" << std::endl;
    std::cout << "Motor Enable/Disable Publisher" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Motor ID: " << motor_id << (motor_id == 0 ? " (all motors)" : "") << std::endl;
    std::cout << "Command:  " << (enable ? "ENABLE" : "DISABLE") << std::endl;
    std::cout << "Repeat:   " << repeat << " time(s)" << std::endl;
    std::cout << "========================================" << std::endl;

    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create publisher
    auto motor_enable_pub = std::make_shared<MotorEnablePublisher>();

    // Wait a bit for publisher to be ready
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Publish enable command
    for (int i = 0; i < repeat; i++) {
        motor_enable_pub->publish_enable(motor_id, enable);
        if (repeat > 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    // Keep alive for a short time to ensure message is delivered
    std::cout << "\nWaiting for message to be delivered..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "Done!" << std::endl;

    // Cleanup
    rclcpp::shutdown();
    return 0;
}
