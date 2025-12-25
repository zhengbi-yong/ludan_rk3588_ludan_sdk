#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <iomanip>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include "xixilowcmd/msg/low_cmd.hpp"
#include "xixilowcmd/msg/motor_cmd.hpp"

static const std::string ROS2_CMD_TOPIC = "/lowcmd";
const int G1_NUM_MOTOR = 30;

// CRC32 calculation function (must match the receiver)
inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else
                CRC32 <<= 1;
            if (data & xbit) CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

class MotorControlPublisher : public rclcpp::Node {
private:
    rclcpp::Publisher<xixilowcmd::msg::LowCmd>::SharedPtr lowcmd_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    MotorControlPublisher() : Node("motor_control_publisher") {
        // Create publisher for lowcmd topic
        lowcmd_publisher_ = this->create_publisher<xixilowcmd::msg::LowCmd>(
            ROS2_CMD_TOPIC, 10);

        // Create timer for 100Hz publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MotorControlPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Motor Control Publisher started");
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", ROS2_CMD_TOPIC.c_str());
    }

    void timer_callback() {
        // This callback is called at 100Hz
        // The actual motor commands are set by calling set_motor_command()
    }

    void publish_motor_command(const xixilowcmd::msg::LowCmd& cmd) {
        lowcmd_publisher_->publish(cmd);
    }
};

// Create a motor command for a single motor
xixilowcmd::msg::LowCmd create_single_motor_command(
    int motor_id,
    float q,
    float dq,
    float kp,
    float kd,
    float tau
) {
    xixilowcmd::msg::LowCmd cmd;

    // Initialize all motors to disabled state
    // Note: motor_cmd is a fixed-size array (std::array), not a vector
    for (int i = 0; i < G1_NUM_MOTOR; i++) {
        cmd.motor_cmd[i].id = i;
        cmd.motor_cmd[i].mode = 0;  // Disabled
        cmd.motor_cmd[i].q = 0.0f;
        cmd.motor_cmd[i].dq = 0.0f;
        cmd.motor_cmd[i].kp = 0.0f;
        cmd.motor_cmd[i].kd = 0.0f;
        cmd.motor_cmd[i].tau = 0.0f;
    }

    // Set the target motor
    if (motor_id >= 0 && motor_id < G1_NUM_MOTOR) {
        cmd.motor_cmd[motor_id].mode = 1;  // Enabled
        cmd.motor_cmd[motor_id].q = q;
        cmd.motor_cmd[motor_id].dq = dq;
        cmd.motor_cmd[motor_id].kp = kp;
        cmd.motor_cmd[motor_id].kd = kd;
        cmd.motor_cmd[motor_id].tau = tau;
    }

    return cmd;
}

void print_usage(const char* prog_name) {
    std::cout << "Usage: " << prog_name << " [options]" << std::endl;
    std::cout << "\nOptions:" << std::endl;
    std::cout << "  --id <n>           Motor ID (default: 9)" << std::endl;
    std::cout << "  --q <value>        Position in radians (default: 0.5)" << std::endl;
    std::cout << "  --dq <value>       Velocity in rad/s (default: 0.0)" << std::endl;
    std::cout << "  --kp <value>       Position gain (default: 1.0)" << std::endl;
    std::cout << "  --kd <value>       Velocity gain (default: 1.0)" << std::endl;
    std::cout << "  --tau <value>      Feedforward torque (default: 0.0)" << std::endl;
    std::cout << "  --duration <s>     Duration in seconds (default: 5)" << std::endl;
    std::cout << "  --rate <hz>        Publish rate in Hz (default: 100)" << std::endl;
    std::cout << "\nExample:" << std::endl;
    std::cout << "  " << prog_name << " --id 9 --q 0.5 --dq 0.0 --kp 1.0 --kd 1.0 --tau 0.0 --duration 5" << std::endl;
    std::cout << "\nNote: Make sure to run motor_controller_with_enable with --enable-motor-cmd first!" << std::endl;
}

int main(int argc, char** argv) {
    // Default parameters
    int motor_id = 9;
    float q = 0.5f;
    float dq = 0.0f;
    float kp = 1.0f;
    float kd = 1.0f;
    float tau = 0.0f;
    int duration_sec = 5;
    int publish_rate = 100;  // Hz

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        }
        else if (arg == "--id" && i + 1 < argc) {
            motor_id = std::atoi(argv[++i]);
        }
        else if (arg == "--q" && i + 1 < argc) {
            q = std::atof(argv[++i]);
        }
        else if (arg == "--dq" && i + 1 < argc) {
            dq = std::atof(argv[++i]);
        }
        else if (arg == "--kp" && i + 1 < argc) {
            kp = std::atof(argv[++i]);
        }
        else if (arg == "--kd" && i + 1 < argc) {
            kd = std::atof(argv[++i]);
        }
        else if (arg == "--tau" && i + 1 < argc) {
            tau = std::atof(argv[++i]);
        }
        else if (arg == "--duration" && i + 1 < argc) {
            duration_sec = std::atoi(argv[++i]);
        }
        else if (arg == "--rate" && i + 1 < argc) {
            publish_rate = std::atoi(argv[++i]);
        }
    }

    // Print configuration
    std::cout << "========================================" << std::endl;
    std::cout << "Motor Control Publisher" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Motor ID:     " << motor_id << std::endl;
    std::cout << "Position (q): " << std::fixed << std::setprecision(3) << q << " rad" << std::endl;
    std::cout << "Velocity (dq):" << std::fixed << std::setprecision(3) << dq << " rad/s" << std::endl;
    std::cout << "Kp:           " << std::fixed << std::setprecision(3) << kp << std::endl;
    std::cout << "Kd:           " << std::fixed << std::setprecision(3) << kd << std::endl;
    std::cout << "Tau:          " << std::fixed << std::setprecision(3) << tau << " Nm" << std::endl;
    std::cout << "Duration:     " << duration_sec << " seconds" << std::endl;
    std::cout << "Publish Rate: " << publish_rate << " Hz" << std::endl;
    std::cout << "========================================" << std::endl;

    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create the motor control publisher node
    auto motor_publisher = std::make_shared<MotorControlPublisher>();

    // Create motor command
    auto cmd = create_single_motor_command(motor_id, q, dq, kp, kd, tau);

    // Calculate sleep time based on publish rate
    std::chrono::microseconds sleep_time(static_cast<int>(1000000.0 / publish_rate));

    std::cout << "\n>>> Starting motor control <<<" << std::endl;
    std::cout << "Press Ctrl+C to stop early..." << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    int iteration = 0;

    // Main control loop
    while (rclcpp::ok()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            current_time - start_time).count();

        if (elapsed >= duration_sec) {
            std::cout << "\n>>> Duration reached, stopping <<<" << std::endl;
            break;
        }

        // Publish motor command
        motor_publisher->publish_motor_command(cmd);

        // Print status every second
        if (iteration % publish_rate == 0) {
            std::cout << "[" << elapsed << "s] Motor " << motor_id
                      << " | q=" << std::fixed << std::setprecision(3) << q
                      << " | dq=" << dq
                      << " | kp=" << kp
                      << " | kd=" << kd
                      << " | tau=" << tau
                      << std::endl;
        }

        // Spin once to handle callbacks
        rclcpp::spin_some(motor_publisher);

        // Sleep for the remainder of the cycle
        std::this_thread::sleep_for(sleep_time);
        iteration++;
    }

    // Send zero command before exiting
    std::cout << "\n>>> Sending zero command to disable motor <<<" << std::endl;
    auto zero_cmd = create_single_motor_command(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 10; i++) {
        motor_publisher->publish_motor_command(zero_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << ">>> Motor control stopped <<<" << std::endl;

    // Cleanup
    rclcpp::shutdown();
    return 0;
}
