// motor_feedback_subscriber.cpp
// Subscribes to /motor_feedback topic and displays motor status

#include <rclcpp/rclcpp.hpp>
#include <motor_feedback/msg/motor_feedback.hpp>
#include <map>
#include <iomanip>
#include <algorithm>
#include <sstream>

class MotorFeedbackSubscriber : public rclcpp::Node {
public:
    MotorFeedbackSubscriber() : Node("motor_feedback_subscriber") {
        // Declare parameters
        this->declare_parameter("display_rate", 1.0);  // Hz
        this->declare_parameter("show_raw", false);
        this->declare_parameter("show_decoded", true);  // Show decoded physical values

        // Get parameters
        double display_rate = this->get_parameter("display_rate").as_double();
        show_raw_ = this->get_parameter("show_raw").as_bool();
        show_decoded_ = this->get_parameter("show_decoded").as_bool();

        // Create subscription
        subscription_ = this->create_subscription<motor_feedback::msg::MotorFeedback>(
            "/motor_feedback", 10,
            std::bind(&MotorFeedbackSubscriber::feedback_callback, this, std::placeholders::_1));

        // Create timer for periodic display
        auto period = std::chrono::duration<double>(1.0 / display_rate);
        display_timer_ = this->create_wall_timer(
            period, std::bind(&MotorFeedbackSubscriber::display_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Motor Feedback Subscriber started");
        RCLCPP_INFO(this->get_logger(), "Listening to /motor_feedback");
        RCLCPP_INFO(this->get_logger(), "Display rate: %.1f Hz", display_rate);
        RCLCPP_INFO(this->get_logger(), "Show decoded: %s, Show raw: %s",
                    show_decoded_ ? "YES" : "NO", show_raw_ ? "YES" : "NO");

        // Initialize last update time
        last_update_time_ = this->now();
    }

private:
    void feedback_callback(const motor_feedback::msg::MotorFeedback::SharedPtr msg) {
        // Store latest data
        std::lock_guard<std::mutex> lock(motor_data_mutex_);
        motor_data_[msg->motor_id] = *msg;
        last_update_time_ = this->now();
    }

    void display_timer_callback() {
        std::lock_guard<std::mutex> lock(motor_data_mutex_);

        if (motor_data_.empty()) {
            static int empty_count = 0;
            if (++empty_count % 5 == 0) {  // Print every 5 seconds
                RCLCPP_WARN(this->get_logger(), "No motor data received yet...");
            }
            return;
        }

        // Clear screen and print header
        std::cout << "\033[2J\033[H";  // Clear screen and move cursor to top
        std::cout << "========================================" << std::endl;
        std::cout << "     DM Motor Feedback Status (ID 1-30)" << std::endl;
        std::cout << "========================================" << std::endl;

        // Print header with decoded values
        if (show_decoded_) {
            std::cout << std::setw(6) << "ID"
                      << std::setw(14) << "Pos(rad)"
                      << std::setw(14) << "Vel(rad/s)"
                      << std::setw(12) << "Tau(Nm)"
                      << std::setw(8) << "T_MOS"
                      << std::setw(8) << "T_Rotor"
                      << std::setw(10) << "Error"
                      << std::endl;
        } else {
            std::cout << std::setw(6) << "ID"
                      << std::setw(12) << "Pos(raw)"
                      << std::setw(12) << "Vel(raw)"
                      << std::setw(10) << "Tau(raw)"
                      << std::setw(8) << "T_MOS"
                      << std::setw(8) << "T_Rotor"
                      << std::setw(10) << "Error"
                      << std::endl;
        }
        std::cout << std::string(80, '-') << std::endl;

        // Print data for each motor
        for (const auto& pair : motor_data_) {
            const auto& msg = pair.second;

            if (show_decoded_) {
                // Show decoded physical values
                std::cout << std::setw(6) << msg.motor_id
                          << std::setw(14) << std::fixed << std::setprecision(3) << msg.position_rad
                          << std::setw(14) << std::setprecision(2) << msg.velocity_rad_s
                          << std::setw(12) << std::setprecision(2) << msg.torque_nm
                          << std::setw(8) << static_cast<int>(msg.temp_mos)
                          << std::setw(8) << static_cast<int>(msg.temp_rotor);
            } else {
                // Show raw values
                std::cout << std::setw(6) << msg.motor_id
                          << std::setw(12) << msg.position
                          << std::setw(12) << msg.velocity
                          << std::setw(10) << msg.torque
                          << std::setw(8) << static_cast<int>(msg.temp_mos)
                          << std::setw(8) << static_cast<int>(msg.temp_rotor);
            }

            // Error status
            std::string error_str;
            switch (msg.error) {
                case 0x0: error_str = "Disabled"; break;
                case 0x1: error_str = "Enabled"; break;
                case 0x8: error_str = "OverVolt"; break;
                case 0x9: error_str = "UnderVolt"; break;
                case 0xA: error_str = "OverCurr"; break;
                case 0xB: error_str = "MOS_OverT"; break;
                case 0xC: error_str = "Coil_OverT"; break;
                case 0xD: error_str = "CommLost"; break;
                case 0xE: error_str = "Overload"; break;
                default:  {
                    std::ostringstream oss;
                    oss << "Err0x" << std::hex << msg.error;
                    error_str = oss.str();
                    break;
                }
            }
            std::cout << std::setw(10) << error_str << std::endl;

            // Show raw CAN data if enabled
            if (show_raw_) {
                std::cout << "  RAW CAN: ";
                for (size_t i = 0; i < msg.can_data.size(); i++) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0')
                              << static_cast<int>(msg.can_data[i]) << " ";
                }
                std::cout << std::dec << std::setfill(' ') << std::endl;
            }

            // Show raw values alongside decoded values (if both enabled)
            if (show_decoded_ && show_raw_) {
                std::cout << "  Raw: Pos=" << msg.position
                          << " Vel=" << msg.velocity
                          << " Tau=" << msg.torque << std::endl;
            }
        }

        // Print summary
        std::cout << std::string(80, '-') << std::endl;
        std::cout << "Total motors: " << motor_data_.size();

        // Count by status
        int enabled_count = 0, error_count = 0;
        for (const auto& pair : motor_data_) {
            if (pair.second.error == 0x1) enabled_count++;
            if (pair.second.error >= 0x8) error_count++;
        }
        std::cout << " | Enabled: " << enabled_count
                  << " | Errors: " << error_count;

        // Print age of data
        auto now = this->now();
        auto age = now - last_update_time_;
        std::cout << " | Last update: " << age.seconds() << "s ago" << std::endl;
        std::cout << "========================================" << std::endl;
    }

    rclcpp::Subscription<motor_feedback::msg::MotorFeedback>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr display_timer_;

    std::map<int, motor_feedback::msg::MotorFeedback> motor_data_;
    std::mutex motor_data_mutex_;
    rclcpp::Time last_update_time_;
    bool show_raw_;
    bool show_decoded_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MotorFeedbackSubscriber>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
