#include <ros/ros.h>
#include <motor_controller/MotorFeedback.h>
#include <iomanip>

class MotorFeedbackSubscriber {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    // 存储每个电机的最新数据
    std::map<int, motor_controller::MotorFeedback> motor_data_;

public:
    MotorFeedbackSubscriber() {
        // 订阅motor feedback话题
        sub_ = nh_.subscribe("/motor_feedback", 10,
                          &MotorFeedbackSubscriber::feedbackCallback, this);

        ROS_INFO("Motor Feedback Subscriber started");
        ROS_INFO("Listening to /motor_feedback");
    }

    void feedbackCallback(const motor_controller::MotorFeedback::ConstPtr& msg) {
        // 存储数据
        motor_data_[msg->motor_id] = *msg;

        // 实时打印每个反馈（可选）
        ROS_INFO_STREAM("Motor " << msg->motor_id
                        << " | Pos: " << std::fixed << std::setprecision(3) << msg->position
                        << " rad | Vel: " << std::setprecision(3) << msg->velocity
                        << " rad/s | Torque: " << std::setprecision(2) << msg->torque << " Nm");
    }

    void printStatusSummary() {
        if (motor_data_.empty()) {
            ROS_INFO("No motor data received");
            return;
        }

        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "MOTOR STATUS SUMMARY (Last received data)" << std::endl;
        std::cout << std::string(60, '=') << std::endl;

        std::cout << std::setw(8) << "Motor"
                  << std::setw(12) << "Position"
                  << std::setw(12) << "Velocity"
                  << std::setw(10) << "Torque"
                  << std::setw(10) << "Temp(MOS)"
                  << std::setw(10) << "Status" << std::endl;
        std::cout << std::string(60, '-') << std::endl;

        for (const auto& pair : motor_data_) {
            const auto& msg = pair.second;
            std::cout << std::setw(6) << msg.motor_id
                      << std::setw(12) << std::fixed << std::setprecision(3) << msg.position
                      << std::setw(12) << std::setprecision(3) << msg.velocity
                      << std::setw(10) << std::setprecision(2) << msg.torque
                      << std::setw(10) << std::setprecision(1) << msg.temperature_mos;

            if (msg.error) {
                std::cout << std::setw(10) << "ERR" << std::endl;
            } else {
                std::cout << std::setw(10) << "OK" << std::endl;
            }
        }
        std::cout << std::string(60, '=') << std::endl;
    }

    motor_controller::MotorFeedback getMotorData(int motor_id) {
        return motor_data_[motor_id];
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_feedback_subscriber");

    MotorFeedbackSubscriber subscriber;

    ros::Rate rate(1);  // 1 Hz
    while (ros::ok()) {
        subscriber.printStatusSummary();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}