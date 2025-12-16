#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

// DDS
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/LowCmd_.hpp>

static const std::string HG_CMD_TOPIC = "rt/lowcmd";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

const int G1_NUM_MOTOR = 29;

// Joint names for better readability
const std::string joint_names[G1_NUM_MOTOR] = {
    "LeftHipPitch", "LeftHipRoll", "LeftHipYaw", "LeftKnee", "LeftAnklePitch", "LeftAnkleRoll",
    "RightHipPitch", "RightHipRoll", "RightHipYaw", "RightKnee", "RightAnklePitch", "RightAnkleRoll",
    "WaistYaw", "WaistRoll", "WaistPitch",
    "LeftShoulderPitch", "LeftShoulderRoll", "LeftShoulderYaw", "LeftElbow", "LeftWristRoll", "LeftWristPitch", "LeftWristYaw",
    "RightShoulderPitch", "RightShoulderRoll", "RightShoulderYaw", "RightElbow", "RightWristRoll", "RightWristPitch", "RightWristYaw"
};

class CommandMonitor {
private:
    ChannelSubscriberPtr<LowCmd_> lowcmd_subscriber_;
    uint64_t message_count_;
    std::chrono::steady_clock::time_point start_time_;

public:
    CommandMonitor() : message_count_(0) {
        start_time_ = std::chrono::steady_clock::now();

        // Create subscriber to monitor commands
        lowcmd_subscriber_.reset(new ChannelSubscriber<LowCmd_>(HG_CMD_TOPIC));
        lowcmd_subscriber_->InitChannel(std::bind(&CommandMonitor::CommandHandler, this, std::placeholders::_1), 1);
    }

    void CommandHandler(const void *message) {
        const LowCmd_ *cmd = static_cast<const LowCmd_*>(message);
        message_count_++;

        // Calculate timestamp
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count() / 1000.0;

        // Clear screen for better readability (optional)
        // system("clear");

        std::cout << "\n=== COMMAND MONITOR ===" << std::endl;
        std::cout << "Time: " << std::fixed << std::setprecision(2) << elapsed << "s" << std::endl;
        std::cout << "Message Count: " << message_count_ << std::endl;
        std::cout << "Mode PR: " << static_cast<int>(cmd->mode_pr()) << " (0=PR, 1=AB)" << std::endl;
        std::cout << "Mode Machine: " << static_cast<int>(cmd->mode_machine()) << std::endl;
        std::cout << "CRC: 0x" << std::hex << cmd->crc() << std::dec << std::endl;

        std::cout << "\n=== MOTOR COMMANDS ===" << std::endl;
        std::cout << std::setw(20) << "Joint Name"
                  << std::setw(10) << "Mode"
                  << std::setw(10) << "Target Pos"
                  << std::setw(10) << "Target Vel"
                  << std::setw(10) << "Kp"
                  << std::setw(10) << "Kd"
                  << std::setw(10) << "Tau FF" << std::endl;
        std::cout << std::string(90, '-') << std::endl;

        for (int i = 0; i < G1_NUM_MOTOR; i++) {
            const auto &motor_cmd = cmd->motor_cmd()[i];

            // Only show non-zero commands for ankles to reduce noise
            if ((i >= 4 && i <= 5) || (i >= 10 && i <= 11) ||  // Ankle joints
                motor_cmd.q() != 0 || motor_cmd.dq() != 0 ||
                motor_cmd.tau() != 0 || motor_cmd.mode() != 0) {

                std::cout << std::setw(20) << joint_names[i]
                          << std::setw(10) << static_cast<int>(motor_cmd.mode())
                          << std::setw(10) << std::fixed << std::setprecision(3) << motor_cmd.q()
                          << std::setw(10) << std::fixed << std::setprecision(3) << motor_cmd.dq()
                          << std::setw(10) << std::fixed << std::setprecision(1) << motor_cmd.kp()
                          << std::setw(10) << std::fixed << std::setprecision(1) << motor_cmd.kd()
                          << std::setw(10) << std::fixed << std::setprecision(3) << motor_cmd.tau()
                          << std::endl;
            }
        }

        // Special focus on ankle joints
        std::cout << "\n=== ANKLE JOINTS DETAIL ===" << std::endl;
        std::cout << "Left Ankle Pitch (ID 4):  pos=" << std::fixed << std::setprecision(3) << cmd->motor_cmd()[4].q()
                  << ", vel=" << cmd->motor_cmd()[4].dq() << ", kp=" << cmd->motor_cmd()[4].kp() << std::endl;
        std::cout << "Left Ankle Roll (ID 5):   pos=" << std::fixed << std::setprecision(3) << cmd->motor_cmd()[5].q()
                  << ", vel=" << cmd->motor_cmd()[5].dq() << ", kp=" << cmd->motor_cmd()[5].kp() << std::endl;
        std::cout << "Right Ankle Pitch (ID 10): pos=" << std::fixed << std::setprecision(3) << cmd->motor_cmd()[10].q()
                  << ", vel=" << cmd->motor_cmd()[10].dq() << ", kp=" << cmd->motor_cmd()[10].kp() << std::endl;
        std::cout << "Right Ankle Roll (ID 11):  pos=" << std::fixed << std::setprecision(3) << cmd->motor_cmd()[11].q()
                  << ", vel=" << cmd->motor_cmd()[11].dq() << ", kp=" << cmd->motor_cmd()[11].kp() << std::endl;

        std::cout << std::endl;
    }
};

int main(int argc, char const *argv[]) {
    if (argc < 2) {
        std::cout << "Usage: command_monitor network_interface" << std::endl;
        exit(0);
    }

    std::string networkInterface = argv[1];

    // Initialize DDS channel
    ChannelFactory::Instance()->Init(0, networkInterface);

    std::cout << "Command Monitor Started..." << std::endl;
    std::cout << "Listening for commands on topic: " << HG_CMD_TOPIC << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl << std::endl;

    // Create monitor
    CommandMonitor monitor;

    // Keep running
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}