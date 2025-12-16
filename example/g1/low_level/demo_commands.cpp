#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>

// IDL
#include <unitree/idl/hg/LowCmd_.hpp>

static const std::string HG_CMD_TOPIC = "rt/lowcmd";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

const int G1_NUM_MOTOR = 29;

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
};

int main(int argc, char const *argv[]) {
    if (argc < 2) {
        std::cout << "Usage: demo_commands network_interface" << std::endl;
        exit(0);
    }

    std::string networkInterface = argv[1];

    // Initialize DDS channel
    ChannelFactory::Instance()->Init(0, networkInterface);

    // Create publisher
    ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();

    std::cout << "Demo Command Publisher Started" << std::endl;
    std::cout << "Sending test ankle commands..." << std::endl;

    // Send a series of test commands
    for (int step = 0; step < 100; step++) {
        LowCmd_ cmd;

        // Configure basic settings
        cmd.mode_pr() = 0;  // PR mode
        cmd.mode_machine() = 1;  // G1 type

        // Configure all motors (disable most)
        for (int i = 0; i < G1_NUM_MOTOR; i++) {
            cmd.motor_cmd().at(i).mode() = (i >= 4 && i <= 5) || (i >= 10 && i <= 11) ? 1 : 0;
            cmd.motor_cmd().at(i).tau() = 0.0;
            cmd.motor_cmd().at(i).q() = 0.0;
            cmd.motor_cmd().at(i).dq() = 0.0;
            cmd.motor_cmd().at(i).kp() = 0.0;
            cmd.motor_cmd().at(i).kd() = 0.0;
        }

        // Ankle motion pattern
        double t = step * 0.1;

        // Left ankle (ID 4,5)
        cmd.motor_cmd()[4].q() = 0.5 * std::sin(t);    // Pitch
        cmd.motor_cmd()[4].kp() = 40.0;
        cmd.motor_cmd()[5].q() = 0.2 * std::cos(t);    // Roll
        cmd.motor_cmd()[5].kp() = 40.0;

        // Right ankle (ID 10,11)
        cmd.motor_cmd()[10].q() = 0.5 * std::sin(t);   // Pitch
        cmd.motor_cmd()[10].kp() = 40.0;
        cmd.motor_cmd()[11].q() = -0.2 * std::cos(t);  // Roll
        cmd.motor_cmd()[11].kp() = 40.0;

        // Calculate and set CRC
        cmd.crc() = Crc32Core((uint32_t *)&cmd, (sizeof(cmd) >> 2) - 1);

        // Send command
        lowcmd_publisher_->Write(cmd);

        std::cout << "Step " << step << ": "
                  << "L_Pitch=" << std::fixed << std::setprecision(3) << cmd.motor_cmd()[4].q()
                  << ", L_Roll=" << cmd.motor_cmd()[5].q()
                  << ", R_Pitch=" << cmd.motor_cmd()[10].q()
                  << ", R_Roll=" << cmd.motor_cmd()[11].q()
                  << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Demo completed!" << std::endl;
    return 0;
}