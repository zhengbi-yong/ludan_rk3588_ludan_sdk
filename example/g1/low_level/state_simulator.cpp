#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>

// IDL
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

static const std::string HG_STATE_TOPIC = "rt/lowstate";
static const std::string HG_IMU_TORSO = "rt/secondary_imu";

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

class StateSimulator {
private:
    ChannelPublisherPtr<LowState_> lowstate_publisher_;
    ChannelPublisherPtr<IMUState_> imu_publisher_;
    double time_;
    uint8_t mode_machine_;

public:
    StateSimulator() : time_(0.0), mode_machine_(1) {  // G1 type = 1

        // Create publishers
        lowstate_publisher_.reset(new ChannelPublisher<LowState_>(HG_STATE_TOPIC));
        lowstate_publisher_->InitChannel();

        imu_publisher_.reset(new ChannelPublisher<IMUState_>(HG_IMU_TORSO));
        imu_publisher_->InitChannel();

        std::cout << "State Simulator Initialized" << std::endl;
        std::cout << "Publishing simulated robot state..." << std::endl;
    }

    void PublishSimulatedState() {
        // Create simulated IMU state
        IMUState_ imu_state;
        imu_state.rpy()[0] = 0.0;  // roll
        imu_state.rpy()[1] = 0.0;  // pitch
        imu_state.rpy()[2] = 0.0;  // yaw
        imu_state.gyroscope()[0] = 0.0;  // wx
        imu_state.gyroscope()[1] = 0.0;  // wy
        imu_state.gyroscope()[2] = 0.0;  // wz
        imu_state.accelerometer()[0] = 0.0;
        imu_state.accelerometer()[1] = 0.0;
        imu_state.accelerometer()[2] = 9.81;  // gravity
        imu_publisher_->Write(imu_state);

        // Create simulated low state
        LowState_ low_state;
        low_state.mode_machine() = mode_machine_;
        low_state.crc() = 0;  // Will be calculated later

        // Simulate motor states (all motors initialized to zero)
        for (int i = 0; i < G1_NUM_MOTOR; i++) {
            low_state.motor_state()[i].mode() = 0;
            low_state.motor_state()[i].q() = 0.0;
            low_state.motor_state()[i].dq() = 0.0;
            low_state.motor_state()[i].ddq() = 0.0;
            low_state.motor_state()[i].tau_est() = 0.0;
            low_state.motor_state()[i].temperature()[0] = 25;
            low_state.motor_state()[i].temperature()[1] = 25;
            low_state.motor_state()[i].motorstate() = 0;  // No error
            low_state.motor_state()[i].reserve()[0] = 0;
            low_state.motor_state()[i].reserve()[1] = 0;
            low_state.motor_state()[i].reserve()[2] = 0;
            low_state.motor_state()[i].reserve()[3] = 0;
        }

        // Set some initial positions for more realistic simulation
        // Standing position for legs
        low_state.motor_state()[0].q() = 0.0;    // LeftHipPitch
        low_state.motor_state()[1].q() = 0.0;    // LeftHipRoll
        low_state.motor_state()[2].q() = 0.0;    // LeftHipYaw
        low_state.motor_state()[3].q() = 0.0;    // LeftKnee
        low_state.motor_state()[4].q() = 0.0;    // LeftAnklePitch
        low_state.motor_state()[5].q() = 0.0;    // LeftAnkleRoll
        low_state.motor_state()[6].q() = 0.0;    // RightHipPitch
        low_state.motor_state()[7].q() = 0.0;    // RightHipRoll
        low_state.motor_state()[8].q() = 0.0;    // RightHipYaw
        low_state.motor_state()[9].q() = 0.0;    // RightKnee
        low_state.motor_state()[10].q() = 0.0;   // RightAnklePitch
        low_state.motor_state()[11].q() = 0.0;   // RightAnkleRoll

        // Arms in neutral position
        low_state.motor_state()[15].q() = 0.0;   // LeftShoulderPitch
        low_state.motor_state()[16].q() = 0.0;   // LeftShoulderRoll
        low_state.motor_state()[17].q() = 0.0;   // LeftShoulderYaw
        low_state.motor_state()[18].q() = 0.0;   // LeftElbow
        low_state.motor_state()[19].q() = 0.0;   // LeftWristRoll
        low_state.motor_state()[22].q() = 0.0;   // RightShoulderPitch
        low_state.motor_state()[23].q() = 0.0;   // RightShoulderRoll
        low_state.motor_state()[24].q() = 0.0;   // RightShoulderYaw
        low_state.motor_state()[25].q() = 0.0;   // RightElbow
        low_state.motor_state()[26].q() = 0.0;   // RightWristRoll

        // Simulate wireless remote data (all zeros)
        for (int i = 0; i < 40; i++) {
            low_state.wireless_remote()[i] = 0;
        }

        // Calculate CRC
        low_state.crc() = Crc32Core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1);

        // Publish state
        lowstate_publisher_->Write(low_state);

        time_ += 0.002;  // 2ms period
    }

    void Run() {
        while (true) {
            PublishSimulatedState();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));  // 500Hz
        }
    }
};

int main(int argc, char const *argv[]) {
    if (argc < 2) {
        std::cout << "Usage: state_simulator network_interface" << std::endl;
        exit(0);
    }

    std::string networkInterface = argv[1];

    // Initialize DDS channel
    ChannelFactory::Instance()->Init(0, networkInterface);

    // Create and run simulator
    StateSimulator simulator;
    simulator.Run();

    return 0;
}