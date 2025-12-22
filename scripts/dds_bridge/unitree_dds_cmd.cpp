#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>

using namespace unitree::robot;
using namespace unitree::common;

// Define the command data structure for position/velocity/torque
struct LowCmdData {
    double timestamp;
    std::vector<double> position;    // Joint positions
    std::vector<double> velocity;    // Joint velocities
    std::vector<double> torque;      // Joint torques
};

int main(int argc, char *argv[]) {
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0] << " <interface> <amplitude> <frequency> <duration> [num_joints]" << std::endl;
        std::cerr << "Example: " << argv[0] << " eth0 1.0 2.0 30 12" << std::endl;
        return 1;
    }

    std::string interface = argv[1];
    double amplitude = std::stod(argv[2]);
    double frequency = std::stod(argv[3]);
    double duration = std::stod(argv[4]);
    int num_joints = 12; // default
    if (argc >= 6) {
        num_joints = std::stoi(argv[5]);
    }

    std::cout << "Unitree DDS Command Publisher" << std::endl;
    std::cout << "Interface: " << interface << std::endl;
    std::cout << "Amplitude: " << amplitude << std::endl;
    std::cout << "Frequency: " << frequency << " Hz" << std::endl;
    std::cout << "Duration: " << duration << " seconds" << std::endl;
    std::cout << "Number of joints: " << num_joints << std::endl;
    std::cout << "Data format: position/velocity/torque" << std::endl;

    try {
        // Initialize Unitree DDS Channel Factory
        ChannelFactory::Instance()->Init(0); // Domain ID 0

        // Create publisher for /lowcmd topic
        ChannelPublisher<LowCmdData> publisher("/lowcmd");
        publisher.InitChannel();

        std::cout << "DDS Publisher initialized successfully" << std::endl;
        std::cout << "Publishing to topic: /lowcmd" << std::endl;

        // Prepare command data structure
        LowCmdData cmd;
        cmd.position.resize(num_joints, 0.0);
        cmd.velocity.resize(num_joints, 0.0);
        cmd.torque.resize(num_joints, 0.0);

        // Square wave generation
        double period = 1.0 / frequency;
        double half_period = period / 2.0;

        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + std::chrono::milliseconds(static_cast<int>(duration * 1000));

        int switch_count = 0;
        double current_value = amplitude;

        std::cout << "Starting square wave command generation..." << std::endl;
        std::cout << "Period: " << period << "s, Half period: " << half_period << "s" << std::endl;

        while (std::chrono::steady_clock::now() < end_time) {
            // Update timestamp
            cmd.timestamp = GetCurrentTimeMillisecond();

            // Update command data with current square wave value
            for (int i = 0; i < num_joints; ++i) {
                // Apply square wave with different patterns for different joints
                double joint_factor = 1.0;

                // Joint-specific patterns
                switch (i % 4) {
                    case 0: // Joint in phase
                        joint_factor = 1.0;
                        break;
                    case 1: // Joint opposite phase
                        joint_factor = -1.0;
                        break;
                    case 2: // Joint half amplitude
                        joint_factor = 0.5;
                        break;
                    case 3: // Joint double amplitude
                        joint_factor = 1.5;
                        break;
                }

                double joint_value = current_value * joint_factor;

                cmd.position[i] = joint_value;
                cmd.velocity[i] = 0.0; // For square wave, velocity is 0 (instant change)
                cmd.torque[i] = joint_value * 0.3; // Simple torque proportional to position
            }

            // Publish command
            publisher.Write(cmd);

            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time).count();

            std::cout << "[" << elapsed_ms << "ms] Published /lowcmd - Value: " << current_value
                     << " Switch #" << switch_count << " - Positions: [";

            // Show first 4 joint positions as example
            for (int i = 0; i < std::min(4, num_joints); ++i) {
                std::cout << cmd.position[i];
                if (i < std::min(3, num_joints-1)) std::cout << ", ";
            }
            if (num_joints > 4) std::cout << "...";
            std::cout << "]" << std::endl;

            // Wait for half period
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(half_period * 1000)));

            // Switch square wave value
            current_value = (current_value == amplitude) ? -amplitude : amplitude;
            switch_count++;
        }

        std::cout << "Command generation completed." << std::endl;
        std::cout << "Total switches: " << switch_count << std::endl;
        std::cout << "Total messages published: " << switch_count << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}