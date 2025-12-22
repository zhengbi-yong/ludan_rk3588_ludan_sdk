#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <dds/dds.hpp>

// Define a simple command structure with position, velocity, and torque
struct CommandData {
    std::vector<double> position;    // Joint positions
    std::vector<double> velocity;    // Joint velocities
    std::vector<double> torque;      // Joint torques
};

int main(int argc, char *argv[]) {
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0] << " <interface> <amplitude> <frequency> <duration> [num_joints]" << std::endl;
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

    std::cout << "DDS Native Command Publisher" << std::endl;
    std::cout << "Interface: " << interface << std::endl;
    std::cout << "Amplitude: " << amplitude << std::endl;
    std::cout << "Frequency: " << frequency << " Hz" << std::endl;
    std::cout << "Duration: " << duration << " seconds" << std::endl;
    std::cout << "Number of joints: " << num_joints << std::endl;

    try {
        // Create DDS DomainParticipant
        dds::domain::DomainParticipant participant(0); // Domain ID 0

        // Create Topic
        dds::topic::Topic<CommandData> topic(participant, "lowcmd");

        // Create Publisher
        dds::pub::Publisher publisher(participant);

        // Create DataWriter
        dds::pub::DataWriter<CommandData> writer(publisher, topic);

        std::cout << "DDS Publisher initialized successfully" << std::endl;

        // Create command data structure
        CommandData cmd;
        cmd.position.resize(num_joints, 0.0);
        cmd.velocity.resize(num_joints, 0.0);
        cmd.torque.resize(num_joints, 0.0);

        // Square wave generation
        double period = 1.0 / frequency;
        double half_period = period / 2.0;

        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + std::chrono::seconds(static_cast<int>(duration));

        int switch_count = 0;
        double current_value = amplitude;

        std::cout << "Starting square wave command generation..." << std::endl;

        while (std::chrono::steady_clock::now() < end_time) {
            // Update command data with current square wave value
            for (int i = 0; i < num_joints; ++i) {
                // Apply square wave with different phases for different joints
                double phase_offset = static_cast<double>(i) / num_joints * 2.0 * M_PI;
                double joint_value = current_value * std::sin(phase_offset);

                cmd.position[i] = joint_value;
                cmd.velocity[i] = 0.0; // For square wave, velocity is 0 (instant change)
                cmd.torque[i] = joint_value * 0.5; // Simple torque proportional to position
            }

            // Publish command
            writer.write(cmd);

            std::cout << "[" << std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time).count() << "ms] "
                << "Published command - Value: " << current_value
                << " Switch count: " << switch_count << std::endl;

            // Wait for half period
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(half_period * 1000)));

            // Switch square wave value
            current_value = (current_value == amplitude) ? -amplitude : amplitude;
            switch_count++;
        }

        std::cout << "Command generation completed. Published " << switch_count << " switches." << std::endl;

    } catch (const dds::core::Exception& e) {
        std::cerr << "DDS Exception: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}