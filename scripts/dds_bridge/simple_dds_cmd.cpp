#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <cmath>

#include "dds/dds.hpp"

using namespace org::eclipse::cyclonedds;

// Simple command structure
struct LowCmd {
    double timestamp;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> torque;
};

// Topic type definition
struct LowCmdTopic {
    double timestamp;
    double pos[12];  // Fixed size array for 12 joints
    double vel[12];
    double tau[12];
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
    int num_joints = (argc >= 6) ? std::stoi(argv[5]) : 12;

    std::cout << "Simple DDS Command Publisher" << std::endl;
    std::cout << "Interface: " << interface << std::endl;
    std::cout << "Amplitude: " << amplitude << std::endl;
    std::cout << "Frequency: " << frequency << " Hz" << std::endl;
    std::cout << "Duration: " << duration << " seconds" << std::endl;
    std::cout << "Number of joints: " << num_joints << std::endl;

    try {
        // Create domain participant
        dds::domain::DomainParticipant participant(domain::default_id());

        // Register topic type
        dds::topic::qos::TopicQos topic_qos;
        topic_qos = participant.default_topic_qos();
        topic_qos.reliability(dds::core::policy::Reliability::Reliable());
        topic_qos.durability(dds::core::policy::Durability::Volatile());

        // Create topic
        dds::topic::Topic<LowCmdTopic> topic(participant, "lowcmd", topic_qos);

        // Create publisher
        dds::pub::Publisher publisher(participant);

        // Create writer
        dds::pub::DataWriter<LowCmdTopic> writer(publisher, topic);

        std::cout << "DDS Publisher initialized successfully" << std::endl;

        // Prepare data
        LowCmdTopic cmd;
        double period = 1.0 / frequency;
        double half_period = period / 2.0;
        double current_value = amplitude;
        int switch_count = 0;

        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + std::chrono::milliseconds(static_cast<int>(duration * 1000));

        std::cout << "Starting square wave generation..." << std::endl;
        std::cout << "Press Ctrl+C to stop early" << std::endl;

        while (std::chrono::steady_clock::now() < end_time) {
            // Update timestamp
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time).count();
            cmd.timestamp = static_cast<double>(elapsed) / 1000.0;

            // Update joint data
            for (int i = 0; i < num_joints; ++i) {
                double joint_factor = 1.0;

                // Different patterns for different joints
                switch (i % 4) {
                    case 0: joint_factor = 1.0; break;   // Normal
                    case 1: joint_factor = -1.0; break;  // Opposite
                    case 2: joint_factor = 0.5; break;   // Half amplitude
                    case 3: joint_factor = 1.5; break;   // 1.5x amplitude
                }

                double joint_value = current_value * joint_factor;
                cmd.pos[i] = joint_value;
                cmd.vel[i] = 0.0;
                cmd.tau[i] = joint_value * 0.3;
            }

            // Write to DDS
            writer.write(cmd);

            std::cout << "[" << elapsed << "ms] Published - Value: " << current_value
                     << " - Pos[0]: " << cmd.pos[0] << " Pos[1]: " << cmd.pos[1]
                     << " Switch #" << switch_count << std::endl;

            // Wait for half period
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(half_period * 1000)));

            // Switch value
            current_value = (current_value == amplitude) ? -amplitude : amplitude;
            switch_count++;
        }

        std::cout << "Command generation completed" << std::endl;
        std::cout << "Total messages published: " << switch_count << std::endl;

    } catch (const dds::core::Exception& e) {
        std::cerr << "DDS Exception: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}