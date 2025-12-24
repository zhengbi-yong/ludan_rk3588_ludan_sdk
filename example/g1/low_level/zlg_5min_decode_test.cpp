#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>
#include <iomanip>
#include <cstring>
#include <sstream>
#include <mutex>
#include <map>
#include "CANFDNET.h"
#include "zlgcan/canframe.h"
#include "zlgcan/zlgcan.h"

// Test configuration
const int TEST_DURATION_SEC = 300;  // 5 minutes

// Motor feedback frame decoder based on the image
class MotorFrameDecoder {
public:
    struct MotorData {
        uint32_t can_id;
        bool is_fd;
        uint8_t mode;
        int8_t torque;
        int8_t speed;
        uint8_t position;
        uint8_t temperature;
        uint8_t error;
        uint8_t status;
    };

    static MotorData DecodeFrame(const ZCAN_Receive_Data& frame) {
        MotorData data;
        memset(&data, 0, sizeof(data));

        data.can_id = frame.frame.can_id & 0x7FF;  // Standard CAN ID (11 bits)
        data.is_fd = false;

        const uint8_t* d = frame.frame.data;

        // Based on the image:
        // Data[0]: bit 7-0 = mode
        // Data[1]: bit 7 = torque (sign bit)
        // Data[2]: bit 7 = speed (sign bit)
        // Data[3]: bit 7-0 = position
        // Data[4]: bit 7-4 = temperature, bit 3-0 = error
        // Data[5]: bit 6-0 = status
        // Data[6-7]: reserved

        data.mode = d[0] & 0xFF;
        data.torque = d[1] & 0x7F;  // bit 7 is sign bit
        if (d[1] & 0x80) data.torque = -data.torque;  // Handle sign bit
        data.speed = d[2] & 0x7F;   // bit 7 is sign bit
        if (d[2] & 0x80) data.speed = -data.speed;      // Handle sign bit
        data.position = d[3] & 0xFF;

        // Temperature: bits 7-4 are temperature, bits 3-0 are error
        data.temperature = (d[4] & 0xF0) >> 4;
        data.error = d[4] & 0x0F;

        // Status bits
        data.status = d[5] & 0x7F;

        return data;
    }

    static MotorData DecodeFrameFD(const ZCAN_ReceiveFD_Data& frame) {
        MotorData data;
        memset(&data, 0, sizeof(data));

        data.can_id = frame.frame.can_id & 0x7FF;
        data.is_fd = true;

        const uint8_t* d = frame.frame.data;

        data.mode = d[0] & 0xFF;
        data.torque = d[1] & 0x7F;
        if (d[1] & 0x80) data.torque = -data.torque;
        data.speed = d[2] & 0x7F;
        if (d[2] & 0x80) data.speed = -data.speed;
        data.position = d[3] & 0xFF;
        data.temperature = (d[4] & 0xF0) >> 4;
        data.error = d[4] & 0x0F;
        data.status = d[5] & 0x7F;

        return data;
    }

    static std::string FormatDecoded(const MotorData& data, uint64_t timestamp_ms, int seq) {
        std::stringstream ss;
        ss << "#" << std::setw(3) << seq << " | ";
        ss << "Time: " << std::setw(8) << timestamp_ms << " ms | ";
        ss << "ID: " << std::setw(3) << (int)data.can_id << " | ";
        ss << "Mode: " << std::setw(3) << (int)data.mode << " | ";
        ss << "Torque: " << std::setw(4) << (int)data.torque << " | ";
        ss << "Speed: " << std::setw(4) << (int)data.speed << " | ";
        ss << "Pos: " << std::setw(3) << (int)data.position << " | ";
        ss << "Temp: " << std::setw(3) << (int)data.temperature << " | ";
        ss << "Err: " << std::setw(2) << (int)data.error << " | ";
        ss << "Status: 0x" << std::hex << (int)data.status << std::dec;
        return ss.str();
    }
};

class ZLGExtendedTester {
private:
    DEVICE_HANDLE device_handle;
    CHANNEL_HANDLE channel_handle;
    std::atomic<bool> running{false};
    std::atomic<uint64_t> total_sent{0};
    std::atomic<uint64_t> total_recv{0};
    std::atomic<uint64_t> total_tx_errors{0};

    // Frame logging
    std::mutex log_mutex;
    std::vector<std::string> decoded_frames;

public:
    bool Initialize(const std::string& ip, int port, int channel) {
        std::cout << "========================================" << std::endl;
        std::cout << "    ZLG 5-Minute Extended Test with Frame Decode" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Target: " << ip << ":" << port << std::endl;
        std::cout << "Channel: CAN" << channel << std::endl;
        std::cout << "Duration: " << TEST_DURATION_SEC << " seconds (" << (TEST_DURATION_SEC/60) << " min)" << std::endl;
        std::cout << "========================================" << std::endl;

        // Open device
        device_handle = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, 0, 0);
        if (device_handle == INVALID_DEVICE_HANDLE) {
            std::cerr << "Failed to open device" << std::endl;
            return false;
        }
        std::cout << "[1/4] Device opened" << std::endl;

        // Configure CAN-FD
        ZCAN_CHANNEL_INIT_CONFIG init_config;
        memset(&init_config, 0, sizeof(init_config));
        init_config.can_type = TYPE_CANFD;
        init_config.canfd.acc_code = 0;
        init_config.canfd.acc_mask = 0;
        init_config.canfd.abit_timing = 1000000;  // 1Mbps arbitration
        init_config.canfd.dbit_timing = 5000000;  // 5Mbps data
        init_config.canfd.brp = 0;
        init_config.canfd.filter = 0;
        init_config.canfd.mode = 0;

        channel_handle = ZCAN_InitCAN(device_handle, channel, &init_config);
        if (channel_handle == INVALID_CHANNEL_HANDLE) {
            std::cerr << "Failed to initialize CAN channel" << std::endl;
            return false;
        }
        std::cout << "[2/4] CAN channel initialized (CAN-FD 1M/5M)" << std::endl;

        // Set TCP connection parameters
        UINT val = 0;  // TCP client mode
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, channel, CMD_TCP_TYPE, &val);
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, channel, CMD_DESIP, (void*)ip.c_str());
        val = port;
        ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, channel, CMD_DESPORT, &val);

        // Start CAN
        if (ZCAN_StartCAN(channel_handle) != STATUS_OK) {
            std::cerr << "Failed to start CAN" << std::endl;
            return false;
        }
        std::cout << "[3/4] CAN started" << std::endl;
        std::cout << "[4/4] Buffer cleared, ready!" << std::endl;
        std::cout << "=====================================" << std::endl;
        return true;
    }

    void GetTxRxErrors(BYTE& tx_err, BYTE& rx_err) {
        ZCAN_CHANNEL_STATUS status;
        if (ZCAN_ReadChannelStatus(channel_handle, &status) == STATUS_OK) {
            tx_err = status.regTECounter;
            rx_err = status.regRECounter;
        } else {
            tx_err = rx_err = 255;  // Error reading
        }
    }

    // Main test - send enable commands and receive feedback
    bool RunExtendedTest() {
        std::cout << "\n=== Starting " << TEST_DURATION_SEC << " Second Extended Test ===" << std::endl;
        std::cout << "Testing control frequency and capturing motor feedback..." << std::endl;

        running = true;
        total_sent = 0;
        total_recv = 0;
        total_tx_errors = 0;

        // Prepare enable command for motor 1
        ZCAN_Transmit_Data cmd_frame;
        memset(&cmd_frame, 0, sizeof(cmd_frame));
        cmd_frame.frame.can_id = 1;
        cmd_frame.frame.can_dlc = 8;
        memset(cmd_frame.frame.data, 0xFF, 7);
        cmd_frame.frame.data[7] = 0xFC;  // ENABLE command

        // Receive buffers
        ZCAN_Receive_Data recv_frames[100];
        ZCAN_ReceiveFD_Data recv_fd_frames[50];

        std::cout << "\n";
        std::cout << "==================================================================================" << std::endl;
        std::cout << "   Time    |  Sent/s  | Recv/s |  Sent Tot | Recv Tot | TX_Err | RX_Err | Saved | Decode Summary" << std::endl;
        std::cout << "==================================================================================" << std::endl;

        int report_count = 0;
        auto start_time = std::chrono::steady_clock::now();
        auto next_report = start_time + std::chrono::seconds(1);
        auto next_save = start_time + std::chrono::seconds(1);

        uint64_t last_sent = 0;
        uint64_t last_recv = 0;
        int saved_frame_count = 0;

        // Count different motor IDs seen
        std::map<uint32_t, uint64_t> motor_count;

        while (running) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - start_time).count();

            // Send command
            if (ZCAN_Transmit(channel_handle, &cmd_frame, 1) != 1) {
                total_tx_errors++;
            } else {
                total_sent++;
            }

            // Small delay
            std::this_thread::sleep_for(std::chrono::microseconds(100));

            // Try to receive CAN standard frames
            UINT recv_count = ZCAN_Receive(channel_handle, recv_frames, 100, 0);
            for (UINT i = 0; i < recv_count; i++) {
                total_recv++;
                // Count motor IDs
                uint32_t id = recv_frames[i].frame.can_id & 0x7F;
                motor_count[id]++;
            }

            // Try to receive CAN FD frames
            UINT recv_fd_count = ZCAN_ReceiveFD(channel_handle, recv_fd_frames, 50, 0);
            for (UINT i = 0; i < recv_fd_count; i++) {
                total_recv++;
                uint32_t id = recv_fd_frames[i].frame.can_id & 0x7F;
                motor_count[id]++;
            }

            // Save one decoded frame per second
            if (now >= next_save) {
                std::string decoded_str;

                // Choose which frame type to save based on what we received
                if (recv_fd_count > 0) {
                    MotorFrameDecoder::MotorData data = MotorFrameDecoder::DecodeFrameFD(recv_fd_frames[0]);
                    decoded_str = MotorFrameDecoder::FormatDecoded(data,
                        static_cast<uint64_t>(elapsed * 1000), saved_frame_count);
                    saved_frame_count++;
                } else if (recv_count > 0) {
                    MotorFrameDecoder::MotorData data = MotorFrameDecoder::DecodeFrame(recv_frames[0]);
                    decoded_str = MotorFrameDecoder::FormatDecoded(data,
                        static_cast<uint64_t>(elapsed * 1000), saved_frame_count);
                    saved_frame_count++;
                } else {
                    decoded_str = "Frame #" + std::to_string(saved_frame_count) + " | " +
                               "Time: " + std::to_string(static_cast<uint64_t>(elapsed * 1000)) + " ms | " +
                               "No data received in this second";
                    saved_frame_count++;
                }

                {
                    std::lock_guard<std::mutex> lock(log_mutex);
                    decoded_frames.push_back(decoded_str);
                }
                next_save = now + std::chrono::seconds(1);
            }

            // Report every second
            if (now >= next_report) {
                BYTE tx_err, rx_err;
                GetTxRxErrors(tx_err, rx_err);

                uint64_t sent_this_sec = total_sent - last_sent;
                uint64_t recv_this_sec = total_recv - last_recv;

                // Build decode summary
                std::string summary = "Motors seen: ";
                for (const auto& pair : motor_count) {
                    summary += "ID" + std::to_string(pair.first) + "(" + std::to_string(pair.second) + ") ";
                }

                std::cout << "[ "
                         << std::setw(6) << std::fixed << std::setprecision(1) << elapsed << "s ] "
                         << "| " << std::setw(7) << sent_this_sec << " "
                         << "| " << std::setw(7) << recv_this_sec << " "
                         << "| " << std::setw(9) << total_sent << " "
                         "| " << std::setw(9) << total_recv << " "
                         "| " << std::setw(6) << (int)tx_err << " "
                         "| " << std::setw(6) << (int)rx_err << " "
                         "| " << std::setw(5) << saved_frame_count << " "
                         << " | " << summary << std::endl;

                last_sent = total_sent;
                last_recv = total_recv;
                report_count++;

                if (elapsed >= TEST_DURATION_SEC) {
                    break;
                }
                next_report = now + std::chrono::seconds(1);
            }
        }

        running = false;

        // Final report
        auto end_time = std::chrono::steady_clock::now();
        double elapsed_sec = std::chrono::duration<double>(end_time - start_time).count();
        double avg_tx_fps = total_sent / elapsed_sec;
        double avg_rx_fps = total_recv / elapsed_sec;

        BYTE tx_err, rx_err;
        GetTxRxErrors(tx_err, rx_err);

        std::cout << "==================================================================================" << std::endl;
        std::cout << "\n=== Final Test Results ===" << std::endl;
        std::cout << "Test Duration: " << std::fixed << std::setprecision(2) << elapsed_sec << " seconds" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Total Sent:       " << std::setw(12) << total_sent << " frames" << std::endl;
        std::cout << "Total Recv:       " << std::setw(12) << total_recv << " frames" << std::endl;
        std::cout << "Total TX Errors:  " << std::setw(12) << total_tx_errors << " frames" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Avg TX Rate:   " << std::setw(12) << std::fixed << std::setprecision(2) << avg_tx_fps << " fps" << std::endl;
        std::cout << "Avg RX Rate:   " << std::setw(12) << std::fixed << std::setprecision(2) << avg_rx_fps << " fps" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Final TX_Error: " << std::setw(12) << (int)tx_err << std::endl;
        std::cout << "Final RX_Error: " << std::setw(12) << (int)rx_err << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Frames Saved:    " << std::setw(12) << saved_frame_count << " frames" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Motor ID Summary:" << std::endl;
        for (const auto& pair : motor_count) {
            std::cout << "  Motor ID " << std::setw(3) << pair.first
                      << ": " << pair.second << " frames received" << std::endl;
        }
        std::cout << "=====================================" << std::endl;

        return true;
    }

    bool SaveLogToFile(const std::string& filename) {
        std::cout << "\n=== Saving decoded frames to " << filename << " ===" << std::endl;

        std::ofstream logfile(filename);
        if (!logfile.is_open()) {
            std::cerr << "Failed to open log file!" << std::endl;
            return false;
        }

        // Write header
        logfile << "========================================" << std::endl;
        logfile << "   Motor Feedback Test Log (Decoded)" << std::endl;
        logfile << "========================================" << std::endl;
        logfile << "Total Frames Decoded: " << decoded_frames.size() << std::endl;
        logfile << "Test Duration: " << TEST_DURATION_SEC << " seconds" << std::endl;
        logfile << "========================================" << std::endl;
        logfile << std::endl;

        // Write all decoded frames
        for (const auto& frame : decoded_frames) {
            logfile << frame << std::endl;
        }

        // Write summary
        logfile << std::endl;
        logfile << "========================================" << std::endl;
        logfile << "   Summary" << std::endl;
        logfile << "========================================" << std::endl;
        logfile << "Total Sent:     " << total_sent << std::endl;
        logfile << "Total Received: " << total_recv << std::endl;
        logfile << "TX Errors:      " << total_tx_errors << std::endl;
        logfile << std::endl;

        logfile.close();
        std::cout << "Successfully saved " << decoded_frames.size() << " decoded frames to " << filename << std::endl;

        return true;
    }

    void Cleanup() {
        if (channel_handle != INVALID_CHANNEL_HANDLE) {
            ZCAN_ResetCAN(channel_handle);
        }
        if (device_handle != INVALID_DEVICE_HANDLE) {
            ZCAN_CloseDevice(device_handle);
        }
    }
};

int main(int argc, char* argv[]) {
    std::string ip = "192.168.1.5";
    int port = 8002;
    int channel = 2;

    if (argc > 1) {
        std::string arg = argv[1];
        size_t colon_pos = arg.find(':');
        if (colon_pos != std::string::npos) {
            ip = arg.substr(0, colon_pos);
            port = std::stoi(arg.substr(colon_pos + 1));
        }
    }

    std::cout << "========================================" << std::endl;
    std::cout << "    ZLG 5-Minute Extended Test (Decoded)" << std::endl;
    std::cout << "========================================" << std::endl;

    ZLGExtendedTester tester;

    if (!tester.Initialize(ip, port, channel)) {
        std::cerr << "Failed to initialize!" << std::endl;
        return 1;
    }

    // Run the extended test
    tester.RunExtendedTest();

    // Save decoded log to file
    tester.SaveLogToFile("motor_test_frame_decode.txt");

    tester.Cleanup();

    std::cout << "\n## Test Complete ##" << std::endl;

    return 0;
}
