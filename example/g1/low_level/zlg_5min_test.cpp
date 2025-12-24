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
#include "CANFDNET.h"
#include "zlgcan/canframe.h"
#include "zlgcan/zlgcan.h"

// Test configuration
const int TEST_DURATION_SEC = 300;  // 5 minutes
const std::string LOG_FILE = "motor_test_log.txt";

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
    std::vector<std::string> saved_frames;

    // Statistics
    struct Stats {
        uint64_t sent_per_sec;
        uint64_t recv_per_sec;
        BYTE tx_err;
        BYTE rx_err;
    } latest_stats;

public:
    bool Initialize(const std::string& ip, int port, int channel) {
        std::cout << "========================================" << std::endl;
        std::cout << "    ZLG 5-Minute Extended Test" << std::endl;
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

        // Clear buffer
        ZCAN_ClearBuffer(channel_handle);
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
            tx_err = rx_err = 255;
        }
    }

    std::string FormatFrame(const ZCAN_Receive_Data& frame, uint64_t timestamp_ms, int seq) {
        std::stringstream ss;
        ss << "Frame #" << std::setw(5) << seq << " | ";
        ss << "Time: " << std::setw(8) << timestamp_ms << " ms | ";
        ss << "ID: 0x" << std::setfill('0') << std::setw(3) << std::hex << (frame.frame.can_id & 0x7FF) << std::dec << std::setfill(' ') << " | ";
        ss << "DLC: " << (int)frame.frame.can_dlc << " | ";
        ss << "Data: ";
        for (int i = 0; i < frame.frame.can_dlc; i++) {
            ss << std::setfill('0') << std::setw(2) << std::hex << (int)frame.frame.data[i] << std::dec << std::setfill(' ');
            if (i < frame.frame.can_dlc - 1) ss << " ";
        }
        ss << " | ";
        ss << "Timestamp: " << frame.timestamp << " us";
        return ss.str();
    }

    std::string FormatFrameFD(const ZCAN_ReceiveFD_Data& frame, uint64_t timestamp_ms, int seq) {
        std::stringstream ss;
        ss << "FrameFD #" << std::setw(5) << seq << " | ";
        ss << "Time: " << std::setw(8) << timestamp_ms << " ms | ";
        ss << "ID: 0x" << std::setfill('0') << std::setw(3) << std::hex << (frame.frame.can_id & 0x7FF) << std::dec << std::setfill(' ') << " | ";
        ss << "DLC: " << (int)frame.frame.len << " | ";
        ss << "Data: ";
        for (int i = 0; i < frame.frame.len && i < 64; i++) {
            ss << std::setfill('0') << std::setw(2) << std::hex << (int)frame.frame.data[i] << std::dec << std::setfill(' ');
            if (i < frame.frame.len - 1) ss << " ";
        }
        ss << " | ";
        ss << "Timestamp: " << frame.timestamp << " us";
        return ss.str();
    }

    // Main test - send enable commands and receive feedback
    bool RunExtendedTest() {
        std::cout << "\n=== Starting " << TEST_DURATION_SEC << " Second Extended Test ===" << std::endl;
        std::cout << "Testing control frequency and capturing motor feedback..." << std::endl;

        running = true;
        total_sent = 0;
        total_recv = 0;
        total_tx_errors = 0;

        auto start_time = std::chrono::steady_clock::now();
        auto next_report = start_time + std::chrono::seconds(1);
        auto next_save = start_time + std::chrono::seconds(1);

        uint64_t last_sent = 0;
        uint64_t last_recv = 0;
        int saved_frame_count = 0;

        // Prepare enable command for motor 1
        ZCAN_Transmit_Data cmd_frame;
        memset(&cmd_frame, 0, sizeof(cmd_frame));
        cmd_frame.frame.can_id = 1;
        cmd_frame.frame.can_dlc = 8;
        memset(cmd_frame.frame.data, 0xFF, 7);
        cmd_frame.frame.data[7] = 0xFC;

        // Receive buffers
        ZCAN_Receive_Data recv_frames[100];
        ZCAN_ReceiveFD_Data recv_fd_frames[50];

        std::cout << "\n";
        std::cout << "==================================================================================" << std::endl;
        std::cout << "   Time    |  Sent/s  | Recv/s |  Sent Tot | Recv Tot | TX_Err | RX_Err | Saved" << std::endl;
        std::cout << "==================================================================================" << std::endl;

        int report_count = 0;
        while (running) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - start_time).count();

            // Send command (try different frequencies)
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
            }

            // Try to receive CAN FD frames
            UINT recv_fd_count = ZCAN_ReceiveFD(channel_handle, recv_fd_frames, 50, 0);
            for (UINT i = 0; i < recv_fd_count; i++) {
                total_recv++;
            }

            // Save one frame per second
            if (now >= next_save) {
                std::string frame_str;

                if (recv_fd_count > 0) {
                    frame_str = FormatFrameFD(recv_fd_frames[0],
                        static_cast<uint64_t>(elapsed * 1000), saved_frame_count);
                } else if (recv_count > 0) {
                    frame_str = FormatFrame(recv_frames[0],
                        static_cast<uint64_t>(elapsed * 1000), saved_frame_count);
                } else {
                    frame_str = "Frame #" + std::to_string(saved_frame_count) + " | " +
                               "Time: " + std::to_string(static_cast<uint64_t>(elapsed * 1000)) + " ms | " +
                               "No data received in this second";
                }

                {
                    std::lock_guard<std::mutex> lock(log_mutex);
                    saved_frames.push_back(frame_str);
                }
                saved_frame_count++;
                next_save = now + std::chrono::seconds(1);
            }

            // Report every second
            if (now >= next_report) {
                BYTE tx_err, rx_err;
                GetTxRxErrors(tx_err, rx_err);

                uint64_t sent_this_sec = total_sent - last_sent;
                uint64_t recv_this_sec = total_recv - last_recv;

                std::cout << "[ "
                         << std::setw(6) << std::fixed << std::setprecision(1) << elapsed << "s ] "
                         << "| " << std::setw(8) << sent_this_sec << " "
                         << "| " << std::setw(7) << recv_this_sec << " "
                         << "| " << std::setw(9) << total_sent << " "
                         << "| " << std::setw(9) << total_recv << " "
                         << "| " << std::setw(6) << (int)tx_err << " "
                         << "| " << std::setw(6) << (int)rx_err << " "
                         << "| " << std::setw(5) << saved_frame_count << " "
                         << std::endl;

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
        std::cout << "Avg TX Rate:      " << std::setw(12) << std::fixed << std::setprecision(2) << avg_tx_fps << " fps" << std::endl;
        std::cout << "Avg RX Rate:      " << std::setw(12) << std::fixed << std::setprecision(2) << avg_rx_fps << " fps" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Final TX_Error:   " << std::setw(12) << (int)tx_err << std::endl;
        std::cout << "Final RX_Error:   " << std::setw(12) << (int)rx_err << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Frames Saved:     " << std::setw(12) << saved_frame_count << " frames" << std::endl;
        std::cout << "=====================================" << std::endl;

        return true;
    }

    bool SaveLogToFile() {
        std::cout << "\n=== Saving frames to " << LOG_FILE << " ===" << std::endl;

        std::ofstream logfile(LOG_FILE);
        if (!logfile.is_open()) {
            std::cerr << "Failed to open log file!" << std::endl;
            return false;
        }

        // Write header
        logfile << "========================================" << std::endl;
        logfile << "   Motor Feedback Test Log" << std::endl;
        logfile << "========================================" << std::endl;
        logfile << "Total Frames Saved: " << saved_frames.size() << std::endl;
        logfile << "Test Duration: " << TEST_DURATION_SEC << " seconds" << std::endl;
        logfile << "========================================" << std::endl;
        logfile << std::endl;

        // Write all saved frames
        for (const auto& frame : saved_frames) {
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

        logfile.close();
        std::cout << "Successfully saved " << saved_frames.size() << " frames to " << LOG_FILE << std::endl;

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

    ZLGExtendedTester tester;

    if (!tester.Initialize(ip, port, channel)) {
        std::cerr << "Failed to initialize!" << std::endl;
        return 1;
    }

    // Run the extended test
    tester.RunExtendedTest();

    // Save log to file
    tester.SaveLogToFile();

    tester.Cleanup();

    std::cout << "\n## Test Complete ##" << std::endl;

    return 0;
}
