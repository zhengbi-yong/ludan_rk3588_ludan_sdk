#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>
#include <iomanip>
#include <cstring>
#include "CANFDNET.h"
#include "zlgcan/canframe.h"
#include "zlgcan/zlgcan.h"

// Test configuration
const int TEST_DURATION_SEC = 10;
const int BURST_TEST_COUNT = 1000;

class ZLGStabilityTester {
private:
    DEVICE_HANDLE device_handle;
    CHANNEL_HANDLE channel_handle;
    std::atomic<bool> running{false};
    std::atomic<uint64_t> total_sent{0};
    std::atomic<uint64_t> total_errors{0};

public:
    bool Initialize(const std::string& ip, int port, int channel) {
        std::cout << "=== Initializing ZLG Connection ===" << std::endl;
        std::cout << "Target: " << ip << ":" << port << std::endl;
        std::cout << "Channel: CAN" << channel << std::endl;

        // Open device
        device_handle = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, 0, 0);
        if (device_handle == INVALID_DEVICE_HANDLE) {
            std::cerr << "Failed to open device" << std::endl;
            return false;
        }
        std::cout << "[1/3] Device opened" << std::endl;

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
        std::cout << "[2/3] CAN channel initialized (CAN-FD 1M/5M)" << std::endl;

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
        std::cout << "[3/3] CAN started" << std::endl;
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

    // Single frame test with delay
    bool TestSingleFrameWithDelay(uint32_t delay_us, int count) {
        std::cout << "\n=== Test: Single Frame with " << delay_us << "us delay ===" << std::endl;

        ZCAN_Transmit_Data frame;
        memset(&frame, 0, sizeof(frame));
        frame.frame.can_id = 1;  // Motor 1
        frame.frame.can_dlc = 8;
        memset(frame.frame.data, 0xFF, 7);
        frame.frame.data[7] = 0xFC;  // ENABLE command

        total_sent = 0;
        total_errors = 0;
        auto start_time = std::chrono::steady_clock::now();

        for (int i = 0; i < count; i++) {
            if (ZCAN_Transmit(channel_handle, &frame, 1) != 1) {
                total_errors++;
            } else {
                total_sent++;
            }

            if (delay_us > 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
            }
        }

        auto end_time = std::chrono::steady_clock::now();
        double elapsed_sec = std::chrono::duration<double>(end_time - start_time).count();
        double fps = total_sent / elapsed_sec;

        // Get final error info
        BYTE tx_err, rx_err;
        GetTxRxErrors(tx_err, rx_err);

        std::cout << "Frames sent: " << total_sent << std::endl;
        std::cout << "Errors: " << total_errors << std::endl;
        std::cout << "TX_Error: " << (int)tx_err << " | RX_Error: " << (int)rx_err << std::endl;
        std::cout << "Frame rate: " << std::fixed << std::setprecision(2) << fps << " fps" << std::endl;
        std::cout << "Success rate: " << std::fixed << std::setprecision(2)
                  << (100.0 * total_sent / count) << "%" << std::endl;

        return total_errors == 0;
    }

    // Burst test (no delay)
    bool TestBurst(int count) {
        std::cout << "\n=== Test: Burst Mode (" << count << " frames) ===" << std::endl;

        // Prepare frames
        std::vector<ZCAN_Transmit_Data> frames(count);
        for (int i = 0; i < count; i++) {
            memset(&frames[i], 0, sizeof(ZCAN_Transmit_Data));
            frames[i].frame.can_id = (i % 30) + 1;  // Motors 1-30
            frames[i].frame.can_dlc = 8;
            memset(frames[i].frame.data, 0xFF, 7);
            frames[i].frame.data[7] = 0xFC;
        }

        auto start_time = std::chrono::steady_clock::now();
        UINT result = ZCAN_Transmit(channel_handle, frames.data(), count);
        auto end_time = std::chrono::steady_clock::now();

        double elapsed_sec = std::chrono::duration<double>(end_time - start_time).count();
        double fps = result / elapsed_sec;

        // Get error info
        BYTE tx_err, rx_err;
        GetTxRxErrors(tx_err, rx_err);

        std::cout << "Frames transmitted: " << result << " / " << count << std::endl;
        std::cout << "TX_Error: " << (int)tx_err << " | RX_Error: " << (int)rx_err << std::endl;
        std::cout << "Time: " << std::fixed << std::setprecision(6) << elapsed_sec << " sec" << std::endl;
        std::cout << "Peak frame rate: " << std::fixed << std::setprecision(0) << fps << " fps" << std::endl;

        return result == count;
    }

    // Continuous stability test
    bool TestStability(int duration_sec) {
        std::cout << "\n=== Test: Stability (" << duration_sec << " seconds) ===" << std::endl;
        std::cout << "Sending frames continuously..." << std::endl;

        ZCAN_Transmit_Data frame;
        memset(&frame, 0, sizeof(frame));
        frame.frame.can_id = 1;
        frame.frame.can_dlc = 8;
        memset(frame.frame.data, 0xFF, 7);
        frame.frame.data[7] = 0xFC;

        running = true;
        total_sent = 0;
        total_errors = 0;

        auto start_time = std::chrono::steady_clock::now();
        auto next_report = start_time + std::chrono::seconds(1);

        while (running) {
            auto now = std::chrono::steady_clock::now();

            // Report every second
            if (now >= next_report) {
                double elapsed = std::chrono::duration<double>(now - start_time).count();
                double fps = total_sent / elapsed;

                BYTE tx_err, rx_err;
                GetTxRxErrors(tx_err, rx_err);

                std::cout << "[" << std::setw(5) << std::fixed << std::setprecision(1) << elapsed << "s] "
                          << "Sent: " << std::setw(7) << total_sent << " | "
                          << "FPS: " << std::setw(6) << std::fixed << std::setprecision(0) << fps << " | "
                          << "TX_Err: " << std::setw(3) << (int)tx_err << " | "
                          << "RX_Err: " << std::setw(3) << (int)rx_err << std::endl;

                if (elapsed >= duration_sec) {
                    break;
                }
                next_report = now + std::chrono::seconds(1);
            }

            // Send frame
            if (ZCAN_Transmit(channel_handle, &frame, 1) != 1) {
                total_errors++;
            } else {
                total_sent++;
            }

            // Small delay to prevent overwhelming
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        running = false;

        // Final report
        auto end_time = std::chrono::steady_clock::now();
        double elapsed_sec = std::chrono::duration<double>(end_time - start_time).count();
        double avg_fps = total_sent / elapsed_sec;

        BYTE tx_err, rx_err;
        GetTxRxErrors(tx_err, rx_err);

        std::cout << "\n=== Final Results ===" << std::endl;
        std::cout << "Total sent: " << total_sent << std::endl;
        std::cout << "Total errors: " << total_errors << std::endl;
        std::cout << "Average FPS: " << std::fixed << std::setprecision(2) << avg_fps << std::endl;
        std::cout << "Final TX_Error: " << (int)tx_err << std::endl;
        std::cout << "Final RX_Error: " << (int)rx_err << std::endl;

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
    std::cout << "    ZLG CAN Stability & Performance Test" << std::endl;
    std::cout << "========================================" << std::endl;

    ZLGStabilityTester tester;

    if (!tester.Initialize(ip, port, channel)) {
        std::cerr << "Failed to initialize!" << std::endl;
        return 1;
    }

    // Run tests
    std::cout << "\n## Starting Tests ##\n" << std::endl;

    // Test 1: With 1000us delay (1ms)
    tester.TestSingleFrameWithDelay(1000, 100);

    // Reset errors between tests
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Test 2: With 100us delay
    tester.TestSingleFrameWithDelay(100, 100);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Test 3: With 10us delay
    tester.TestSingleFrameWithDelay(10, 100);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Test 4: No delay (as fast as possible)
    tester.TestSingleFrameWithDelay(0, 100);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Test 5: Burst test
    tester.TestBurst(BURST_TEST_COUNT);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Test 6: Stability test
    tester.TestStability(TEST_DURATION_SEC);

    std::cout << "\n## All Tests Complete ##" << std::endl;

    tester.Cleanup();

    return 0;
}
