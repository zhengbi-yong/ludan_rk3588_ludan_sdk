// can_shm.h
// Shared memory structure for CAN frame passing between can_send_test and can_motor_feedback_publisher

#ifndef CAN_SHM_H
#define CAN_SHM_H

#include <cstdint>
#include <cstring>

// Shared memory CAN frame structure (matches ZCAN_ReceiveFD_Data frame format)
struct ShmCANFrame {
    uint32_t can_id;       // CAN ID (11-bit standard ID)
    uint8_t  data[8];      // CAN data bytes
    uint8_t  len;          // Data length (0-8)
    uint32_t flags;        // Flags (FD frame, etc.)
    uint64_t timestamp;    // Timestamp in microseconds
    bool     valid;        // Whether this frame contains valid data

    ShmCANFrame() : can_id(0), len(8), flags(0), timestamp(0), valid(false) {
        memset(data, 0, sizeof(data));
    }
};

// Shared memory layout
struct ShmLayout {
    static constexpr int MAX_MOTORS = 30;
    static constexpr int MAX_FRAMES = 100;  // Max frames per update

    ShmCANFrame frames[MAX_FRAMES];  // CAN frames buffer
    uint32_t    frame_count;         // Number of valid frames
    uint64_t    sequence;            // Sequence number for synchronization
    bool        data_ready;          // Flag to indicate new data is available

    ShmLayout() : frame_count(0), sequence(0), data_ready(false) {
        for (int i = 0; i < MAX_FRAMES; i++) {
            frames[i].valid = false;
        }
    }

    void Clear() {
        for (int i = 0; i < MAX_FRAMES; i++) {
            frames[i].valid = false;
        }
        frame_count = 0;
    }
};

// Shared memory constants
constexpr const char* SHM_NAME = "/can_motor_test_shm";
constexpr size_t SHM_SIZE = sizeof(ShmLayout);

#endif // CAN_SHM_H
