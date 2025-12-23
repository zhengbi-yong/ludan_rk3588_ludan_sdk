// Simple CAN test tool - verify communication
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <thread>

// Send CAN frame via SocketCAN
bool send_can_frame(const char* ifname, canid_t id, const uint8_t* data, uint8_t dlc) {
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        perror("socket");
        return false;
    }

    struct sockaddr_can addr;
    struct ifreq ifr;
    strcpy(ifr.ifr_name, ifname);
    ioctl(sock, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(sock);
        return false;
    }

    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = id;
    frame.can_dlc = dlc;
    memcpy(frame.data, data, dlc);

    if (write(sock, &frame, sizeof(frame)) != sizeof(frame)) {
        perror("write");
        close(sock);
        return false;
    }

    close(sock);
    return true;
}

// Receive CAN frame
bool receive_can_frame(const char* ifname, int timeout_ms = 1000) {
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        perror("socket");
        return false;
    }

    struct sockaddr_can addr;
    struct ifreq ifr;
    strcpy(ifr.ifr_name, ifname);
    ioctl(sock, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(sock);
        return false;
    }

    // Set timeout
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct can_frame frame;
    ssize_t nbytes = read(sock, &frame, sizeof(struct can_frame));

    if (nbytes < 0) {
        close(sock);
        return false;
    }

    if (nbytes < sizeof(struct can_frame)) {
        close(sock);
        return false;
    }

    printf("Received: ID=0x%03X, DLC=%d, Data=", frame.can_id, frame.can_dlc);
    for (int i = 0; i < frame.can_dlc; i++) {
        printf("%02X ", frame.data[i]);
    }
    printf("\n");

    close(sock);
    return true;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printf("Usage: %s <can_interface> [test_id]\n", argv[0]);
        printf("Example: %s can0\n", argv[0]);
        printf("Example: %s can0 123\n", argv[0]);
        return 1;
    }

    const char* ifname = argv[1];
    canid_t test_id = (argc > 2) ? strtol(argv[2], NULL, 16) : 0x123;

    printf("========================================\n");
    printf("  CAN Communication Test Tool\n");
    printf("========================================\n");
    printf("Interface: %s\n", ifname);
    printf("Test ID: 0x%03X\n\n", test_id);

    // Check interface exists
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        printf("ERROR: Cannot create CAN socket. Load can-utils first:\n");
        printf("  sudo modprobe can-dev\n");
        printf("  sudo ip link set %s up\n", ifname);
        return 1;
    }
    close(sock);

    // Test 1: Send standard CAN frame
    printf("[Test 1] Sending CAN frame ID=0x%03X...\n", test_id);
    uint8_t data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    if (send_can_frame(ifname, test_id, data, 8)) {
        printf("  OK: Frame sent successfully\n");
    } else {
        printf("  FAIL: Could not send frame\n");
        return 1;
    }

    // Test 2: Try to receive (loopback or from other device)
    printf("\n[Test 2] Listening for CAN frames (1 second)...\n");
    if (receive_can_frame(ifname, 1000)) {
        printf("  OK: Received frame\n");
    } else {
        printf("  INFO: No frames received (this is normal if no loopback)\n");
    }

    // Test 3: Send multiple frames
    printf("\n[Test 3] Sending 10 frames...\n");
    int success = 0;
    for (int i = 0; i < 10; i++) {
        uint8_t d[8] = {(uint8_t)i, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (send_can_frame(ifname, test_id, d, 8)) {
            success++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    printf("  Result: %d/10 frames sent\n", success);

    printf("\n========================================\n");
    printf("Test Complete!\n");
    printf("========================================\n");

    return 0;
}
