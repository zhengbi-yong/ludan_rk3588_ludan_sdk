// CAN Network Box Test Program - 支持 ZLG 网络版 CAN 盒
// ID: 0x01-0x0A 轮询使能，周期 2ms
// 使能帧: FF FF FF FF FF FF FF FC
// 直接使用 TCP 网络协议发送，不依赖 zlgcan_linux.cpp

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

using namespace std;

// 全局控制标志
static atomic<bool> g_running(true);

// TCP socket 连接
static int g_socket = -1;

// 信号处理
void signal_handler(int signum) {
    g_running = false;
}

//=============================================================================
// 网络协议打包函数 - 直接实现 TCP 发送
//=============================================================================

// 计算 XOR 校验码
static unsigned char calc_checksum(const unsigned char* data, int len) {
    unsigned char checksum = 0;
    for (int i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// 打包并发送标准 CAN 帧 (31 字节) - 全部大端序
static bool send_can_frame_tcp(int sock, int channel, uint32_t can_id,
                                const uint8_t* data, uint8_t dlc) {
    unsigned char packet[128];
    int offset = 0;

    // 1. 协议头
    packet[offset++] = 0x55;

    // 2. 类型 + 保留字节
    packet[offset++] = 0x00;  // CAN 消息类型
    packet[offset++] = 0x00;  // 保留
    packet[offset++] = 0x00;  // 保留

    // 3. 数据长度 (大端序) - 24 字节
    packet[offset++] = 0x00;
    packet[offset++] = 0x18;

    // 4. 时间戳 (8 字节) - 发送时填 0
    for (int j = 0; j < 8; j++) {
        packet[offset++] = 0x00;
    }

    // 5. CAN ID (4 字节, 大端序)
    packet[offset++] = (can_id >> 24) & 0xFF;
    packet[offset++] = (can_id >> 16) & 0xFF;
    packet[offset++] = (can_id >> 8) & 0xFF;
    packet[offset++] = can_id & 0xFF;

    // 6. 报文信息 (2 字节, 大端序)
    packet[offset++] = 0x00;  // 高字节
    packet[offset++] = 0x00;  // 低字节 (标准 CAN 帧)

    // 7. 通道
    packet[offset++] = channel & 0xFF;

    // 8. 数据长度
    packet[offset++] = dlc;

    // 9. 数据 (8 字节)
    for (int j = 0; j < 8; j++) {
        packet[offset++] = data[j];
    }

    // 10. 校验码 (XOR)
    unsigned char checksum = calc_checksum(packet, offset);
    packet[offset++] = checksum;

    // 调试打印
    fprintf(stderr, "Sending %d bytes: ", offset);
    for (int j = 0; j < offset; j++) {
        fprintf(stderr, "%02X ", packet[j]);
    }
    fprintf(stderr, "\n");

    // 发送
    ssize_t sent = send(sock, packet, offset, 0);
    return sent == offset;
}

// 创建 TCP 连接
static int create_tcp_connection(const char* ip, int port) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("socket");
        return -1;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &addr.sin_addr) <= 0) {
        perror("inet_pton");
        close(sock);
        return -1;
    }

    // 设置超时
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof(tv));

    cout << "Connecting to " << ip << ":" << port << "..." << endl;
    if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("connect");
        close(sock);
        return -1;
    }
    cout << "Connected!" << endl;

    return sock;
}

// 发送使能帧到指定电机 - 直接使用 TCP 协议
static bool send_enable_frame_tcp(int sock, int channel, uint8_t motor_id) {
    // 使能帧数据: FF FF FF FF FF FF FF FC
    uint8_t enable_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

    // 发送 CAN 帧
    return send_can_frame_tcp(sock, channel, motor_id, enable_data, 8);
}

//=============================================================================
// 主程序
//=============================================================================

int main(int argc, char* argv[]) {
    // 注册信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    cout << "========================================" << endl;
    cout << "  ZLG Network CAN Box Test Program" << endl;
    cout << "  Direct TCP Protocol (no zlgcan_linux)" << endl;
    cout << "========================================" << endl;

    // 参数解析
    const char* ip = "192.168.1.5";
    int port = 8002;  // TCP端口
    int channel = 2;  // CAN2

    if (argc >= 2) {
        ip = argv[1];
    }
    if (argc >= 3) {
        port = atoi(argv[2]);
    }

    cout << "Target: " << ip << ":" << port << endl;
    cout << "Channel: CAN" << channel << endl;
    cout << "Motor IDs: 0x01 - 0x0A" << endl;
    cout << "Enable frame: FF FF FF FF FF FF FF FC" << endl;
    cout << endl;

    // 创建 TCP 连接
    g_socket = create_tcp_connection(ip, port);
    if (g_socket < 0) {
        cerr << "ERROR: Failed to connect" << endl;
        return 1;
    }

    // 等待连接稳定
    this_thread::sleep_for(chrono::milliseconds(100));

    cout << endl;
    cout << "[4] Starting motor enable sequence..." << endl;
    cout << "========================================" << endl;

    // 轮询使能电机 0x01 - 0x0A
    for (uint8_t motor_id = 0x01; motor_id <= 0x0A && g_running; motor_id++) {
        cout << "Enabling motor 0x" << hex << (int)motor_id << dec << " ..." << endl;

        // 发送使能帧 (CAN ID = motor_id)
        if (!send_enable_frame_tcp(g_socket, channel, motor_id)) {
            cerr << "    WARNING: Failed to send enable frame" << endl;
        } else {
            cout << "    Enable frame sent: CAN ID=0x" << hex << (int)motor_id << dec << ", Data=FF FF FF FF FF FF FF FC" << endl;
        }

        // 等待 100ms
        this_thread::sleep_for(chrono::milliseconds(100));
    }

    cout << endl;
    cout << "[5] Enable complete!" << endl;
    cout << "All motors (0x01-0x0A) have been enabled." << endl;
    cout << "========================================" << endl;

    // 关闭连接
    if (g_socket >= 0) {
        close(g_socket);
    }

    cout << "Done!" << endl;
    cout << "========================================" << endl;

    return 0;
}
