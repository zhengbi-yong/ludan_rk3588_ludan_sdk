// 简单的UDP接收测试程序
// 编译: g++ -o udp_test_receiver udp_test_receiver.cpp
// 运行: ./udp_test_receiver 8888

#include <iostream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <iomanip>

int main(int argc, char* argv[]) {
    int port = (argc > 1) ? std::atoi(argv[1]) : 8888;

    std::cout << "========================================" << std::endl;
    std::cout << "     UDP Test Receiver" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "监听端口: " << port << std::endl;
    std::cout << "等待接收数据..." << std::endl;
    std::cout << "按 Ctrl+C 退出" << std::endl;
    std::cout << "========================================" << std::endl << std::endl;

    // 创建UDP socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "创建socket失败: " << strerror(errno) << std::endl;
        return 1;
    }

    // 绑定端口
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "绑定端口失败: " << strerror(errno) << std::endl;
        close(sockfd);
        return 1;
    }

    std::cout << "✅ 监听器已启动，等待数据..." << std::endl << std::endl;

    // 接收数据循环
    char buffer[4096];
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);

    int packet_count = 0;

    while (true) {
        ssize_t recv_len = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0,
                                   (struct sockaddr*)&client_addr, &client_len);

        if (recv_len < 0) {
            std::cerr << "接收失败: " << strerror(errno) << std::endl;
            continue;
        }

        buffer[recv_len] = '\0';
        packet_count++;

        // 获取当前时间
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);

        std::cout << "========================================" << std::endl;
        std::cout << "📦 收到数据包 #" << packet_count << std::endl;
        std::cout << "时间: " << std::ctime(&time_t);
        std::cout << "来源: " << inet_ntoa(client_addr.sin_addr)
                  << ":" << ntohs(client_addr.sin_port) << std::endl;
        std::cout << "长度: " << recv_len << " 字节" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "内容:" << std::endl;
        std::cout << buffer << std::endl;
        std::cout << "========================================" << std::endl << std::endl;
    }

    close(sockfd);
    return 0;
}
