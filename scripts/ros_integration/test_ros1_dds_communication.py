#!/usr/bin/env python3
"""
ROS1 to DDS Communication Test Script
测试ROS1 Noetic到RK3588 DDS的通信链路

Author: Claude Code Assistant
Date: 2025-12-17
"""

import rospy
import time
import math
import socket
import struct
import threading
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

class CommunicationTester:
    """通信测试器"""

    def __init__(self):
        # 初始化ROS1节点
        rospy.init_node('ros1_dds_tester', anonymous=True)

        # 配置参数
        self.rk3588_ip = rospy.get_param('~rk3588_ip', '192.168.1.20')
        self.test_port = rospy.get_param('~test_port', 8889)  # 测试端口
        self.test_duration = rospy.get_param('~test_duration', 30)  # 测试持续时间(秒)
        self.test_frequency = rospy.get_param('~test_frequency', 10)  # 测试频率(Hz)

        # 统计数据
        self.packets_sent = 0
        self.packets_received = 0
        self.test_start_time = None

        # ROS1发布者
        self.test_pub = rospy.Publisher('/test_topic', PoseStamped, queue_size=10)

        # 设置socket
        self.setup_socket()

        rospy.loginfo(f"Communication Tester initialized")
        rospy.loginfo(f"Target: {self.rk3588_ip}:{self.test_port}")
        rospy.loginfo(f"Test duration: {self.test_duration}s")

    def setup_socket(self):
        """设置测试socket"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.settimeout(1.0)  # 1秒超时
            rospy.loginfo("Test socket initialized")
        except Exception as e:
            rospy.logerr(f"Failed to setup test socket: {e}")

    def send_test_packet(self):
        """发送测试数据包"""
        try:
            # 创建测试消息
            timestamp = int(time.time() * 1000000)  # 微秒时间戳
            test_data = struct.pack('<IIQ', 0x12345678, 1, timestamp)
            test_data += struct.pack('<I', self.packets_sent)

            # 发送数据
            self.socket.sendto(test_data, (self.rk3588_ip, self.test_port))
            self.packets_sent += 1

            # 发布ROS1消息
            msg = PoseStamped()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "test_frame"
            msg.pose.position.x = math.sin(self.packets_sent * 0.1)
            msg.pose.position.y = math.cos(self.packets_sent * 0.1)
            self.test_pub.publish(msg)

            rospy.logdebug(f"Sent test packet {self.packets_sent}")

        except Exception as e:
            rospy.logerr(f"Failed to send test packet: {e}")

    def listen_for_responses(self):
        """监听响应"""
        while not rospy.is_shutdown() and (not self.test_start_time or
                                          time.time() - self.test_start_time < self.test_duration):
            try:
                data, addr = self.socket.recvfrom(1024)
                self.packets_received += 1
                rospy.logdebug(f"Received response from {addr}: {len(data)} bytes")
            except socket.timeout:
                continue
            except Exception as e:
                rospy.logerr(f"Error receiving response: {e}")
                break

    def run_test(self):
        """运行通信测试"""
        rospy.loginfo("Starting communication test...")

        self.test_start_time = time.time()

        # 启动响应监听线程
        listener_thread = threading.Thread(target=self.listen_for_responses)
        listener_thread.daemon = True
        listener_thread.start()

        # 发送测试数据包
        rate = rospy.Rate(self.test_frequency)

        while not rospy.is_shutdown() and time.time() - self.test_start_time < self.test_duration:
            self.send_test_packet()
            rate.sleep()

        # 等待最后的响应
        time.sleep(2)

        # 输出测试结果
        self.print_test_results()

    def print_test_results(self):
        """输出测试结果"""
        test_time = time.time() - self.test_start_time
        packet_loss_rate = ((self.packets_sent - self.packets_received) / self.packets_sent * 100) if self.packets_sent > 0 else 0

        rospy.loginfo("="*50)
        rospy.loginfo("COMMUNICATION TEST RESULTS")
        rospy.loginfo("="*50)
        rospy.loginfo(f"Test Duration: {test_time:.2f} seconds")
        rospy.loginfo(f"Packets Sent: {self.packets_sent}")
        rospy.loginfo(f"Packets Received: {self.packets_received}")
        rospy.loginfo(f"Packet Loss Rate: {packet_loss_rate:.2f}%")
        rospy.loginfo(f"Average Rate: {self.packets_sent/test_time:.2f} packets/sec")

        if packet_loss_rate < 5:
            rospy.loginfo("✅ Test PASSED: Communication link is stable")
        elif packet_loss_rate < 20:
            rospy.logwarn("⚠️  Test PARTIAL: Some packet loss detected")
        else:
            rospy.logerr("❌ Test FAILED: High packet loss or no communication")

        rospy.loginfo("="*50)

    def cleanup(self):
        """清理资源"""
        if self.socket:
            self.socket.close()

def main():
    """主函数"""
    try:
        tester = CommunicationTester()

        # 设置退出处理
        rospy.on_shutdown(tester.cleanup)

        # 运行测试
        tester.run_test()

    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted")
    except Exception as e:
        rospy.logerr(f"Test failed: {e}")

if __name__ == '__main__':
    main()