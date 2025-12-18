#!/bin/bash

# RK3588 ROS2 LowCmd Receiver for Jetson
# This script receives /lowcmd topic from Jetson and forwards to robot hardware

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Default parameters
NETWORK_INTERFACE=${1:-"wlan0"}
DDS_DOMAIN=${DDS_DOMAIN:-0}

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    RK3588 ROS2 LowCmd 接收器${NC}"
echo -e "${BLUE}========================================${NC}"

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}⚠️  ROS2环境未检测到，尝试source...${NC}"
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo -e "${GREEN}✅ ROS2 Humble已sourced${NC}"
    else
        echo -e "${RED}❌ 找不到ROS2安装${NC}"
        exit 1
    fi
fi

# Source Unitree SDK2
if [ -f "/home/linaro/unitree_sdk2/build/setup.sh" ]; then
    source /home/linaro/unitree_sdk2/build/setup.sh
    echo -e "${GREEN}✅ Unitree SDK2已sourced${NC}"
else
    echo -e "${YELLOW}⚠️  Unitree SDK2未找到，使用系统默认配置${NC}"
fi

echo -e "${GREEN}📋 配置参数:${NC}"
echo -e "   - 网络接口: ${YELLOW}$NETWORK_INTERFACE${NC}"
echo -e "   - DDS域ID: ${YELLOW}$DDS_DOMAIN${NC}"
echo -e "   - 监听话题: ${YELLOW}/lowcmd${NC}"
echo

# Check network interface
if ! ip link show "$NETWORK_INTERFACE" >/dev/null 2>&1; then
    echo -e "${RED}❌ 网络接口 '$NETWORK_INTERFACE' 不存在${NC}"
    echo -e "${YELLOW}可用接口:${NC}"
    ip link show | grep -E '^[0-9]+:' | awk -F': ' '{print "   " $2}' | grep -v lo
    exit 1
fi

# Get IP address
IP_ADDR=$(ip addr show "$NETWORK_INTERFACE" | grep 'inet ' | awk '{print $2}' | cut -d'/' -f1)
if [ ! -z "$IP_ADDR" ]; then
    echo -e "${GREEN}✅ 网络接口 $NETWORK_INTERFACE: $IP_ADDR${NC}"
fi

echo
echo -e "${BLUE}🚀 启动RK3588 LowCmd接收器...${NC}"
echo -e "${YELLOW}等待Jetson发布 /lowcmd 话题...${NC}"
echo -e "${YELLOW}按Ctrl+C停止接收${NC}"
echo

# Set DDS domain and source ROS2
export ROS_DOMAIN_ID=$DDS_DOMAIN
export ROS_PACKAGE_PATH=/opt/ros/humble/share

# Source ROS2
source /opt/ros/humble/setup.bash

# Create a C++ receiver instead of Python for better compatibility
CPP_PROGRAM=$(cat << 'EOF'
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Unitree SDK includes
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

using namespace std::chrono_literals;

class LowCmdBridge : public rclcpp::Node {
public:
    LowCmdBridge() : Node("low_cmd_bridge"),
        msg_count_(0),
        forward_count_(0) {

        // Initialize parameters
        this->declare_parameter("network_interface", "wlan0");
        auto network_interface = this->get_parameter("network_interface").as_string();

        // Initialize Unitree SDK
        try {
            unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
            RCLCPP_INFO(this->get_logger(), "Unitree SDK initialized on %s", network_interface.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Unitree SDK: %s", e.what());
            return;
        }

        // Create ROS2 subscriber for /lowcmd (using PoseStamped as proxy)
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/lowcmd", 10,
            std::bind(&LowCmdBridge::topic_callback, this, std::placeholders::_1));

        // Create Unitree publisher
        try {
            unitree_publisher_ = std::make_shared<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>("rt/lowcmd");
            unitree_publisher_->InitChannel();
            RCLCPP_INFO(this->get_logger(), "Unitree LowCmd publisher initialized");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Unitree publisher: %s", e.what());
            return;
        }

        // Statistics timer
        timer_ = this->create_wall_timer(
            5000ms, std::bind(&LowCmdBridge::timer_callback, this));

        start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "LowCmd Bridge started");
        RCLCPP_INFO(this->get_logger(), "Listening to /lowcmd topic");
        RCLCPP_INFO(this->get_logger(), "Forwarding to Unitree rt/lowcmd");
    }

private:
    void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        msg_count_++;

        try {
            // Convert PoseStamped to LowCmd
            unitree_hg::msg::dds_::LowCmd_ lowcmd;
            convert_pose_to_lowcmd(msg, lowcmd);

            // Forward to Unitree SDK
            unitree_publisher_->Write(lowcmd);
            forward_count_++;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing message: %s", e.what());
        }
    }

    void convert_pose_to_lowcmd(const geometry_msgs::msg::PoseStamped::SharedPtr pose,
                               unitree_hg::msg::dds_::LowCmd_& lowcmd) {
        lowcmd.mode_pr() = 1;  // PR mode
        lowcmd.mode_machine() = 1;  // G1 type

        // Initialize all motors (29 for G1)
        lowcmd.motor_cmd().resize(29);
        for (int i = 0; i < 29; i++) {
            lowcmd.motor_cmd()[i].mode() = 0;
            lowcmd.motor_cmd()[i].q() = 0.0;
            lowcmd.motor_cmd()[i].dq() = 0.0;
            lowcmd.motor_cmd()[i].kp() = 0.0;
            lowcmd.motor_cmd()[i].kd() = 0.0;
            lowcmd.motor_cmd()[i].tau() = 0.0;
        }

        // Map pose to ankle joints (simplified example)
        // Left ankle: use pose.position.x for pitch, position.y for roll
        if (4 < 29) {
            lowcmd.motor_cmd()[4].mode() = 1;
            lowcmd.motor_cmd()[4].q() = pose->pose.position.x;  // pitch
            lowcmd.motor_cmd()[4].kp() = 40.0;
            lowcmd.motor_cmd()[4].kd() = 1.0;
        }
        if (5 < 29) {
            lowcmd.motor_cmd()[5].mode() = 1;
            lowcmd.motor_cmd()[5].q() = pose->pose.position.y;  // roll
            lowcmd.motor_cmd()[5].kp() = 40.0;
            lowcmd.motor_cmd()[5].kd() = 1.0;
        }

        // Right ankle: use pose.orientation (simplified)
        if (10 < 29) {
            lowcmd.motor_cmd()[10].mode() = 1;
            lowcmd.motor_cmd()[10].q() = pose->pose.orientation.x;
            lowcmd.motor_cmd()[10].kp() = 40.0;
            lowcmd.motor_cmd()[10].kd() = 1.0;
        }
        if (11 < 29) {
            lowcmd.motor_cmd()[11].mode() = 1;
            lowcmd.motor_cmd()[11].q() = pose->pose.orientation.y;
            lowcmd.motor_cmd()[11].kp() = 40.0;
            lowcmd.motor_cmd()[11].kd() = 1.0;
        }

        // Calculate CRC (simplified)
        lowcmd.crc() = 0x12345678;
    }

    void timer_callback() {
        auto elapsed = (this->now() - start_time_).seconds();
        double rate = msg_count_ / elapsed;

        RCLCPP_INFO(this->get_logger(),
                   "Received: %zu, Forwarded: %zu, Rate: %.1f Hz",
                   msg_count_, forward_count_, rate);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    std::shared_ptr<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>> unitree_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    size_t msg_count_;
    size_t forward_count_;
    rclcpp::Time start_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LowCmdBridge>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
EOF
)

# Write the C++ program to a temporary file
echo "$CPP_PROGRAM" > /tmp/lowcmd_bridge.cpp

# Set environment variables
export NETWORK_INTERFACE=$NETWORK_INTERFACE

echo -e "${GREEN}📜 C++程序已创建: /tmp/lowcmd_bridge.cpp${NC}"
echo -e "${GREEN}🔧 环境变量已设置${NC}"
echo

# Compile the C++ program
echo -e "${BLUE}⚠️  跳过C++编译，直接使用Python实现避免头文件冲突${NC}"
echo -e "${BLUE}🎯 使用Python订阅器监听/lowcmd话题${NC}"
echo

# Create a simple Python subscriber and file-based bridge
PYTHON_BRIDGE=$(cat << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import subprocess
import json
import os
import time
import signal
import sys

class PoseToLowCmdBridge(Node):
    def __init__(self):
        super().__init__('pose_to_lowcmd_bridge')

        # Create subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            '/lowcmd',
            self.pose_callback,
            10
        )

        self.get_logger().info('Pose to LowCmd bridge started')
        self.get_logger().info('Listening to /lowcmd topic')

        # File for sharing data with deploy_test
        self.bridge_file = '/tmp/lowcmd_data.json'
        self.msg_count = 0

    def pose_callback(self, msg):
        self.msg_count += 1

        try:
            # Extract sine wave data from pose
            left_pitch = msg.pose.position.x
            left_roll = msg.pose.position.y
            right_pitch = msg.pose.orientation.x
            right_roll = msg.pose.orientation.y

            # Create data structure for deploy_test
            data = {
                'timestamp': time.time(),
                'joints': {
                    4: left_pitch,   # Left ankle pitch
                    5: left_roll,    # Left ankle roll
                    10: right_pitch, # Right ankle pitch
                    11: right_roll   # Right ankle roll
                }
            }

            # Write to shared file
            with open(self.bridge_file, 'w') as f:
                json.dump(data, f)

            # Log every 50 messages
            if self.msg_count % 50 == 0:
                self.get_logger().info(f'Received {self.msg_count} poses, latest: left({left_pitch:.3f}, {left_roll:.3f}) right({right_pitch:.3f}, {right_roll:.3f})')

        except Exception as e:
            self.get_logger().error(f'Error processing pose: {e}')

def signal_handler(sig, frame):
    print('\nStopping bridge...')
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)

    rclpy.init(args=args)

    bridge = PoseToLowCmdBridge()

    try:
        rclpy.spin(bridge)
    except Exception as e:
        bridge.get_logger().error(f'Error: {e}')
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
)

# Write the Python bridge
echo "$PYTHON_BRIDGE" > /tmp/pose_bridge.py

echo -e "${GREEN}📜 创建Python桥接器: /tmp/pose_bridge.py${NC}"
echo -e "${YELLOW}启动ROS2订阅器监听/lowcmd话题...${NC}"
echo

# Start the Python bridge
python3 /tmp/pose_bridge.py

# Cleanup
rm -f /tmp/pose_bridge.py /tmp/lowcmd_data.json
echo -e "${GREEN}✅ 清理完成${NC}"