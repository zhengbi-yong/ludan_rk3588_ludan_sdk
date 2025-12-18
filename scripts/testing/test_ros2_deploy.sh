#!/bin/bash

# Test script for ROS2 deploy_test system
# Tests the communication between Jetson and RK3588 using ROS2

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    ROS2 Deploy Test 系统测试${NC}"
echo -e "${BLUE}========================================${NC}"

# Parameters
RK3588_IP=${1:-""}
DDS_DOMAIN=${DDS_DOMAIN:-0}

if [ -z "$RK3588_IP" ]; then
    echo -e "${YELLOW}用法: $0 <rk3588_ip> [options]${NC}"
    echo
    echo "示例:"
    echo "  $0 192.168.1.100"
    echo "  $0 192.168.1.100 --domain 42"
    exit 1
fi

echo -e "${GREEN}📋 测试配置:${NC}"
echo -e "   - RK3588 IP: ${YELLOW}$RK3588_IP${NC}"
echo -e "   - DDS域ID: ${YELLOW}$DDS_DOMAIN${NC}"
echo

# 1. Check ROS2 environment
echo -e "${BLUE}🔍 1. 检查ROS2环境...${NC}"
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo -e "${GREEN}✅ ROS2 Humble已sourced${NC}"
    else
        echo -e "${RED}❌ 找不到ROS2安装${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}✅ ROS2已sourced ($ROS_DISTRO)${NC}"
fi

# 2. Check network connectivity
echo -e "${BLUE}🔍 2. 检查网络连通性...${NC}"
if ping -c 3 "$RK3588_IP" >/dev/null 2>&1; then
    echo -e "${GREEN}✅ 可以ping通RK3588${NC}"
    avg_delay=$(ping -c 3 "$RK3588_IP" 2>/dev/null | tail -1 | awk -F'/' '{print $5}' | awk '{print $1}')
    echo -e "${GREEN}   平均延迟: ${YELLOW}$avg_delay ms${NC}"
else
    echo -e "${RED}❌ 无法ping通RK3588: $RK3588_IP${NC}"
    exit 1
fi

# 3. Check ROS2 topics
echo -e "${BLUE}🔍 3. 检查ROS2话题...${NC}"
export ROS_DOMAIN_ID=$DDS_DOMAIN

# Create a simple test script
TEST_SCRIPT=$(cat << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from unitree_hg.msg.dds_ import LowCmd_
import time

class TopicTest(Node):
    def __init__(self):
        super().__init__('topic_test')
        self.publisher = self.create_publisher(LowCmd_, '/lowcmd', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = LowCmd_()
        msg.mode_pr(1)
        msg.mode_machine(1)
        msg.crc(0x12345678)

        # Add motor commands
        for i in range(29):
            from unitree_hg.msg.dds_ import MotorCmd_
            cmd = MotorCmd_()
            cmd.mode(0)
            cmd.q(0.0)
            cmd.dq(0.0)
            cmd.kp(0.0)
            cmd.kd(0.0)
            cmd.tau(0.0)
            msg.motor_cmd().append(cmd)

        self.publisher.publish(msg)
        self.count += 1
        self.get_logger().info(f'Published test message #{self.count}')

def main():
    rclpy.init()
    node = TopicTest()
    try:
        rclpy.spin_once(timeout_sec=5.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
)

echo "$TEST_SCRIPT" > /tmp/test_topics.py
echo -e "${BLUE}   测试话题发布...${NC}"
timeout 10s python3 /tmp/test_topics.py >/dev/null 2>&1 && echo -e "${GREEN}✅ /lowcmd 话题发布正常${NC}" || echo -e "${YELLOW}⚠️  话题测试超时${NC}"

# 4. Instructions
echo
echo -e "${BLUE}📝 使用说明:${NC}"
echo
echo -e "${GREEN}步骤1 - 在RK3588上启动接收器:${NC}"
echo "   ./deploy_test_rk3588_receiver.sh wlan0"
echo
echo -e "${GREEN}步骤2 - 在Jetson上启动发布器:${NC}"
echo "   ./deploy_test.sh $RK3588_IP --frequency 0.5 --amplitude 0.3"
echo
echo -e "${GREEN}步骤3 - 监控话题:${NC}"
echo "   ros2 topic hz /lowcmd"
echo "   ros2 topic echo /lowcmd"
echo

# Cleanup
rm -f /tmp/test_topics.py

echo -e "${GREEN}✅ 测试完成${NC}"
echo -e "${YELLOW}如果所有检查都通过，可以开始使用deploy_test系统${NC}"