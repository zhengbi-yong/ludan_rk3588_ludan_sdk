#!/bin/bash

# Jetson ROS2 Sine Wave Publisher for LowCmd
# This script publishes sine wave motor commands to /lowcmd topic
# which will be received by RK3588 and forwarded to the robot

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Default parameters
RK3588_IP=""
DDS_DOMAIN=${DDS_DOMAIN:-0}
FREQUENCY=${FREQUENCY:-0.5}  # Hz
AMPLITUDE=${AMPLITUDE:-0.3}  # radians
RATE=${RATE:-50}  # Hz
DURATION=${DURATION:-60}  # seconds
TARGET_JOINTS=${TARGET_JOINTS:-"4,5,10,11"}  # Ankle joints

# Parse command line arguments
if [ $# -lt 1 ]; then
    echo "用法: $0 <rk3588_ip> [options]"
    echo ""
    echo "参数说明:"
    echo "  rk3588_ip    RK3588的IP地址"
    echo ""
    echo "可选参数:"
    echo "  --frequency <hz>    正弦波频率 (默认: 0.5 Hz)"
    echo "  --amplitude <rad>   正弦波幅度 (默认: 0.3 rad)"
    echo "  --rate <hz>         发布频率 (默认: 50 Hz)"
    echo "  --duration <s>      测试持续时间 (默认: 60 秒)"
    echo "  --joints <ids>      目标关节ID (默认: 4,5,10,11)"
    echo "  --domain <id>       DDS域ID (默认: 0)"
    echo ""
    echo "示例:"
    echo "  $0 192.168.1.100"
    echo "  $0 192.168.1.100 --frequency 1.0 --amplitude 0.5"
    echo "  $0 192.168.1.100 --rate 100 --duration 120"
    exit 1
fi

RK3588_IP="$1"
shift

# Parse optional arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --frequency)
            FREQUENCY="$2"
            shift 2
            ;;
        --amplitude)
            AMPLITUDE="$2"
            shift 2
            ;;
        --rate)
            RATE="$2"
            shift 2
            ;;
        --duration)
            DURATION="$2"
            shift 2
            ;;
        --joints)
            TARGET_JOINTS="$2"
            shift 2
            ;;
        --domain)
            DDS_DOMAIN="$2"
            shift 2
            ;;
        *)
            echo "未知参数: $1"
            exit 1
            ;;
    esac
done

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    Jetson ROS2 LowCmd Sine Wave 发布器${NC}"
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

# Also source the ROS2 Python environment
if [ -f "/opt/ros/humble/local_setup.bash" ]; then
    source /opt/ros/humble/local_setup.bash
    echo -e "${GREEN}✅ ROS2 Python环境已sourced${NC}"
fi

# Validate IP address
if ! echo "$RK3588_IP" | grep -qE '^([0-9]{1,3}\.){3}[0-9]{1,3}$'; then
    echo -e "${RED}❌ 无效的IP地址: $RK3588_IP${NC}"
    exit 1
fi

echo -e "${GREEN}📋 配置参数:${NC}"
echo -e "   - RK3588 IP: ${YELLOW}$RK3588_IP${NC}"
echo -e "   - DDS域ID: ${YELLOW}$DDS_DOMAIN${NC}"
echo -e "   - 目标关节: ${YELLOW}$TARGET_JOINTS${NC}"
echo -e "   - 正弦波幅度: ${YELLOW}$AMPLITUDE rad ($(echo "$AMPLITUDE * 180 / 3.14159" | bc -l)°)${NC}"
echo -e "   - 正弦波频率: ${YELLOW}$FREQUENCY Hz${NC}"
echo -e "   - 发布频率: ${YELLOW}$RATE Hz${NC}"
echo -e "   - 测试时长: ${YELLOW}$DURATION 秒${NC}"
echo

# Test network connectivity
echo -e "${BLUE}🔍 测试与RK3588的网络连通性...${NC}"
if ping -c 3 "$RK3588_IP" >/dev/null 2>&1; then
    echo -e "${GREEN}✅ 可以ping通RK3588${NC}"
    avg_delay=$(ping -c 3 "$RK3588_IP" 2>/dev/null | tail -1 | awk -F'/' '{print $5}' | awk '{print $1}')
    echo -e "${GREEN}   平均延迟: ${YELLOW}$avg_delay ms${NC}"
else
    echo -e "${RED}❌ 无法ping通RK3588: $RK3588_IP${NC}"
    echo -e "${YELLOW}请检查网络连接${NC}"
fi

echo
echo -e "${BLUE}🚀 启动ROS2 LowCmd Sine Wave发布器...${NC}"
echo -e "${YELLOW}注意: 确保RK3588已启动对应的接收节点${NC}"
echo -e "${YELLOW}按Ctrl+C停止发布${NC}"
echo

# Set DDS domain
export ROS_DOMAIN_ID=$DDS_DOMAIN

# Create the Python script - using PoseStamped for compatibility
PYTHON_SCRIPT=$(cat << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import math
import time
import signal
import sys
import os

class SineWavePosePublisher(Node):
    def __init__(self):
        super().__init__('sine_wave_pose_publisher')

        # Parameters from environment
        self.frequency = float(os.getenv('FREQUENCY', '0.5'))
        self.amplitude = float(os.getenv('AMPLITUDE', '0.3'))
        self.rate = float(os.getenv('RATE', '50'))
        self.duration = float(os.getenv('DURATION', '60'))

        # Create publisher for PoseStamped messages
        self.publisher_ = self.create_publisher(PoseStamped, '/lowcmd', 10)

        # Timing
        self.timer_period = 1.0 / self.rate
        self.start_time = time.time()

        # Create timer
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Statistics
        self.msg_count = 0

        self.get_logger().info(f'Sine Wave Pose Publisher started')
        self.get_logger().info(f'Frequency: {self.frequency} Hz, Amplitude: {self.amplitude} rad')
        self.get_logger().info(f'Publish rate: {self.rate} Hz')
        self.get_logger().info(f'Duration: {self.duration} seconds')
        self.get_logger().info(f'Publishing to /lowcmd as PoseStamped')

    def timer_callback(self):
        current_time = time.time() - self.start_time

        # Check duration
        if current_time >= self.duration:
            self.get_logger().info('Test duration reached, stopping...')
            rclpy.shutdown()
            return

        # Create PoseStamped message
        msg = PoseStamped()

        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        # Calculate sine wave values
        phase = 2 * math.pi * self.frequency * current_time

        # Map sine wave to pose components
        # Left ankle joints (4,5) -> position.x, position.y
        msg.pose.position.x = self.amplitude * math.sin(phase)                    # Left ankle pitch
        msg.pose.position.y = self.amplitude * math.cos(phase)                    # Left ankle roll
        msg.pose.position.z = 1.0                                                  # Height (constant)

        # Right ankle joints (10,11) -> orientation (using quaternion)
        # Create rotation from sine wave
        right_pitch = self.amplitude * math.sin(phase + math.pi/2)                # 180 deg phase shift
        right_roll = self.amplitude * math.cos(phase + math.pi/2)

        # Convert to quaternion (simplified - small angle approximation)
        msg.pose.orientation.x = right_roll  # Roll
        msg.pose.orientation.y = right_pitch # Pitch
        msg.pose.orientation.z = 0.0         # Yaw
        msg.pose.orientation.w = 1.0         # W (normalized)

        # Publish
        self.publisher_.publish(msg)
        self.msg_count += 1

        # Log every 50 messages
        if self.msg_count % 50 == 0:
            actual_rate = self.msg_count / current_time
            self.get_logger().info(f'Published {self.msg_count} messages, actual rate: {actual_rate:.1f} Hz')

def signal_handler(sig, frame):
    print('\nStopping publisher...')
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    import os

    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)

    rclpy.init(args=args)

    publisher = SineWavePosePublisher()

    try:
        rclpy.spin(publisher)
    except Exception as e:
        publisher.get_logger().error(f'Error: {e}')
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
)

# Write the Python script to a temporary file
echo "$PYTHON_SCRIPT" > /tmp/lowcmd_sine_publisher.py

# Set environment variables for the Python script
export FREQUENCY=$FREQUENCY
export AMPLITUDE=$AMPLITUDE
export RATE=$RATE
export DURATION=$DURATION
export TARGET_JOINTS=$TARGET_JOINTS

echo -e "${GREEN}📜 临时脚本已创建: /tmp/lowcmd_sine_publisher.py${NC}"
echo -e "${GREEN}🔧 环境变量已设置${NC}"
echo

# Run the Python script
echo -e "${BLUE}🎯 执行命令: python3 /tmp/lowcmd_sine_publisher.py${NC}"
echo

python3 /tmp/lowcmd_sine_publisher.py

# Cleanup
rm -f /tmp/lowcmd_sine_publisher.py
echo -e "${GREEN}✅ 清理完成${NC}"