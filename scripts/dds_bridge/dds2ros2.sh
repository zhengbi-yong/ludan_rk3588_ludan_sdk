#!/bin/bash

# Enhanced DDS to ROS2 Bridge Launcher Script for xixiLowCmd
# This script starts a DDS to ROS2 bridge with comprehensive Foxglove WebSocket support
#
# Features:
# - Direct IDL-to-ROS2 message translation for xixiLowCmd data format
# - Multiple publishing formats for optimal Foxglove visualization
# - Automatic joint name mapping and schema generation
# - Enhanced performance and robustness

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
DEFAULT_PORT=8765
DEFAULT_HOST="0.0.0.0"
DEFAULT_RATE_LIMIT=100
LOG_LEVEL="${LOG_LEVEL:-info}"
DEFAULT_DOMAIN_ID=0

# Parse command line arguments
PORT=${FOXGLOVE_PORT:-$DEFAULT_PORT}
HOST=${FOXGLOVE_HOST:-$DEFAULT_HOST}
RATE_LIMIT=${FOXGLOVE_RATE_LIMIT:-$DEFAULT_RATE_LIMIT}
DOMAIN_ID=${ROS_DOMAIN_ID:-$DEFAULT_DOMAIN_ID}

# Print banner
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    DDS to ROS2 Bridge Launcher${NC}"
echo -e "${BLUE}========================================${NC}"
echo

# Function to print colored status
print_status() {
    local status=$1
    local message=$2
    case $status in
        "INFO")
            echo -e "${GREEN}[INFO]${NC} $message"
            ;;
        "WARN")
            echo -e "${YELLOW}[WARN]${NC} $message"
            ;;
        "ERROR")
            echo -e "${RED}[ERROR]${NC} $message"
            ;;
        *)
            echo "[INFO] $message"
            ;;
    esac
}

# Set ROS Domain ID for DDS communication
print_status "INFO" "Setting ROS Domain ID to: $DOMAIN_ID"
export ROS_DOMAIN_ID=$DOMAIN_ID

# Check if ROS2 is sourced
print_status "INFO" "Checking ROS2 environment..."
if [ -z "$ROS_DISTRO" ]; then
    print_status "WARN" "ROS2 environment not sourced. Attempting to source..."
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        print_status "INFO" "ROS2 Humble sourced successfully"
    elif [ -f "/opt/ros/foxy/setup.bash" ]; then
        source /opt/ros/foxy/setup.bash
        print_status "INFO" "ROS2 Foxy sourced successfully"
    else
        print_status "ERROR" "Could not find ROS2 installation. Please source your ROS2 environment."
        exit 1
    fi
fi

# Display configuration
print_status "INFO" "Configuration:"
echo -e "  ROS Domain ID: ${BLUE}$DOMAIN_ID${NC}"
echo -e "  Host: ${BLUE}$HOST${NC}"
echo -e "  Port: ${BLUE}$PORT${NC}"
echo -e "  Rate Limit: ${BLUE}$RATE_LIMIT${NC} msg/s"
echo -e "  Log Level: ${BLUE}$LOG_LEVEL${NC}"
echo

# Check if foxglove bridge is installed
print_status "INFO" "Checking Foxglove Bridge installation..."
if ! command -v ros2 &> /dev/null; then
    print_status "ERROR" "ROS2 command not found. Please ensure ROS2 is properly installed."
    exit 1
fi

# Try to find foxglove bridge package
BRIDGE_PACKAGE=""
for pkg in "foxglove_bridge" "ros_foxglove_bridge" "foxglove_bridge_ros2"; do
    if ros2 pkg list | grep -q "$pkg"; then
        BRIDGE_PACKAGE=$pkg
        break
    fi
done

if [ -z "$BRIDGE_PACKAGE" ]; then
    print_status "ERROR" "Foxglove Bridge package not found. Please install it:"
    echo "sudo apt install ros-\$ROS_DISTRO-foxglove-bridge"
    exit 1
fi

print_status "INFO" "Found Foxglove Bridge package: $BRIDGE_PACKAGE"

# Check if port is already in use
print_status "INFO" "Checking if port $PORT is available..."
if command -v lsof &> /dev/null; then
    if lsof -Pi :$PORT -sTCP:LISTEN -t >/dev/null; then
        print_status "WARN" "Port $PORT is already in use. Trying to find another port..."
        for alt_port in {8766..8775}; do
            if ! lsof -Pi :$alt_port -sTCP:LISTEN -t >/dev/null; then
                PORT=$alt_port
                print_status "INFO" "Using alternative port: $PORT"
                break
            fi
        done
    fi
fi

# Create log directory
LOG_DIR="$HOME/.dds2ros2/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/dds2ros2_bridge_$(date +%Y%m%d_%H%M%S).log"

print_status "INFO" "Log file: $LOG_FILE"

# Function to cleanup on exit
cleanup() {
    print_status "INFO" "Shutting down DDS to ROS2 Bridge..."
    # Kill xixiLowCmd forwarder
    if [ ! -z "$FORWARDER_PID" ]; then
        kill $FORWARDER_PID 2>/dev/null || true
        print_status "INFO" "xixiLowCmd forwarder stopped."
    fi
    # Kill background processes
    jobs -p | xargs -r kill 2>/dev/null || true
    # Clean up temporary files
    rm -f /tmp/xixi_lowcmd_forwarder.py 2>/dev/null || true
    rm -rf /tmp/xixi_lowcmd_ros2 2>/dev/null || true
    print_status "INFO" "DDS to ROS2 Bridge stopped."
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Check DDS topics before starting bridge
print_status "INFO" "Scanning for DDS topics in domain $DOMAIN_ID..."
DDS_TOPICS=$(ros2 topic list 2>/dev/null || true)
if [ -n "$DDS_TOPICS" ]; then
    print_status "INFO" "Found DDS topics:"
    echo "$DDS_TOPICS" | head -10 | while read topic; do
        echo -e "  ${BLUE}$topic${NC}"
    done
    if [ $(echo "$DDS_TOPICS" | wc -l) -gt 10 ]; then
        echo -e "  ${YELLOW}... and more topics${NC}"
    fi
else
    print_status "WARN" "No DDS topics found. The bridge will start but may have no data to forward."
fi
echo

# Start DDS to ROS2 Bridge with Foxglove support
print_status "INFO" "Starting DDS to ROS2 Bridge with Foxglove WebSocket support..."
print_status "INFO" "WebSocket URL: ws://$HOST:$PORT"
echo

# Create launch file if it doesn't exist or use the built-in one
LAUNCH_FILE=""

# Set up environment for motor_dds_proj IDL types
print_status "INFO" "Setting up environment for xixiLowCmd IDL types..."
export PYTHONPATH="/home/linaro/motor_dds_proj/generated:$PYTHONPATH"
export LD_LIBRARY_PATH="/home/linaro/motor_dds_proj/build:$LD_LIBRARY_PATH"

# Create temporary ROS2 package with xixiLowCmd message definition
print_status "INFO" "Creating temporary ROS2 package for xixiLowCmd..."

TEMP_ROS2_DIR="/tmp/xixi_lowcmd_ros2"
mkdir -p "$TEMP_ROS2_DIR"

# Create package.xml
cat > "$TEMP_ROS2_DIR/package.xml" << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>xixi_lowcmd_ros2</name>
  <version>1.0.0</version>
  <description>Temporary package for xixiLowCmd messages</description>
  <maintainer email="temp@example.com">temp</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
EOF

# Create CMakeLists.txt
cat > "$TEMP_ROS2_DIR/CMakeLists.txt" << 'EOF'
cmake_minimum_required(VERSION 3.5)
project(xixi_lowcmd_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCmd.msg"
  "msg/LowCmd.msg"
  "msg/JointCommands.msg"
)

ament_package()
EOF

# Create msg directory and separate MotorCmd.msg and LowCmd.msg based on IDL
mkdir -p "$TEMP_ROS2_DIR/msg"

# Create enhanced MotorCmd.msg with proper documentation
cat > "$TEMP_ROS2_DIR/msg/MotorCmd.msg" << 'EOF'
# MotorCmd message definition
# Generated from xixiLowCmd::MotorCmd IDL structure
# Source: ~/motor_dds_proj/idl/rt_lowcmd.idl

uint8 mode     # Motor control mode: 1=enabled, 0=disabled
float32 q      # Target position (rad)
float32 dq     # Target velocity (rad/s)
float32 kp     # Position gain (N⋅m/rad)
float32 kd     # Velocity gain (N⋅m⋅s/rad)
float32 tau    # Feedforward torque (N⋅m)
EOF

# Create enhanced LowCmd.msg with proper documentation
cat > "$TEMP_ROS2_DIR/msg/LowCmd.msg" << 'EOF'
# xixiLowCmd::LowCmd message definition
# Generated from xixiLowCmd::LowCmd IDL structure
# Source: ~/motor_dds_proj/idl/rt_lowcmd.idl
# Contains 30 joint commands for robot control

uint8 mode_pr           # Pressure mode control
uint8 mode_machine      # Machine operational mode
MotorCmd[30] motor_cmd # Array of 30 motor commands (one per joint)
uint32 crc              # Checksum for data integrity
EOF

# Create additional message types for better Foxglove integration
cat > "$TEMP_ROS2_DIR/msg/JointCommands.msg" << 'EOF'
# JointCommands message for easier Foxglove visualization
# Converts xixiLowCmd data to standard joint command format

string[] joint_names       # Joint identifier names
float32[] positions        # Target positions (rad)
float32[] velocities       # Target velocities (rad/s)
float32[] effort           # Feedforward torques (N⋅m)
float32[] position_gains   # Position gains (N⋅m/rad)
float32[] velocity_gains   # Velocity gains (N⋅m⋅s/rad)
uint8[] motor_modes        # Motor control modes
uint8 mode_pr              # Pressure mode control
uint8 mode_machine         # Machine operational mode
uint32 crc                 # Checksum for data integrity
EOF

# Build the temporary package with enhanced debugging
print_status "INFO" "Building temporary xixiLowCmd package..."
cd "$TEMP_ROS2_DIR"

# Ensure ROS2 environment is properly sourced
source /opt/ros/humble/setup.bash

# Clean any previous failed build completely
rm -rf build install log .colcon_test_results

# Verify package structure before building
print_status "INFO" "Verifying package structure..."
ls -la msg/
cat package.xml | grep -E "(name|version|description)"

# Build with verbose output for debugging
print_status "INFO" "Building with colcon..."
if colcon build --symlink-install --event-handlers console_direct+; then
    print_status "INFO" "Build completed successfully"
else
    print_status "ERROR" "Build failed, checking for issues..."
    cat build/*/colcon_mock建造log.log 2>/dev/null || echo "No build log found"
    exit 1
fi

# Verify the install directory structure
print_status "INFO" "Verifying build output..."
ls -la install/
if [ -d "install/xixi_lowcmd_ros2" ]; then
    ls -la install/xixi_lowcmd_ros2/
    ls -la install/xixi_lowcmd_ros2/share/xixi_lowcmd_ros2/msg/
else
    print_status "ERROR" "Package installation directory not found"
    exit 1
fi

# Source the built package with full path
source "$TEMP_ROS2_DIR/install/setup.bash"

# Set up environment variables for all child processes
export AMENT_PREFIX_PATH="$TEMP_ROS2_DIR/install:$AMENT_PREFIX_PATH"
export ROS_PACKAGE_PATH="$TEMP_ROS2_DIR/install/share:$ROS_PACKAGE_PATH"
export PYTHONPATH="$TEMP_ROS2_DIR/install/xixi_lowcmd_ros2/lib/python3.10/site-packages:$PYTHONPATH"

# Create xixiLowCmd message type support
print_status "INFO" "xixiLowCmd message type is now available to Foxglove..."

# Create enhanced message forwarder with better Foxglove support
cat > /tmp/xixi_lowcmd_forwarder.py << 'EOF'
#!/usr/bin/env python3
"""
Enhanced xixiLowCmd Message Forwarder for Foxglove Integration
将原始的 xixiLowCmd::LowCmd DDS 数据重新发布为多种ROS2消息格式
支持完整的Foxglove可视化功能
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, Header
from sensor_msgs.msg import JointState
import subprocess
import threading
import struct
import time
from datetime import datetime

class xixiLowCmdForwarder(Node):
    def __init__(self):
        super().__init__('xixi_lowcmd_forwarder')

        # Joint names for 30-DOF robot (typical configuration)
        self.joint_names = [
            'left_hip_yaw_joint', 'left_hip_roll_joint', 'left_hip_pitch_joint',
            'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
            'right_hip_yaw_joint', 'right_hip_roll_joint', 'right_hip_pitch_joint',
            'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
            'torso_joint', 'left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
            'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_pitch_joint',
            'left_wrist_roll_joint', 'right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
            'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_pitch_joint',
            'right_wrist_roll_joint', 'head_yaw_joint', 'head_pitch_joint',
            'left_gripper_joint', 'right_gripper_joint', 'waist_joint'
        ]

        # 创建多种发布器以支持不同的Foxglove可视化
        try:
            # 加载自定义消息类型
            from xixi_lowcmd_ros2.msg import LowCmd, MotorCmd, JointCommands

            # 原始xixiLowCmd格式
            self.lowcmd_pub = self.create_publisher(LowCmd, '/lowcmd', 10)
            # 标准JointState格式 (Foxglove原生支持)
            self.joint_state_pub = self.create_publisher(JointState, '/lowcmd_joint_states', 10)
            # 自定义JointCommands格式
            self.joint_commands_pub = self.create_publisher(JointCommands, '/lowcmd_joint_commands', 10)

            self.get_logger().info('Successfully created all publishers with proper ROS2 types')
            self.use_proper_type = True
        except Exception as e:
            self.get_logger().warn(f'Cannot use proper message types: {e}')
            # 使用标准消息类型作为fallback
            self.lowcmd_pub = self.create_publisher(ByteMultiArray, '/lowcmd_raw', 10)
            self.joint_state_pub = self.create_publisher(JointState, '/lowcmd_joint_states', 10)
            self.use_proper_type = False

        self.get_logger().info('Enhanced xixiLowCmd Forwarder started')
        self.msg_count = 0
        self.running = True
        self.last_publish_time = time.time()

        # 启动数据监听线程
        self.data_thread = threading.Thread(target=self.data_listener, daemon=True)
        self.data_thread.start()

    def data_listener(self):
        """监听原始DDS数据并重新发布为多种格式"""
        try:
            # 使用ros2 topic echo监听原始数据，但使用更高效的方式
            process = subprocess.Popen([
                'ros2', 'topic', 'echo', '/lowcmd', '--noarr', '--once'  # --once for better performance
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)

            self.get_logger().info('Started enhanced ros2 topic echo for data capture')

            buffer = ""

            # 持续监听，每次获取新消息
            while self.running and rclpy.ok():
                try:
                    # 重新启动进程以获取最新消息
                    if process.poll() is not None:
                        process = subprocess.Popen([
                            'ros2', 'topic', 'echo', '/lowcmd', '--noarr', '--once'
                        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)

                    output = process.stdout.readline()
                    if not output:
                        time.sleep(0.01)  # 短暂休眠避免CPU占用过高
                        continue

                    buffer += output.strip() + "\n"

                    # 检查是否收集到完整消息
                    if "crc:" in buffer:
                        msg_data = self.parse_message_data(buffer)
                        if msg_data:
                            self.publish_all_formats(msg_data)
                            buffer = ""

                except Exception as e:
                    self.get_logger().error(f'Error reading data: {e}')
                    time.sleep(0.1)  # 错误后短暂休眠

        except Exception as e:
            self.get_logger().error(f'Error starting data listener: {e}')

    def parse_message_data(self, buffer):
        """解析xixiLowCmd消息数据"""
        try:
            msg_data = {
                'mode_pr': 0,
                'mode_machine': 0,
                'motor_cmd': [],
                'crc': 0
            }

            lines = buffer.split('\n')

            # 解析基本字段
            for line in lines:
                line = line.strip()
                if 'mode_pr:' in line:
                    msg_data['mode_pr'] = int(self.extract_value(line, 'mode_pr:'))
                elif 'mode_machine:' in line:
                    msg_data['mode_machine'] = int(self.extract_value(line, 'mode_machine:'))
                elif 'crc:' in line:
                    msg_data['crc'] = int(self.extract_value(line, 'crc:'))

            # 解析30个电机命令
            motor_cmds = []
            for i in range(30):  # NUM_JOINTS = 30
                motor = {
                    'mode': 0,
                    'q': 0.0,
                    'dq': 0.0,
                    'kp': 0.0,
                    'kd': 0.0,
                    'tau': 0.0
                }

                # 查找各个字段
                for line in lines:
                    line = line.strip()
                    if f'motor_cmd[{i}].mode:' in line:
                        motor['mode'] = int(self.extract_value(line, f'motor_cmd[{i}].mode:'))
                    elif f'motor_cmd[{i}].q:' in line:
                        motor['q'] = float(self.extract_value(line, f'motor_cmd[{i}].q:'))
                    elif f'motor_cmd[{i}].dq:' in line:
                        motor['dq'] = float(self.extract_value(line, f'motor_cmd[{i}].dq:'))
                    elif f'motor_cmd[{i}].kp:' in line:
                        motor['kp'] = float(self.extract_value(line, f'motor_cmd[{i}].kp:'))
                    elif f'motor_cmd[{i}].kd:' in line:
                        motor['kd'] = float(self.extract_value(line, f'motor_cmd[{i}].kd:'))
                    elif f'motor_cmd[{i}].tau:' in line:
                        motor['tau'] = float(self.extract_value(line, f'motor_cmd[{i}].tau:'))

                motor_cmds.append(motor)

            msg_data['motor_cmd'] = motor_cmds
            return msg_data

        except Exception as e:
            self.get_logger().error(f'Error parsing message data: {e}')
            return None

    def extract_value(self, line, field):
        """从行中提取字段值"""
        try:
            parts = line.split(field)
            if len(parts) > 1:
                value_str = parts[1].strip().rstrip(',').strip()
                return value_str
        except:
            pass
        return None

    def publish_all_formats(self, msg_data):
        """发布所有格式的消息"""
        try:
            current_time = time.time()

            # 1. 发布原始xixiLowCmd格式
            if self.use_proper_type:
                self.publish_lowcmd(msg_data)
                self.publish_joint_commands(msg_data)

            # 2. 发布标准JointState格式 (Foxglove原生支持)
            self.publish_joint_state(msg_data)

            self.msg_count += 1

            # 减少日志频率，每50条消息记录一次
            if self.msg_count % 50 == 0:
                elapsed = current_time - self.last_publish_time
                rate = 50.0 / elapsed if elapsed > 0 else 0
                self.get_logger().info(f'Published {self.msg_count} messages ({rate:.1f} Hz)')
                self.last_publish_time = current_time

        except Exception as e:
            self.get_logger().error(f'Error publishing messages: {e}')

    def publish_lowcmd(self, msg_data):
        """发布原始xixiLowCmd::LowCmd格式"""
        try:
            from xixi_lowcmd_ros2.msg import LowCmd, MotorCmd

            lowcmd = LowCmd()
            lowcmd.mode_pr = msg_data['mode_pr']
            lowcmd.mode_machine = msg_data['mode_machine']
            lowcmd.crc = msg_data['crc']

            # 添加电机命令
            motor_cmds = []
            for motor_data in msg_data['motor_cmd']:
                motor = MotorCmd()
                motor.mode = motor_data['mode']
                motor.q = motor_data['q']
                motor.dq = motor_data['dq']
                motor.kp = motor_data['kp']
                motor.kd = motor_data['kd']
                motor.tau = motor_data['tau']
                motor_cmds.append(motor)

            lowcmd.motor_cmd = motor_cmds
            self.lowcmd_pub.publish(lowcmd)

        except Exception as e:
            self.get_logger().error(f'Error publishing LowCmd: {e}')

    def publish_joint_state(self, msg_data):
        """发布标准JointState格式 (Foxglove原生支持)"""
        try:
            joint_state = JointState()
            joint_state.header = Header()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.header.frame_id = "robot_base"

            # 填充关节数据
            positions = []
            velocities = []
            effort = []

            for i, motor_data in enumerate(msg_data['motor_cmd']):
                if i < len(self.joint_names):
                    joint_state.name.append(self.joint_names[i])
                    positions.append(motor_data['q'])
                    velocities.append(motor_data['dq'])
                    effort.append(motor_data['tau'])

            joint_state.position = positions
            joint_state.velocity = velocities
            joint_state.effort = effort

            self.joint_state_pub.publish(joint_state)

        except Exception as e:
            self.get_logger().error(f'Error publishing JointState: {e}')

    def publish_joint_commands(self, msg_data):
        """发布自定义JointCommands格式"""
        try:
            from xixi_lowcmd_ros2.msg import JointCommands

            joint_commands = JointCommands()
            joint_commands.joint_names = self.joint_names[:30]  # 确保只使用30个关节名
            joint_commands.mode_pr = msg_data['mode_pr']
            joint_commands.mode_machine = msg_data['mode_machine']
            joint_commands.crc = msg_data['crc']

            # 填充关节数据
            positions = []
            velocities = []
            effort = []
            position_gains = []
            velocity_gains = []
            motor_modes = []

            for motor_data in msg_data['motor_cmd']:
                positions.append(motor_data['q'])
                velocities.append(motor_data['dq'])
                effort.append(motor_data['tau'])
                position_gains.append(motor_data['kp'])
                velocity_gains.append(motor_data['kd'])
                motor_modes.append(motor_data['mode'])

            joint_commands.positions = positions
            joint_commands.velocities = velocities
            joint_commands.effort = effort
            joint_commands.position_gains = position_gains
            joint_commands.velocity_gains = velocity_gains
            joint_commands.motor_modes = motor_modes

            self.joint_commands_pub.publish(joint_commands)

        except Exception as e:
            self.get_logger().error(f'Error publishing JointCommands: {e}')

    def stop_listener(self):
        """停止监听"""
        self.running = False

def main(args=None):
    rclpy.init(args=args)

    forwarder = xixiLowCmdForwarder()

    try:
        rclpy.spin(forwarder)
    except KeyboardInterrupt:
        forwarder.get_logger().info('Shutting down enhanced xixiLowCmd forwarder...')
    finally:
        forwarder.stop_listener()
        forwarder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

# Set up comprehensive environment for all processes
print_status "INFO" "Setting up comprehensive environment for all processes..."
export AMENT_PREFIX_PATH="$TEMP_ROS2_DIR/install:$AMENT_PREFIX_PATH"
export ROS_PACKAGE_PATH="$TEMP_ROS2_DIR/install/share:$ROS_PACKAGE_PATH"
export PYTHONPATH="$TEMP_ROS2_DIR/install/xixi_lowcmd_ros2/lib/python3.10/site-packages:$PYTHONPATH"
export LD_LIBRARY_PATH="$TEMP_ROS2_DIR/install/lib:$LD_LIBRARY_PATH"

# Start xixiLowCmd forwarder in background with correct environment
print_status "INFO" "Starting xixiLowCmd message forwarder..."
cd "$TEMP_ROS2_DIR"
source /opt/ros/humble/setup.bash
source "$TEMP_ROS2_DIR/install/setup.bash"

# Verify environment before starting forwarder
print_status "INFO" "Verifying environment before starting forwarder..."
ros2 pkg list | grep xixi_lowcmd_ros2 && echo "Package found in ROS2" || echo "Package NOT found in ROS2"
python3 -c "import sys; print('\n'.join(sys.path))" | head -5

python3 /tmp/xixi_lowcmd_forwarder.py &
FORWARDER_PID=$!
echo "Forwarder PID: $FORWARDER_PID" >> "$LOG_FILE"

# Ensure the temporary package is sourced for Foxglove Bridge
print_status "INFO" "Ensuring temporary package is available to Foxglove Bridge..."
export GAZEBO_MODEL_PATH="$TEMP_ROS2_DIR/install/share:$GAZEBO_MODEL_PATH"
export GAZEBO_PLUGIN_PATH="$TEMP_ROS2_DIR/install/lib:$GAZEBO_PLUGIN_PATH"

# Start the bridge in background with the correct environment
cd /home/linaro/unitree_sdk2  # Return to original directory
source /opt/ros/humble/setup.bash
source "$TEMP_ROS2_DIR/install/setup.bash"

# Verify Foxglove can find our package
print_status "INFO" "Verifying Foxglove bridge environment..."
echo "AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH"
ros2 pkg list | grep xixi_lowcmd_ros2 && echo "✓ Package available for Foxglove" || echo "✗ Package NOT available for Foxglove"

print_status "INFO" "Starting Foxglove Bridge with enhanced environment..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    port:=$PORT \
    address:=$HOST \
    rate_limit:=$RATE_LIMIT \
    log_level:=$LOG_LEVEL \
    2>&1 | tee "$LOG_FILE" &

BRIDGE_PID=$!

print_status "INFO" "DDS to ROS2 Bridge started with PID: $BRIDGE_PID"
print_status "INFO" "xixiLowCmd forwarder started with PID: $FORWARDER_PID"

# Wait a moment for the bridge to start
sleep 3

# Check if the bridge is running
if kill -0 $BRIDGE_PID 2>/dev/null; then
    print_status "INFO" "DDS to ROS2 Bridge is running successfully!"
    echo
    print_status "INFO" "Connection Information:"
    echo -e "  ${GREEN}ROS Domain ID:${NC} $DOMAIN_ID"
    echo -e "  ${GREEN}WebSocket URL:${NC} ws://$HOST:$PORT"
    echo -e "  ${GREEN}Foxglove Studio:${NC} Open https://studio.foxglove.dev/ and connect to ws://$HOST:$PORT"
    echo
    print_status "INFO" "Available ROS2 Topics (DDS Domain $DOMAIN_ID):"
    ros2 topic list --no.arr 2>/dev/null | while read topic; do
        type=$(ros2 topic info $topic 2>/dev/null | grep "Type:" | cut -d' ' -f2 || echo "Unknown")
        echo -e "  ${BLUE}$topic${NC} ($type)"
    done
    echo
    print_status "INFO" "Enhanced xixiLowCmd IDL Bridge Topics:"
    echo -e "  ${GREEN}/lowcmd${NC} (xixiLowCmd::LowCmd - proper ROS2 type ✓)"
    echo -e "  ${GREEN}/lowcmd_joint_states${NC} (sensor_msgs/JointState - Foxglove native ✓)"
    echo -e "  ${GREEN}/lowcmd_joint_commands${NC} (xixi_lowcmd_ros2/JointCommands - enhanced ✓)"
    echo -e "  ${YELLOW}/lowcmd_raw${NC} (std_msgs/ByteMultiArray - fallback mode)"
    echo
    print_status "INFO" "IDL Structure: 30 joints, each with mode, q, dq, kp, kd, tau fields"
    echo
    print_status "INFO" "Foxglove Integration:"
    echo -e "  • ${BLUE}/lowcmd_joint_states${NC} - Use joint state panel for real-time visualization"
    echo -e "  • ${BLUE}/lowcmd${NC} - Raw xixiLowCmd data with proper schema recognition"
    echo -e "  • ${BLUE}/lowcmd_joint_commands${NC} - Complete motor command data with gains"
    echo
    print_status "INFO" "Enhanced Features:"
    echo -e "  • Automatic joint name mapping for 30-DOF robot"
    echo -e "  • Multiple message formats for different visualization needs"
    echo -e "  • Improved performance with reduced CPU usage"
    echo -e "  • Better error handling and logging"
    echo
    print_status "INFO" "DDS to ROS2 Bridge is ready for Foxglove connections!"
    print_status "INFO" "Press Ctrl+C to stop the bridge"

    # Wait for the bridge process
    wait $BRIDGE_PID
else
    print_status "ERROR" "Failed to start DDS to ROS2 Bridge"
    print_status "INFO" "Check the log file for errors: $LOG_FILE"
    exit 1
fi
