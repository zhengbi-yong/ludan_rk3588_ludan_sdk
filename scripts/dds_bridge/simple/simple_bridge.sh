#!/bin/bash

# 简化的xixiLowCmd DDS-ROS2桥接脚本
# 直接使用IDL生成ROS2消息，避免复杂转换

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_status() {
    local status=$1
    local message=$2
    case $status in
        "INFO")  echo -e "${GREEN}[INFO]${NC} $message" ;;
        "WARN")  echo -e "${YELLOW}[WARN]${NC} $message" ;;
        "ERROR") echo -e "${RED}[ERROR]${NC} $message" ;;
    esac
}

# 配置
IDL_FILE="/home/linaro/motor_dds_proj/idl/rt_lowcmd.idl"
PACKAGE_DIR="/tmp/xixi_simple_ros2"
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

print_status "INFO" "启动简化xixiLowCmd桥接方案..."
print_status "INFO" "IDL文件: $IDL_FILE"
print_status "INFO" "ROS域ID: $ROS_DOMAIN_ID"

# 检查IDL文件
if [ ! -f "$IDL_FILE" ]; then
    print_status "ERROR" "IDL文件不存在: $IDL_FILE"
    exit 1
fi

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    print_status "INFO" "加载ROS2环境..."
    source /opt/ros/humble/setup.bash
fi

# 创建临时ROS2包
print_status "INFO" "创建临时ROS2包..."
rm -rf "$PACKAGE_DIR"
mkdir -p "$PACKAGE_DIR"

cd "$PACKAGE_DIR"

# 创建包结构
cat > package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>xixi_simple_ros2</name>
  <version>1.0.0</version>
  <description>简化的xixiLowCmd ROS2消息包</description>
  <maintainer email="temp@example.com">temp</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
EOF

cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.5)
project(xixi_simple_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCmd.msg"
  "msg/LowCmd.msg"
)

ament_package()
EOF

# 创建消息文件
mkdir -p msg

cat > msg/MotorCmd.msg << 'EOF'
# 电机命令消息
uint8 mode     # 控制模式: 1=启用, 0=禁用
float32 q      # 目标位置 (rad)
float32 dq     # 目标速度 (rad/s)
float32 kp     # 位置增益
float32 kd     # 速度增益
float32 tau    # 前馈扭矩
EOF

cat > msg/LowCmd.msg << 'EOF'
# xixiLowCmd完整命令消息
uint8 mode_pr                     # 压力模式
uint8 mode_machine                # 机器模式
MotorCmd[30] motor_cmd           # 30个电机命令
uint32 crc                        # 数据校验和
EOF

# 构建包
print_status "INFO" "构建ROS2消息包..."
source /opt/ros/humble/setup.bash

if colcon build --symlink-install; then
    print_status "INFO" "✓ 包构建成功"
else
    print_status "ERROR" "✗ 包构建失败"
    exit 1
fi

# 设置环境
source install/setup.bash
export AMENT_PREFIX_PATH="$PACKAGE_DIR/install:$AMENT_PREFIX_PATH"

# 验证包
print_status "INFO" "验证生成的消息类型..."
ros2 interface show xixi_simple_ros2/msg/MotorCmd || print_status "WARN" "MotorCmd消息不可用"
ros2 interface show xixi_simple_ros2/msg/LowCmd || print_status "WARN" "LowCmd消息不可用"

# 创建简单的桥接节点
print_status "INFO" "创建简单桥接节点..."

cat > simple_bridge_node.py << 'EOF'
#!/usr/bin/env python3
"""
简化的xixiLowCmd DDS-ROS2桥接节点
直接订阅并重新发布为ROS2消息类型
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import subprocess
import threading
import time

class SimplexixiLowBridge(Node):
    def __init__(self):
        super().__init__('simple_xixi_low_bridge')

        # 创建发布器
        try:
            from xixi_simple_ros2.msg import LowCmd
            self.lowcmd_pub = self.create_publisher(LowCmd, '/lowcmd_simple', 10)
            self.use_proper_type = True
            self.get_logger().info('✓ 使用正确的ROS2消息类型')
        except ImportError as e:
            self.get_logger().error(f'无法导入消息类型: {e}')
            self.use_proper_type = False
            return

        # 同时创建标准JointState用于Foxglove
        from sensor_msgs.msg import JointState
        self.joint_pub = self.create_publisher(JointState, '/lowcmd_simple_joints', 10)

        self.get_logger().info('简化xixiLowCmd桥接节点启动')
        self.msg_count = 0

        # 启动数据监听
        self.monitor_thread = threading.Thread(target=self.monitor_topic, daemon=True)
        self.monitor_thread.start()

    def monitor_topic(self):
        """监控DDS话题并重新发布"""
        try:
            # 使用ros2 topic echo获取数据
            process = subprocess.Popen([
                'ros2', 'topic', 'echo', '/lowcmd', '--noarr'
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)

            self.get_logger().info('开始监控/lowcmd话题')
            buffer = ""

            while rclpy.ok():
                output = process.stdout.readline()
                if not output:
                    continue

                buffer += output.strip() + "\n"

                # 检查消息完整性
                if "crc:" in buffer:
                    if self.parse_and_publish(buffer):
                        buffer = ""

        except Exception as e:
            self.get_logger().error(f'话题监控错误: {e}')

    def parse_and_publish(self, buffer):
        """解析并发布消息"""
        try:
            if not self.use_proper_type:
                return False

            # 解析数据
            msg_data = self.extract_data(buffer)
            if not msg_data:
                return False

            # 发布ROS2消息
            self.publish_lowcmd(msg_data)
            self.publish_joint_state(msg_data)

            self.msg_count += 1
            if self.msg_count % 10 == 0:
                self.get_logger().info(f'已发布 {self.msg_count} 条消息')

            return True

        except Exception as e:
            self.get_logger().error(f'解析发布错误: {e}')
            return False

    def extract_data(self, buffer):
        """从buffer中提取数据"""
        try:
            lines = buffer.split('\n')
            data = {
                'mode_pr': 0,
                'mode_machine': 0,
                'motor_cmd': [],
                'crc': 0
            }

            # 基本字段
            for line in lines:
                if 'mode_pr:' in line:
                    data['mode_pr'] = int(line.split('mode_pr:')[1].strip())
                elif 'mode_machine:' in line:
                    data['mode_machine'] = int(line.split('mode_machine:')[1].strip())
                elif 'crc:' in line:
                    data['crc'] = int(line.split('crc:')[1].strip())

            # 电机命令
            for i in range(30):
                motor = {'mode': 0, 'q': 0.0, 'dq': 0.0, 'kp': 0.0, 'kd': 0.0, 'tau': 0.0}

                for line in lines:
                    if f'motor_cmd[{i}].mode:' in line:
                        motor['mode'] = int(line.split(':')[1].strip())
                    elif f'motor_cmd[{i}].q:' in line:
                        motor['q'] = float(line.split(':')[1].strip())
                    elif f'motor_cmd[{i}].dq:' in line:
                        motor['dq'] = float(line.split(':')[1].strip())
                    elif f'motor_cmd[{i}].kp:' in line:
                        motor['kp'] = float(line.split(':')[1].strip())
                    elif f'motor_cmd[{i}].kd:' in line:
                        motor['kd'] = float(line.split(':')[1].strip())
                    elif f'motor_cmd[{i}].tau:' in line:
                        motor['tau'] = float(line.split(':')[1].strip())

                data['motor_cmd'].append(motor)

            return data

        except Exception as e:
            self.get_logger().error(f'数据提取错误: {e}')
            return None

    def publish_lowcmd(self, data):
        """发布LowCmd消息"""
        from xixi_simple_ros2.msg import LowCmd, MotorCmd

        lowcmd = LowCmd()
        lowcmd.mode_pr = data['mode_pr']
        lowcmd.mode_machine = data['mode_machine']
        lowcmd.crc = data['crc']

        motor_cmds = []
        for motor_data in data['motor_cmd']:
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

    def publish_joint_state(self, data):
        """发布JointState消息"""
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header

        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = "robot_base"

        # 简化的关节名称
        joint_names = [f'joint_{i}' for i in range(30)]

        positions = []
        velocities = []
        effort = []

        for i, motor_data in enumerate(data['motor_cmd']):
            if i < len(joint_names):
                joint_state.name.append(joint_names[i])
                positions.append(motor_data['q'])
                velocities.append(motor_data['dq'])
                effort.append(motor_data['tau'])

        joint_state.position = positions
        joint_state.velocity = velocities
        joint_state.effort = effort

        self.joint_pub.publish(joint_state)

def main():
    rclpy.init()
    bridge = SimplexixiLowBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info('关闭桥接节点')
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

chmod +x simple_bridge_node.py

print_status "INFO" "✓ 简化桥接节点创建完成"
print_status "INFO" "启动桥接节点..."

export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
python3 simple_bridge_node.py &
BRIDGE_PID=$!

print_status "INFO" "桥接节点PID: $BRIDGE_PID"
print_status "INFO" "可用话题:"
echo -e "  ${GREEN}/lowcmd_simple${NC} (xixi_simple_ros2/LowCmd)"
echo -e "  ${GREEN}/lowcmd_simple_joints${NC} (sensor_msgs/JointState - Foxglove兼容)"

print_status "INFO" "按Ctrl+C停止桥接"

# 清理函数
cleanup() {
    print_status "INFO" "停止桥接节点..."
    kill $BRIDGE_PID 2>/dev/null || true
    rm -rf "$PACKAGE_DIR" 2>/dev/null || true
    exit 0
}

trap cleanup SIGINT SIGTERM

wait $BRIDGE_PID