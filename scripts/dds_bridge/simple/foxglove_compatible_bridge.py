#!/usr/bin/env python3
"""
Foxglove兼容的xixiLowCmd桥接器
使用标准ROS2消息类型，避免临时包构建问题

Usage:
    python3 foxglove_compatible_bridge.py [--domain-id <id>] [--topic-base <prefix>]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import ByteMultiArray, Header
from geometry_msgs.msg import Vector3
import subprocess
import threading
import struct
import time
import argparse

class FoxgloveCompatiblBridge(Node):
    def __init__(self, domain_id=0, topic_base="lowcmd"):
        super().__init__('foxglove_compatible_bridge')

        self.domain_id = domain_id
        self.topic_base = topic_base
        self.msg_count = 0
        self.last_publish_time = time.time()

        # 30自由度机器人的关节名称
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

        # 设置QoS
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.VOLATILE

        # 创建发布器 - 全部使用标准消息类型
        self.joint_state_pub = self.create_publisher(JointState, f'/{topic_base}_joint_states', qos)
        self.raw_data_pub = self.create_publisher(ByteMultiArray, f'/{topic_base}_raw_bytes', qos)
        self.control_info_pub = self.create_publisher(ByteMultiArray, f'/{topic_base}_control_info', qos)

        self.get_logger().info(f'✓ Foxglove兼容桥接器启动 (域ID: {domain_id})')
        self.get_logger().info(f'✓ 发布话题:')
        self.get_logger().info(f'  - /{topic_base}_joint_states (sensor_msgs/JointState)')
        self.get_logger().info(f'  - /{topic_base}_raw_bytes (std_msgs/ByteMultiArray)')
        self.get_logger().info(f'  - /{topic_base}_control_info (std_msgs/ByteMultiArray)')

        # 启动数据监听
        self.start_data_monitoring()

    def start_data_monitoring(self):
        """启动数据监控"""
        monitor_thread = threading.Thread(target=self.monitor_lowcmd_topic, daemon=True)
        monitor_thread.start()

    def monitor_lowcmd_topic(self):
        """监控/lowcmd话题"""
        try:
            self.get_logger().info('开始监控/lowcmd话题...')
            process = subprocess.Popen([
                'ros2', 'topic', 'echo', '/lowcmd', '--noarr'
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)

            buffer = ""
            while rclpy.ok():
                output = process.stdout.readline()
                if not output:
                    time.sleep(0.01)
                    continue

                buffer += output.strip() + "\n"

                # 检查消息完整性
                if "crc:" in buffer:
                    if self.parse_and_publish(buffer):
                        buffer = ""

        except Exception as e:
            self.get_logger().error(f'话题监控错误: {e}')

    def parse_and_publish(self, buffer):
        """解析并发布消息到所有话题"""
        try:
            data = self.extract_lowcmd_data(buffer)
            if not data:
                return False

            # 发布到所有话题
            self.publish_joint_state(data)
            self.publish_raw_bytes(data)
            self.publish_control_info(data)

            self.msg_count += 1

            # 定期报告
            if self.msg_count % 100 == 0:
                current_time = time.time()
                elapsed = current_time - self.last_publish_time
                rate = 100.0 / elapsed if elapsed > 0 else 0
                self.get_logger().info(f'已发布 {self.msg_count} 条消息 (速率: {rate:.1f} Hz)')
                self.last_publish_time = current_time

            return True

        except Exception as e:
            self.get_logger().error(f'解析发布错误: {e}')
            return False

    def extract_lowcmd_data(self, buffer):
        """从buffer中提取xixiLowCmd数据"""
        try:
            lines = buffer.split('\n')
            data = {
                'mode_pr': 0,
                'mode_machine': 0,
                'motor_cmd': [],
                'crc': 0
            }

            # 提取基本字段
            for line in lines:
                line = line.strip()
                if 'mode_pr:' in line:
                    data['mode_pr'] = int(line.split('mode_pr:')[1].strip().rstrip(','))
                elif 'mode_machine:' in line:
                    data['mode_machine'] = int(line.split('mode_machine:')[1].strip().rstrip(','))
                elif 'crc:' in line:
                    data['crc'] = int(line.split('crc:')[1].strip().rstrip(','))

            # 提取30个电机命令
            for i in range(30):
                motor = {
                    'mode': 0,
                    'q': 0.0,
                    'dq': 0.0,
                    'kp': 0.0,
                    'kd': 0.0,
                    'tau': 0.0
                }

                for line in lines:
                    line = line.strip()
                    if f'motor_cmd[{i}].mode:' in line:
                        motor['mode'] = int(line.split(':')[1].strip().rstrip(','))
                    elif f'motor_cmd[{i}].q:' in line:
                        motor['q'] = float(line.split(':')[1].strip().rstrip(','))
                    elif f'motor_cmd[{i}].dq:' in line:
                        motor['dq'] = float(line.split(':')[1].strip().rstrip(','))
                    elif f'motor_cmd[{i}].kp:' in line:
                        motor['kp'] = float(line.split(':')[1].strip().rstrip(','))
                    elif f'motor_cmd[{i}].kd:' in line:
                        motor['kd'] = float(line.split(':')[1].strip().rstrip(','))
                    elif f'motor_cmd[{i}].tau:' in line:
                        motor['tau'] = float(line.split(':')[1].strip().rstrip(','))

                data['motor_cmd'].append(motor)

            return data

        except Exception as e:
            self.get_logger().error(f'数据提取错误: {e}')
            return None

    def publish_joint_state(self, data):
        """发布标准JointState消息 - Foxglove原生支持"""
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = "robot_base"

        positions = []
        velocities = []
        effort = []

        for i, motor_data in enumerate(data['motor_cmd'][:30]):
            if i < len(self.joint_names):
                joint_state.name.append(self.joint_names[i])
                positions.append(motor_data['q'])
                velocities.append(motor_data['dq'])
                effort.append(motor_data['tau'])

        joint_state.position = positions
        joint_state.velocity = velocities
        joint_state.effort = effort

        self.joint_state_pub.publish(joint_state)

    def publish_raw_bytes(self, data):
        """发布原始字节数据"""
        raw_msg = ByteMultiArray()

        # 手动序列化数据结构
        byte_data = bytearray()

        # 添加基本字段
        byte_data.extend(struct.pack('BB', data['mode_pr'], data['mode_machine']))

        # 添加电机命令
        for motor in data['motor_cmd'][:30]:
            byte_data.extend(struct.pack('B', motor['mode']))
            byte_data.extend(struct.pack('f', motor['q']))
            byte_data.extend(struct.pack('f', motor['dq']))
            byte_data.extend(struct.pack('f', motor['kp']))
            byte_data.extend(struct.pack('f', motor['kd']))
            byte_data.extend(struct.pack('f', motor['tau']))

        # 添加CRC
        byte_data.extend(struct.pack('I', data['crc']))

        raw_msg.data = list(byte_data)
        self.raw_data_pub.publish(raw_msg)

    def publish_control_info(self, data):
        """发布控制信息"""
        info_msg = ByteMultiArray()

        # 简化的控制信息
        byte_data = bytearray()

        # 只包含模式信息和部分控制参数
        byte_data.extend(struct.pack('BB', data['mode_pr'], data['mode_machine']))

        # 只包含前10个关节的基本参数作为示例
        for i, motor in enumerate(data['motor_cmd'][:10]):
            byte_data.extend(struct.pack('Bfff', motor['mode'], motor['q'], motor['dq'], motor['tau']))

        info_msg.data = list(byte_data)
        self.control_info_pub.publish(info_msg)

def main():
    parser = argparse.ArgumentParser(description='Foxglove兼容的xixiLowCmd桥接器')
    parser.add_argument('--domain-id', type=int, default=0, help='DDS域ID')
    parser.add_argument('--topic-base', type=str, default='lowcmd', help='话题前缀')
    args = parser.parse_args()

    # 设置域ID
    import os
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)

    rclpy.init()

    try:
        bridge = FoxgloveCompatiblBridge(domain_id=args.domain_id, topic_base=args.topic_base)

        print(f"\n🎯 Foxglove兼容桥接器已启动!")
        print(f"📡 可用话题:")
        print(f"   /{args.topic_base}_joint_states    - 关节状态 (sensor_msgs/JointState)")
        print(f"   /{args.topic_base}_raw_bytes       - 原始数据 (std_msgs/ByteMultiArray)")
        print(f"   /{args.topic_base}_control_info    - 控制信息 (std_msgs/ByteMultiArray)")
        print(f"\n🔧 在Foxglove中:")
        print(f"   1. 使用 'Joint State' 面板连接 /{args.topic_base}_joint_states")
        print(f"   2. 使用 'Raw Messages' 面板查看原始数据")
        print(f"\n按Ctrl+C停止\n")

        rclpy.spin(bridge)

    except KeyboardInterrupt:
        print('\n🛑 正在停止桥接器...')
    finally:
        if 'bridge' in locals():
            bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()