#!/usr/bin/env python3
"""
ROS1 to DDS Bridge Node
将ROS1 Noetic的消息转换为DDS消息，实现与RK3588的通信

Author: Claude Code Assistant
Date: 2025-12-17
"""

import rospy
import threading
import time
import math
import socket
import struct
import json
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState

# DDS相关导入（使用CycloneDDS）
try:
    import cyclonedds
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.topic import Topic
    from cyclonedds.pub import DataWriter
    from cyclonedds.sub import DataReader
    from cyclonedds.core import Qos, Policy
except ImportError:
    print("Warning: CycloneDDS not found, using network socket instead")
    cyclonedds = None

class NetworkPacket:
    """网络数据包结构"""
    MAGIC = 0xDEADBEEF
    VERSION = 1

    def __init__(self):
        self.magic = self.MAGIC
        self.version = self.VERSION
        self.sequence = 0
        self.timestamp = int(time.time() * 1000000)  # 微秒
        self.motor_commands = []

    def pack(self):
        """打包数据为二进制格式"""
        data = struct.pack('<IIQ', self.magic, self.version, self.timestamp)
        data += struct.pack('<I', len(self.motor_commands))

        for motor in self.motor_commands:
            data += struct.pack('<Id', motor['id'], motor['position'])
            data += struct.pack('<d', motor['velocity'])
            data += struct.pack('<dd', motor['kp'], motor['kd'])
            data += struct.pack('<d', motor['torque'])

        return data

class MotorCommand:
    """电机命令结构"""
    def __init__(self, motor_id=0, q=0.0, dq=0.0, kp=20.0, kd=0.5, tau=0.0):
        self.id = motor_id
        self.position = q
        self.velocity = dq
        self.kp = kp
        self.kd = kd
        self.torque = tau

class SineWaveGenerator:
    """正弦波生成器"""
    def __init__(self, amplitude=0.3, frequency=0.5, phase_offset=0.0):
        self.amplitude = amplitude
        self.frequency = frequency
        self.phase_offset = phase_offset
        self.start_time = time.time()

    def get_value(self):
        """获取当前正弦波值"""
        current_time = time.time() - self.start_time
        phase = 2 * math.pi * self.frequency * current_time + self.phase_offset
        return self.amplitude * math.sin(phase)

class ROS1ToDDSBridge:
    """ROS1到DDS桥接节点"""

    def __init__(self):
        # 初始化ROS1节点
        rospy.init_node('ros1_to_dds_bridge', anonymous=True)

        # 配置参数
        self.rk3588_ip = rospy.get_param('~rk3588_ip', '192.168.1.20')
        self.rk3588_port = rospy.get_param('~rk3588_port', 8888)
        self.use_dds = rospy.get_param('~use_dds', False)
        self.publish_rate = rospy.get_param('~publish_rate', 50)  # Hz

        # 正弦波参数
        self.sine_amplitude = rospy.get_param('~sine_amplitude', 0.3)  # radians
        self.sine_frequency = rospy.get_param('~sine_frequency', 0.5)   # Hz
        self.target_joints = rospy.get_param('~target_joints', [0, 1, 2])  # 目标关节ID

        # 正弦波生成器
        self.sine_generators = {}
        for i, joint_id in enumerate(self.target_joints):
            phase_offset = i * (2 * math.pi / len(self.target_joints))
            self.sine_generators[joint_id] = SineWaveGenerator(
                self.sine_amplitude, self.sine_frequency, phase_offset
            )

        # ROS1发布者和订阅者
        self.setup_ros1_publishers()
        self.setup_ros1_subscribers()

        # DDS或网络连接
        self.setup_dds_connection()

        # 定时器
        self.sequence_counter = 0
        self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.timer_callback)

        rospy.loginfo("ROS1 to DDS Bridge initialized successfully!")
        rospy.loginfo(f"Target RK3588: {self.rk3588_ip}:{self.rk3588_port}")
        rospy.loginfo(f"Sine wave: {self.sine_amplitude}m at {self.sine_frequency}Hz")
        rospy.loginfo(f"Target joints: {self.target_joints}")

    def setup_ros1_publishers(self):
        """设置ROS1发布者"""
        self.joint_state_pub = rospy.Publisher(
            '/joint_states', JointState, queue_size=10
        )
        self.pose_pub = rospy.Publisher(
            '/target_pose', PoseStamped, queue_size=10
        )

    def setup_ros1_subscribers(self):
        """设置ROS1订阅者"""
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/target_joint_states', JointState, self.joint_command_callback)

    def setup_dds_connection(self):
        """设置DDS连接或网络socket"""
        if self.use_dds and cyclonedds:
            try:
                # 使用CycloneDDS
                self.domain_participant = DomainParticipant()
                rospy.loginfo("CycloneDDS initialized")
                self.dds_enabled = True
            except Exception as e:
                rospy.logwarn(f"Failed to initialize CycloneDDS: {e}")
                self.dds_enabled = False
        else:
            self.dds_enabled = False

        # 设置网络socket作为备用方案
        self.socket = None
        if not self.dds_enabled:
            self.setup_network_socket()

    def setup_network_socket(self):
        """设置网络socket连接"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            rospy.loginfo("Network socket initialized")
        except Exception as e:
            rospy.logerr(f"Failed to setup network socket: {e}")

    def cmd_vel_callback(self, msg):
        """速度命令回调"""
        rospy.logdebug(f"Received cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}")

    def joint_command_callback(self, msg):
        """关节命令回调"""
        rospy.logdebug(f"Received joint command with {len(msg.name)} joints")

    def generate_motor_commands(self):
        """生成电机命令"""
        motor_commands = []
        current_time = time.time()

        for joint_id in self.target_joints:
            generator = self.sine_generators[joint_id]
            position = generator.get_value()
            velocity = (self.sine_amplitude * 2 * math.pi * self.sine_frequency *
                       math.cos(2 * math.pi * self.sine_frequency * (current_time - generator.start_time)))

            motor_cmd = MotorCommand(
                motor_id=joint_id,
                q=position,
                dq=velocity,
                kp=20.0,
                kd=0.5,
                tau=0.0
            )
            motor_commands.append(motor_cmd)

        return motor_commands

    def send_to_dds(self, motor_commands):
        """发送命令到DDS或网络"""
        if self.dds_enabled:
            self.send_to_dds_native(motor_commands)
        else:
            self.send_to_network(motor_commands)

    def send_to_network(self, motor_commands):
        """通过网络发送命令"""
        if not self.socket:
            return

        try:
            # 创建网络数据包
            packet = NetworkPacket()
            packet.sequence = self.sequence_counter
            packet.timestamp = int(time.time() * 1000000)

            for motor in motor_commands:
                packet.motor_commands.append({
                    'id': motor.id,
                    'position': motor.position,
                    'velocity': motor.velocity,
                    'kp': motor.kp,
                    'kd': motor.kd,
                    'torque': motor.torque
                })

            # 发送数据
            data = packet.pack()
            self.socket.sendto(data, (self.rk3588_ip, self.rk3588_port))

            rospy.logdebug(f"Sent packet {self.sequence_counter} to {self.rk3588_ip}:{self.rk3588_port}")

        except Exception as e:
            rospy.logerr(f"Failed to send network packet: {e}")

    def send_to_dds_native(self, motor_commands):
        """通过原生DDS发送命令"""
        # TODO: 实现原生DDS发送
        pass

    def publish_ros1_feedback(self):
        """发布ROS1反馈消息"""
        # 发布关节状态
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = [f"joint_{i}" for i in self.target_joints]

        positions = []
        velocities = []
        efforts = []

        for joint_id in self.target_joints:
            generator = self.sine_generators[joint_id]
            positions.append(generator.get_value())
            velocities.append(0.0)
            efforts.append(0.0)

        joint_state.position = positions
        joint_state.velocity = velocities
        joint_state.effort = efforts

        self.joint_state_pub.publish(joint_state)

        # 发布目标位姿
        pose_msg = PoseStamped()
        pose_msg.header = joint_state.header
        pose_msg.pose.position.x = positions[0] if positions else 0.0
        pose_msg.pose.position.y = positions[1] if len(positions) > 1 else 0.0
        pose_msg.pose.position.z = positions[2] if len(positions) > 2 else 0.0

        self.pose_pub.publish(pose_msg)

    def timer_callback(self, event):
        """定时器回调"""
        try:
            # 生成电机命令
            motor_commands = self.generate_motor_commands()

            # 发送到RK3588
            self.send_to_dds(motor_commands)

            # 发布ROS1反馈
            self.publish_ros1_feedback()

            self.sequence_counter += 1

            # 每100次循环输出一次状态
            if self.sequence_counter % 100 == 0:
                rospy.loginfo(f"Published {self.sequence_counter} commands")

        except Exception as e:
            rospy.logerr(f"Error in timer callback: {e}")

    def run(self):
        """运行桥接节点"""
        try:
            rospy.loginfo("ROS1 to DDS Bridge is running...")
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Shutting down ROS1 to DDS Bridge...")
        finally:
            if self.socket:
                self.socket.close()

def main():
    """主函数"""
    try:
        bridge = ROS1ToDDSBridge()
        bridge.run()
    except Exception as e:
        rospy.logerr(f"Failed to start bridge: {e}")

if __name__ == '__main__':
    main()