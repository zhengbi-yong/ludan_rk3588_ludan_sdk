#!/usr/bin/env python3
"""
Jetson LowCmd Publisher
在Jetson上发布/lowcmd话题，发送正弦波轨迹数据到RK3588

Author: Claude Code Assistant
Date: 2025-12-17
"""

import rospy
import time
import math
import numpy as np
import json
import socket
import struct
import threading
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

class JetsonLowCmdPublisher:
    """Jetson LowCmd发布器"""

    def __init__(self):
        # 初始化ROS1节点
        rospy.init_node('jetson_lowcmd_publisher', anonymous=True)

        # 配置参数
        self.rk3588_ip = rospy.get_param('~rk3588_ip', '192.168.1.10')  # 修改为正确的IP
        self.rk3588_port = rospy.get_param('~rk3588_port', 8888)
        self.publish_rate = rospy.get_param('~publish_rate', 50)  # Hz
        self.use_network = rospy.get_param('~use_network', True)
        self.use_dds_bridge = rospy.get_param('~use_dds_bridge', True)

        # 正弦波参数
        self.sine_amplitude = rospy.get_param('~sine_amplitude', 0.3)  # 弧度
        self.sine_frequency = rospy.get_param('~sine_frequency', 0.5)   # Hz
        self.target_joints = rospy.get_param('~target_joints', [4, 5, 10, 11])  # 脚踝关节

        # 电机配置：支持30个电机 (ID: 1-30)
        self.num_motors = 30
        self.motor_id_offset = rospy.get_param('~motor_id_offset', 1)  # motor_id从1开始

        # 初始化组件
        self.setup_publishers()
        self.setup_network()

        # 统计信息
        self.msg_count = 0
        self.start_time = time.time()

        rospy.loginfo("="*60)
        rospy.loginfo("    Jetson LowCmd Publisher Started")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Target RK3588: {self.rk3588_ip}:{self.rk3588_port}")
        rospy.loginfo(f"Publish rate: {self.publish_rate} Hz")
        rospy.loginfo(f"Motor count: {self.num_motors} (ID: 1-{self.num_motors})")
        rospy.loginfo(f"Sine wave: {self.sine_amplitude} rad @ {self.sine_frequency} Hz")
        rospy.loginfo(f"Target joints: {self.target_joints}")
        rospy.loginfo(f"Network mode: {self.use_network}")
        rospy.loginfo(f"DDS bridge: {self.use_dds_bridge}")

    def setup_publishers(self):
        """设置ROS1发布者"""
        # 发布到/lowcmd话题
        self.lowcmd_pub = rospy.Publisher('/lowcmd', PoseStamped, queue_size=10)

        # 额外的状态发布者（用于调试）
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.debug_pub = rospy.Publisher('/debug_info', Header, queue_size=10)

        rospy.loginfo("✅ ROS1 publishers initialized")

    def setup_network(self):
        """设置网络连接"""
        if self.use_network:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.socket.settimeout(1.0)
                rospy.loginfo("✅ Network socket initialized")
            except Exception as e:
                rospy.logerr(f"Failed to setup network socket: {e}")
                self.use_network = False

    def generate_sine_wave_positions(self, current_time):
        """生成正弦波位置"""
        positions = {}

        for i, joint_id in enumerate(self.target_joints):
            # 为不同关节设置不同相位
            phase_offset = i * (2 * math.pi / len(self.target_joints))
            phase = 2 * math.pi * self.sine_frequency * current_time + phase_offset

            position = self.sine_amplitude * math.sin(phase)
            velocity = self.sine_amplitude * 2 * math.pi * self.sine_frequency * math.cos(phase)

            positions[joint_id] = {
                'q': position,
                'dq': velocity,
                'kp': 80.0 if joint_id in [4, 5, 10, 11] else 100.0,  # 脚踝用小齿轮参数
                'kd': 2.0 if joint_id in [4, 5, 10, 11] else 3.0,
                'tau': 0.0
            }

        return positions

    def create_lowcmd_message(self, current_time):
        """创建LowCmd消息"""
        positions = self.generate_sine_wave_positions(current_time)

        # 创建PoseStamped消息作为/lowcmd的数据载体
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "robot_base"

        # 将关节位置映射到pose
        # 使用前4个关节的位置映射到pose的6个自由度
        joint_ids = sorted(positions.keys())[:4]

        if len(joint_ids) >= 1:
            msg.pose.position.x = positions[joint_ids[0]]['q']
        if len(joint_ids) >= 2:
            msg.pose.position.y = positions[joint_ids[1]]['q']
        if len(joint_ids) >= 3:
            msg.pose.position.z = positions[joint_ids[2]]['q']
        if len(joint_ids) >= 4:
            # 使用四元数表示第四个关节
            angle = positions[joint_ids[3]]['q']
            msg.pose.orientation.x = math.sin(angle/2) * 0.0
            msg.pose.orientation.y = math.sin(angle/2) * 0.0
            msg.pose.orientation.z = math.sin(angle/2) * 1.0
            msg.pose.orientation.w = math.cos(angle/2)

        return msg, positions

    def create_joint_state_message(self, positions):
        """创建关节状态消息"""
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()

        # G1关节名称
        joint_names = [
            'left_hip_pitch', 'left_hip_roll', 'left_hip_yaw', 'left_knee',
            'left_ankle_pitch', 'left_ankle_roll', 'right_hip_pitch', 'right_hip_roll',
            'right_hip_yaw', 'right_knee', 'right_ankle_pitch', 'right_ankle_roll'
        ]

        # 为脚踝关节创建名称和位置
        ankle_joint_names = ['left_ankle_pitch', 'left_ankle_roll', 'right_ankle_pitch', 'right_ankle_roll']
        ankle_positions = [0.0] * 4
        ankle_velocities = [0.0] * 4
        ankle_efforts = [0.0] * 4

        for joint_id, pos_data in positions.items():
            if joint_id in [4, 5, 10, 11]:  # 脚踝关节
                index = [4, 5, 10, 11].index(joint_id)
                ankle_positions[index] = pos_data['q']
                ankle_velocities[index] = pos_data['dq']
                ankle_efforts[index] = pos_data['tau']

        joint_state.name = ankle_joint_names
        joint_state.position = ankle_positions
        joint_state.velocity = ankle_velocities
        joint_state.effort = ankle_efforts

        return joint_state

    def send_network_data(self, positions):
        """通过网络发送数据到RK3588"""
        if not self.use_network or not hasattr(self, 'socket'):
            return

        try:
            # 创建motor_cmd字典，包含所有30个电机 (ID: 1-30)
            motor_cmd = {}
            for motor_id in range(1, self.num_motors + 1):
                if motor_id in positions:
                    # 有数据的电机
                    motor_cmd[str(motor_id)] = {
                        'mode': 1 if motor_id in self.target_joints else 0,
                        'q': positions[motor_id]['q'],
                        'dq': positions[motor_id]['dq'],
                        'tau': positions[motor_id]['tau'],
                        'kp': positions[motor_id]['kp'],
                        'kd': positions[motor_id]['kd']
                    }
                else:
                    # 没有数据的电机，设为默认值
                    motor_cmd[str(motor_id)] = {
                        'mode': 0,
                        'q': 0.0,
                        'dq': 0.0,
                        'tau': 0.0,
                        'kp': 0.0,
                        'kd': 0.0
                    }

            # 创建网络数据包，使用xixilowcmd格式
            data = {
                'timestamp': time.time(),
                'sequence': self.msg_count,
                'mode_pr': 0,  # PR模式
                'mode_machine': 1,  # G1机器人
                'motor_cmd': motor_cmd  # 使用motor_cmd格式
            }

            # 转换为JSON并编码
            json_str = json.dumps(data)
            json_bytes = json_str.encode('utf-8')

            # 发送到RK3588 (192.168.1.10:8888)
            self.socket.sendto(json_bytes, (self.rk3588_ip, self.rk3588_port))

        except Exception as e:
            rospy.logwarn(f"Failed to send network data: {e}")

    def write_dds_bridge_file(self, positions):
        """写入DDS桥接文件"""
        if not self.use_dds_bridge:
            return

        try:
            bridge_data = {
                'timestamp': time.time(),
                'sequence': self.msg_count,
                'mode_pr': 1,
                'mode_machine': 1,
                'joints': {str(joint_id): pos_data for joint_id, pos_data in positions.items()},
                'ros_topic': '/lowcmd'
            }

            with open('/tmp/lowcmd_data.json', 'w') as f:
                json.dump(bridge_data, f, indent=2)

        except Exception as e:
            rospy.logwarn(f"Failed to write bridge file: {e}")

    def publish_debug_info(self):
        """发布调试信息"""
        debug_msg = Header()
        debug_msg.stamp = rospy.Time.now()
        debug_msg.frame_id = f"msg_count_{self.msg_count}_time_{time.time():.3f}"
        self.debug_pub.publish(debug_msg)

    def log_status(self):
        """输出状态信息"""
        if self.msg_count % 100 == 0:
            elapsed_time = time.time() - self.start_time
            actual_rate = self.msg_count / elapsed_time

            rospy.loginfo(f"📊 Published {self.msg_count} messages")
            rospy.loginfo(f"📈 Actual rate: {actual_rate:.1f} Hz (target: {self.publish_rate} Hz)")
            rospy.loginfo(f"⏱️  Running time: {elapsed_time:.1f}s")

    def update_and_publish(self):
        """更新并发布消息"""
        try:
            current_time = time.time() - self.start_time

            # 生成LowCmd消息
            lowcmd_msg, positions = self.create_lowcmd_message(current_time)

            # 发布到ROS1话题
            self.lowcmd_pub.publish(lowcmd_msg)

            # 发布关节状态
            joint_state_msg = self.create_joint_state_message(positions)
            self.joint_state_pub.publish(joint_state_msg)

            # 网络传输
            self.send_network_data(positions)

            # DDS桥接文件
            self.write_dds_bridge_file(positions)

            # 调试信息
            if self.msg_count % 50 == 0:
                self.publish_debug_info()

            self.msg_count += 1

            # 状态日志
            self.log_status()

        except Exception as e:
            rospy.logerr(f"Error in update_and_publish: {e}")

    def run(self):
        """运行发布器"""
        rospy.loginfo("🚀 Starting to publish /lowcmd messages...")
        rospy.loginfo("📡 Publishing to ROS1 topic: /lowcmd")
        rospy.loginfo("🌐 Network destination: {}:{}".format(self.rk3588_ip, self.rk3588_port))
        rospy.loginfo("📄 Bridge file: /tmp/lowcmd_data.json")
        rospy.loginfo("⏹️  Press Ctrl+C to stop")

        rate = rospy.Rate(self.publish_rate)

        try:
            while not rospy.is_shutdown():
                self.update_and_publish()
                rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("🛑 Shutting down Jetson LowCmd Publisher...")
        finally:
            self.cleanup()

    def cleanup(self):
        """清理资源"""
        if hasattr(self, 'socket'):
            self.socket.close()

        # 清理桥接文件
        try:
            import os
            os.remove('/tmp/lowcmd_data.json')
        except:
            pass

def main():
    """主函数"""
    try:
        publisher = JetsonLowCmdPublisher()
        publisher.run()
    except Exception as e:
        rospy.logerr(f"Failed to start publisher: {e}")

if __name__ == '__main__':
    main()