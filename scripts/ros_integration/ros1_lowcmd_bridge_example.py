#!/usr/bin/env python3
"""
ROS1 LowCmd Bridge Example
展示如何在ROS1环境中构建和发送LowCmd格式数据

Author: Claude Code Assistant
Date: 2025-12-17
"""

import rospy
import time
import math
import json
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState

from python_lowcmd_builder import LowCmdBuilder, create_lowcmd_for_g1

class ROS1LowCmdPublisher:
    """ROS1 LowCmd发布器"""

    def __init__(self):
        # 初始化ROS1节点
        rospy.init_node('ros1_lowcmd_publisher', anonymous=True)

        # 配置参数
        self.publish_rate = rospy.get_param('~publish_rate', 50)  # Hz
        self.sine_amplitude = rospy.get_param('~sine_amplitude', 0.3)
        self.sine_frequency = rospy.get_param('~sine_frequency', 0.5)
        self.use_bridge_file = rospy.get_param('~use_bridge_file', True)
        self.bridge_file_path = rospy.get_param('~bridge_file_path', '/tmp/lowcmd_data.json')

        # 创建LowCmd构建器
        self.lowcmd = create_lowcmd_for_g1()

        # 调整正弦波参数
        self.lowcmd.set_sine_wave_motors(
            motor_ids=[4, 5, 10, 11],  # 脚踝关节
            amplitude=self.sine_amplitude,
            frequency=self.sine_frequency
        )

        # ROS1发布者
        self.pose_pub = rospy.Publisher('/lowcmd', PoseStamped, queue_size=10)
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # 统计信息
        self.msg_count = 0
        self.start_time = time.time()

        rospy.loginfo("ROS1 LowCmd Publisher initialized")
        rospy.loginfo(f"Publish rate: {self.publish_rate} Hz")
        rospy.loginfo(f"Sine wave: {self.sine_amplitude} rad @ {self.sine_frequency} Hz")
        rospy.loginfo(f"Bridge file: {self.bridge_file_path}")

    def update_and_publish(self):
        """更新LowCmd数据并发布"""
        try:
            # 更新正弦波位置
            self.lowcmd.update_positions()

            # 发布ROS1消息
            self.publish_pose_message()
            self.publish_joint_state_message()

            # 写入桥接文件
            if self.use_bridge_file:
                self.write_bridge_file()

            self.msg_count += 1

            # 每100次循环输出状态
            if self.msg_count % 100 == 0:
                self.log_status()

        except Exception as e:
            rospy.logerr(f"Error in update_and_publish: {e}")

    def publish_pose_message(self):
        """发布PoseStamped消息"""
        pose_msg = PoseStamped()

        # 设置头部
        pose_msg.header = Header()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "robot_base"

        # 从LowCmd提取位置信息映射到pose
        # 左脚踝pitch -> position.x
        # 左脚踝roll -> position.y
        # 右脚踝pitch -> orientation.x
        # 右脚踝roll -> orientation.y
        motors = self.lowcmd.motor_commands

        pose_msg.pose.position = Point(
            x=motors[4].q if len(motors) > 4 else 0.0,  # Left ankle pitch
            y=motors[5].q if len(motors) > 5 else 0.0,  # Left ankle roll
            z=0.0
        )

        pose_msg.pose.orientation = Quaternion(
            x=motors[10].q if len(motors) > 10 else 0.0,  # Right ankle pitch
            y=motors[11].q if len(motors) > 11 else 0.0,  # Right ankle roll
            z=0.0,
            w=1.0
        )

        self.pose_pub.publish(pose_msg)

    def publish_joint_state_message(self):
        """发布关节状态消息"""
        joint_state = JointState()

        # 设置头部
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()

        # 设置关节名称（基于G1机器人）
        joint_names = [
            'left_hip_pitch', 'left_hip_roll', 'left_hip_yaw', 'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
            'right_hip_pitch', 'right_hip_roll', 'right_hip_yaw', 'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
            'torso_joint', 'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw', 'left_elbow',
            'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw', 'right_elbow',
            'left_wrist_pitch', 'left_wrist_roll', 'left_wrist_yaw', 'right_wrist_pitch', 'right_wrist_roll', 'right_wrist_yaw',
            'head_pitch', 'head_yaw', 'head_roll', 'waist_pitch', 'waist_roll', 'waist_yaw'
        ]

        joint_state.name = joint_names[:len(self.lowcmd.motor_commands)]

        # 设置位置、速度、力矩
        positions = []
        velocities = []
        efforts = []

        for motor in self.lowcmd.motor_commands:
            positions.append(motor.q)
            velocities.append(motor.dq)
            efforts.append(motor.tau)

        joint_state.position = positions
        joint_state.velocity = velocities
        joint_state.effort = efforts

        self.joint_state_pub.publish(joint_state)

    def write_bridge_file(self):
        """写入桥接文件供RK3588使用"""
        try:
            # 转换LowCmd为字典格式
            lowcmd_dict = self.lowcmd.to_dict()

            # 添加时间戳和统计信息
            bridge_data = {
                'timestamp': time.time(),
                'sequence': self.msg_count,
                'lowcmd': lowcmd_dict,
                'ros_topics': {
                    '/lowcmd': {
                        'position_x': self.lowcmd.motor_commands[4].q if len(self.lowcmd.motor_commands) > 4 else 0.0,
                        'position_y': self.lowcmd.motor_commands[5].q if len(self.lowcmd.motor_commands) > 5 else 0.0,
                        'orientation_x': self.lowcmd.motor_commands[10].q if len(self.lowcmd.motor_commands) > 10 else 0.0,
                        'orientation_y': self.lowcmd.motor_commands[11].q if len(self.lowcmd.motor_commands) > 11 else 0.0
                    }
                }
            }

            # 写入文件
            with open(self.bridge_file_path, 'w') as f:
                json.dump(bridge_data, f, indent=2)

        except Exception as e:
            rospy.logwarn(f"Failed to write bridge file: {e}")

    def log_status(self):
        """输出状态信息"""
        elapsed_time = time.time() - self.start_time
        actual_rate = self.msg_count / elapsed_time

        rospy.loginfo(f"Published {self.msg_count} messages")
        rospy.loginfo(f"Actual rate: {actual_rate:.1f} Hz (target: {self.publish_rate} Hz)")

        # 显示当前脚踝关节状态
        if len(self.lowcmd.motor_commands) > 11:
            rospy.loginfo(f"Ankle joints - Left: ({self.lowcmd.motor_commands[4].q:.3f}, {self.lowcmd.motor_commands[5].q:.3f}), "
                         f"Right: ({self.lowcmd.motor_commands[10].q:.3f}, {self.lowcmd.motor_commands[11].q:.3f})")

    def run(self):
        """运行发布器"""
        rospy.loginfo("Starting ROS1 LowCmd Publisher...")

        rate = rospy.Rate(self.publish_rate)

        try:
            while not rospy.is_shutdown():
                self.update_and_publish()
                rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("Shutting down ROS1 LowCmd Publisher...")

def main():
    """主函数"""
    try:
        publisher = ROS1LowCmdPublisher()
        publisher.run()
    except Exception as e:
        rospy.logerr(f"Failed to start publisher: {e}")

if __name__ == '__main__':
    main()