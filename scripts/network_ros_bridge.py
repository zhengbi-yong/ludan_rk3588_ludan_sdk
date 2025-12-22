#!/usr/bin/env python3
"""
Network ROS Bridge to Jetson
连接到Jetson的ROS master并桥接数据到本地DDS

Author: Claude Code Assistant
Date: 2025-12-18
"""

import rospy
import time
import math
import numpy as np
import sys
import os
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

# 添加Unitree SDK路径
try:
    from unitree.idl.hg.LowCmd_ import LowCmd_
    from unitree.idl.hg.LowCmd_ import MotorCmd_
    from unitree.robot.channel.channel_publisher import ChannelPublisher
    from unitree.robot.channel.channel_factory import ChannelFactory
    UNITREE_AVAILABLE = True
    print("✅ Unitree SDK2 found")
except ImportError as e:
    print(f"❌ Unitree SDK2 not found: {e}")
    UNITREE_AVAILABLE = False

class NetworkROSBridge:
    def __init__(self, jetson_master="http://192.168.0.139:11311", network_interface="eth0"):
        if not UNITREE_AVAILABLE:
            raise ImportError("Unitree SDK2 is required")

        # 设置ROS Master环境变量连接到Jetson
        os.environ['ROS_MASTER_URI'] = jetson_master

        try:
            rospy.init_node('network_ros_bridge', anonymous=True)
        except Exception as e:
            print(f"❌ Failed to connect to ROS master at {jetson_master}: {e}")
            raise

        self.message_count = 0
        self.start_time = time.time()

        # 初始化DDS发布器
        try:
            ChannelFactory.Instance().Init(0, network_interface)
            self.dds_publisher = ChannelPublisher(LowCmd_, "rt/lowcmd")
            self.dds_publisher.InitChannel()
            print(f"✅ DDS publisher initialized on rt/lowcmd")
            print(f"✅ Using network interface: {network_interface}")
        except Exception as e:
            print(f"❌ Failed to initialize DDS: {e}")
            raise

        # 订阅Jetson的ROS话题
        print(f"🔗 Connecting to Jetson ROS master: {jetson_master}")
        self.lowcmd_sub = rospy.Subscriber('/lowcmd', PoseStamped, self.lowcmd_callback)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        print("🎯 Network ROS Bridge started")
        print("📡 Listening to Jetson ROS1 /lowcmd")
        print("📤 Publishing to local DDS rt/lowcmd")
        print("=" * 50)

    def lowcmd_callback(self, msg):
        self.message_count += 1
        current_time = time.time()
        elapsed = current_time - self.start_time
        frequency = self.message_count / elapsed if elapsed > 0 else 0

        try:
            # 创建LowCmd DDS消息
            lowcmd = LowCmd_()

            # 设置基本参数
            lowcmd.mode_pr = 1      # PR模式
            lowcmd.mode_machine = 1 # G1类型

            # 初始化所有电机命令 (G1有29个电机)
            lowcmd.motor_cmd.resize(29)
            for i in range(29):
                motor = MotorCmd_()
                motor.mode = 0
                motor.q = 0.0
                motor.dq = 0.0
                motor.tau = 0.0
                motor.kp = 0.0
                motor.kd = 0.0
                motor.reserve = [0, 0, 0]
                lowcmd.motor_cmd[i] = motor

            # 将PoseStamped映射到脚踝关节 (G1关节映射)
            # 关节4: LeftAnklePitch, 关节5: LeftAnkleRoll
            # 关节10: RightAnklePitch, 关节11: RightAnkleRoll

            ankle_joints = [
                (4, msg.pose.position.x),   # LeftAnklePitch
                (5, msg.pose.position.y),   # LeftAnkleRoll
                (10, msg.pose.orientation.x), # RightAnklePitch
                (11, msg.pose.orientation.y)  # RightAnkleRoll
            ]

            for joint_id, value in ankle_joints:
                if 0 <= joint_id < 29:
                    motor = lowcmd.motor_cmd[joint_id]
                    motor.mode = 1
                    motor.q = float(value)
                    motor.dq = 0.0  # 简化处理
                    motor.tau = 0.0
                    motor.kp = 80.0  # 脚踝用小齿轮参数
                    motor.kd = 2.0

            # 设置CRC (简化处理)
            lowcmd.crc = 0x12345678

            # 发送到本地DDS
            self.dds_publisher.Write(lowcmd)

            # 每100条消息打印一次状态
            if self.message_count % 100 == 0:
                print(f"📨 Jetson→DDS #{self.message_count} (t={elapsed:.1f}s, {frequency:.1f}Hz)")
                print(f"   L_pitch={msg.pose.position.x:.4f}, L_roll={msg.pose.position.y:.4f}")
                print(f"   R_pitch={msg.pose.orientation.x:.4f}, R_roll={msg.pose.orientation.y:.4f}")
                print(f"   ✅ 发送到本地DDS rt/lowcmd")

        except Exception as e:
            print(f"❌ Error processing Jetson message: {e}")
            import traceback
            traceback.print_exc()

    def joint_state_callback(self, msg):
        # 可选：处理joint_states消息
        pass

    def run(self):
        rospy.loginfo("Network ROS Bridge is running...")
        rospy.loginfo("Connected to Jetson ROS master")
        rospy.spin()

def main():
    try:
        # Jetson ROS master地址
        jetson_master = "http://192.168.0.139:11311"
        network_interface = "eth0"

        print("🚀 Starting Network ROS Bridge to Jetson...")
        print(f"🌐 Jetson ROS Master: {jetson_master}")
        print(f"🔌 Network Interface: {network_interface}")
        print()

        bridge = NetworkROSBridge(jetson_master, network_interface)
        bridge.run()

    except rospy.ROSInterruptException:
        print("🛑 Bridge stopped by user")
    except Exception as e:
        print(f"❌ Bridge failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()