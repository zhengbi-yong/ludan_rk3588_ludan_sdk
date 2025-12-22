#!/usr/bin/env python3
"""
Simple ROS1 to Unitree DDS Bridge
直接使用Unitree SDK2的DDS接口发送到rt/lowcmd
"""

import rospy
import time
import math
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

# 导入Unitree SDK2
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

class SimpleRosToDDSBridge:
    def __init__(self, network_interface="eth0"):
        if not UNITREE_AVAILABLE:
            raise ImportError("Unitree SDK2 is required")

        rospy.init_node('simple_ros_to_dds_bridge', anonymous=True)

        self.message_count = 0
        self.start_time = time.time()

        # 初始化Unitree DDS
        try:
            ChannelFactory.Instance().Init(0, network_interface)
            self.dds_publisher = ChannelPublisher(LowCmd_, "rt/lowcmd")
            self.dds_publisher.InitChannel()
            print(f"✅ DDS publisher initialized on rt/lowcmd")
            print(f"✅ Using network interface: {network_interface}")
        except Exception as e:
            print(f"❌ Failed to initialize DDS: {e}")
            raise

        # 订阅ROS1话题
        self.lowcmd_sub = rospy.Subscriber('/lowcmd', PoseStamped, self.lowcmd_callback)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        print("🎯 Simple ROS1 to DDS Bridge started")
        print("📡 Listening to ROS1 /lowcmd")
        print("📤 Publishing to DDS rt/lowcmd")
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
            lowcmd.mode_pr() = 1      # PR模式
            lowcmd.mode_machine() = 1 # G1类型

            # 初始化所有电机命令 (G1有29个电机)
            lowcmd.motor_cmd().resize(29)
            for i in range(29):
                motor = MotorCmd_()
                motor.mode() = 0
                motor.q() = 0.0
                motor.dq() = 0.0
                motor.tau() = 0.0
                motor.kp() = 0.0
                motor.kd() = 0.0
                motor.reserve() = [0, 0, 0]
                lowcmd.motor_cmd()[i] = motor

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
                    motor = lowcmd.motor_cmd()[joint_id]
                    motor.mode() = 1
                    motor.q() = float(value)
                    motor.dq() = 0.0  # 简化处理
                    motor.tau() = 0.0
                    motor.kp() = 80.0  # 脚踝用小齿轮参数
                    motor.kd() = 2.0

            # 设置CRC (简化处理)
            lowcmd.crc() = 0x12345678

            # 发送到DDS
            self.dds_publisher.Write(lowcmd)

            # 打印信息
            print(f"📨 ROS1->DDS #{self.message_count} (t={elapsed:.1f}s, {frequency:.1f}Hz)")
            print(f"   解析脚踝关节: L_pitch={msg.pose.position.x:.4f}, L_roll={msg.pose.position.y:.4f}")
            print(f"                   R_pitch={msg.pose.orientation.x:.4f}, R_roll={msg.pose.orientation.y:.4f}")
            print(f"   ✅ 已发送到DDS rt/lowcmd")

        except Exception as e:
            print(f"❌ Error processing ROS1 message: {e}")

    def joint_state_callback(self, msg):
        # 可选：处理joint_states消息
        pass

    def run(self):
        rospy.loginfo("Simple ROS1 to DDS Bridge is running...")
        rospy.spin()

def main():
    try:
        # 网络接口参数
        network_interface = rospy.get_param('~network_interface', 'eth0')

        bridge = SimpleRosToDDSBridge(network_interface)
        bridge.run()

    except rospy.ROSInterruptException:
        print("🛑 Bridge stopped by user")
    except Exception as e:
        print(f"❌ Bridge failed: {e}")

if __name__ == '__main__':
    main()