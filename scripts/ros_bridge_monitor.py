#!/usr/bin/env python3
"""
ROS Bridge Output Monitor
监控ros1_to_dds_bridge.py的输出情况
"""

import rospy
import time
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

class RosBridgeMonitor:
    def __init__(self):
        rospy.init_node('ros_bridge_monitor', anonymous=True)

        self.message_count = 0
        self.start_time = time.time()

        # 订阅相关话题
        self.lowcmd_sub = rospy.Subscriber('/lowcmd', PoseStamped, self.lowcmd_callback)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        rospy.loginfo("ROS Bridge Monitor Started")
        rospy.loginfo("Monitoring ROS1 -> DDS Bridge output...")
        rospy.loginfo("=" * 60)

        # 定时打印统计信息
        rospy.Timer(rospy.Duration(5.0), self.print_stats)

    def lowcmd_callback(self, msg):
        self.message_count += 1

        # 打印每次收到的/lowcmd消息
        elapsed = time.time() - self.start_time
        frequency = self.message_count / elapsed if elapsed > 0 else 0

        print(f"📨 ROS1 /lowcmd #{self.message_count} (t={elapsed:.1f}s, {frequency:.1f}Hz)")
        print(f"   Header: frame_id={msg.header.frame_id}")
        print(f"   Position: x={msg.pose.position.x:.4f}, y={msg.pose.position.y:.4f}, z={msg.pose.position.z:.4f}")
        print(f"   Orientation: x={msg.pose.orientation.x:.4f}, y={msg.pose.orientation.y:.4f}, z={msg.pose.orientation.z:.4f}, w={msg.pose.orientation.w:.4f}")

        # 解析脚踝关节位置 (假设前4个脚踝关节映射到pose)
        ankle_positions = {
            "LeftAnklePitch": msg.pose.position.x,    # 通常对应G1关节4
            "LeftAnkleRoll": msg.pose.position.y,     # 通常对应G1关节5
            "RightAnklePitch": msg.pose.orientation.x, # 通常对应G1关节10
            "RightAnkleRoll": msg.pose.orientation.y   # 通常对应G1关节11
        }

        print("   🎯 脚踝关节解析:")
        for joint, pos in ankle_positions.items():
            print(f"      {joint}: {pos:.4f} rad")
        print("-" * 40)

    def joint_state_callback(self, msg):
        if self.message_count % 10 == 0:  # 每10个lowcmd消息打印一次joint_state
            print(f"🔧 Joint States Update:")
            for i, (name, pos) in enumerate(zip(msg.name, msg.position)):
                if i < 4:  # 只显示前4个
                    print(f"   {name}: {pos:.4f} rad")
            print()

    def print_stats(self, event):
        elapsed = time.time() - self.start_time
        frequency = self.message_count / elapsed if elapsed > 0 else 0

        print(f"\n📊 统计信息 (运行 {elapsed:.1f}s):")
        print(f"   /lowcmd 消息数: {self.message_count}")
        print(f"   平均频率: {frequency:.1f} Hz")
        print(f"   ROS Bridge 状态: {'✅ 正常' if frequency > 0 else '⚠️ 无数据'}")

        # 检查ROS话题
        topics = rospy.get_published_topics()
        lowcmd_topics = [t for t in topics if '/lowcmd' in t[0]]
        joint_topics = [t for t in topics if '/joint_states' in t[0]]

        print(f"   检测到的/lowcmd话题: {len(lowcmd_topics)}")
        print(f"   检测到的/joint_states话题: {len(joint_topics)}")
        print("=" * 60)

def main():
    try:
        monitor = RosBridgeMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("\n🛑 ROS Bridge Monitor stopped")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == '__main__':
    main()