#!/usr/bin/env python3
"""
订阅motor feedback话题的示例脚本
"""

import rospy
from motor_controller.msg import MotorFeedback
import numpy as np

class MotorFeedbackListener:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('motor_feedback_listener', anonymous=True)

        # 订阅话题
        self.sub = rospy.Subscriber('/motor_feedback', MotorFeedback, self.callback)

        # 存储最近的数据
        self.latest_data = {}

        # 打印初始化信息
        rospy.loginfo("Motor Feedback Listener Started")
        rospy.loginfo("Subscribed to /motor_feedback")
        rospy.loginfo("Waiting for data...")

    def callback(self, msg):
        """回调函数，处理接收到的motor feedback消息"""

        # 存储最新数据
        self.latest_data[msg.motor_id] = msg

        # 打印接收到的数据
        rospy.loginfo(f"\n=== Motor {msg.motor_id} Feedback ===")
        rospy.loginfo(f"CAN ID: 0x{msg.can_id:03X}")
        rospy.loginfo(f"Position: {msg.position:.4f} rad")
        rospy.loginfo(f"Velocity: {msg.velocity:.4f} rad/s")
        rospy.loginfo(f"Torque: {msg.torque:.4f} Nm")
        rospy.loginfo(f"Temperature MOS: {msg.temperature_mos:.1f}°C")
        rospy.loginfo(f"Temperature Coil: {msg.temperature_coil:.1f}°C")

        if msg.error:
            rospy.logwarn(f"Motor {msg.motor_id} has ERROR (code: {msg.error_code})")

    def get_motor_data(self, motor_id):
        """获取特定电机的最新数据"""
        return self.latest_data.get(motor_id, None)

    def print_summary(self):
        """打印所有电机的摘要"""
        if not self.latest_data:
            print("\nNo motor data received yet")
            return

        print("\n" + "="*50)
        print("MOTOR STATUS SUMMARY")
        print("="*50)

        for motor_id in sorted(self.latest_data.keys()):
            msg = self.latest_data[motor_id]
            status = "OK" if not msg.error else f"ERROR({msg.error_code})"
            print(f"Motor {msg.id:2d}: pos={msg.position:7.3f} rad  "
                  f"vel={msg.velocity:6.3f} rad/s  "
                  f"torque={msg.torque:6.2f} Nm  "
                  f"status={status}")

def main():
    try:
        listener = MotorFeedbackListener()

        # 主循环，每2秒打印一次摘要
        rate = rospy.Rate(0.5)  # 0.5 Hz
        while not rospy.is_shutdown():
            listener.print_summary()
            rate.sleep()

    except rospy.ROSInterruptException:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()