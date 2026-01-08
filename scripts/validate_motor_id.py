#!/usr/bin/env python3
"""验证motor_id是否正确设置为1-30"""
import time
import rclpy
from rclpy.node import Node
from xixilowcmd.msg import LowCmd, MotorCmd

class TestValidator(Node):
    def __init__(self):
        super().__init__('test_validator')
        self.sub = self.create_subscription(LowCmd, '/lowcmd', self.callback, 10)
        self.received = False

    def callback(self, msg):
        if self.received:
            return
        self.received = True

        print("\n" + "="*60)
        print("✅ 接收到 /lowcmd 消息")
        print("="*60)
        print(f"数组长度: {len(msg.motor_cmd)}")

        # 检查所有电机ID
        ids = [m.id for m in msg.motor_cmd]
        print(f"\nID范围: {min(ids)} - {max(ids)}")
        print(f"前5个ID: {ids[:5]}")
        print(f"后5个ID: {ids[-5:]}")

        # 检查是否有ID=30
        if 30 in ids:
            idx = ids.index(30)
            m = msg.motor_cmd[idx]
            print(f"\n✅ 找到电机30: mode={m.mode}, q={m.q}, kp={m.kp}, kd={m.kd}")
        else:
            print(f"\n❌ 未找到ID=30的电机！")

        # 检查ID=0是否存在
        if 0 in ids:
            print(f"⚠️  警告: 发现ID=0的电机！")

        # 检查关键测试电机
        test_motors = [12, 21, 30]
        for tid in test_motors:
            motors = [m for m in msg.motor_cmd if m.id == tid]
            if motors:
                m = motors[0]
                status = "✅" if m.q != 0 else "⚠️ "
                print(f"  {status} 电机{tid}: q={m.q}")

        print("="*60)
        rclpy.shutdown()

def main():
    print("等待 /lowcmd 消息...")
    print("请运行: python3 /home/linaro/ludan_sdk/scripts/test_send_30_motors.py")

    rclpy.init()
    validator = TestValidator()

    # 等待最多10秒
    for _ in range(100):
        rclpy.spin_once(validator, timeout_sec=0.1)
        if not rclpy.ok():
            break

    if not validator.received:
        print("\n❌ 超时：未收到 /lowcmd 消息")
        print("请确认listener正在运行")

    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
