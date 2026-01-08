#!/usr/bin/env python3
"""
详细显示/lowcmd话题的内容
"""

import rclpy
from rclpy.node import Node
from xixilowcmd.msg import LowCmd

class LowCmdEchoer(Node):
    def __init__(self):
        super().__init__('lowcmd_echoer')
        self.subscription = self.create_subscription(
            LowCmd,
            '/lowcmd',
            self.callback,
            10
        )
        self.count = 0
        self.show_all = False  # 默认只显示非零电机

    def callback(self, msg):
        self.count += 1

        # 每10条消息显示一次详细信息
        if self.count % 10 == 0:
            print("\n" + "="*60)
            print(f"📨 消息 #{self.count} - 包含 {len(msg.motor_cmd)} 个电机命令")
            print("="*60)

            # 找出有数据的电机（mode!=0或q!=0）
            active_motors = []
            for motor in msg.motor_cmd:
                if motor.mode != 0 or motor.q != 0 or motor.kp != 0 or motor.kd != 0:
                    active_motors.append(motor)

            if active_motors:
                print(f"\n活跃电机 ({len(active_motors)} 个):")
                for motor in active_motors:
                    print(f"  id: {motor.id}")
                    print(f"    mode: {motor.mode}")
                    print(f"    q: {motor.q}")
                    print(f"    dq: {motor.dq}")
                    print(f"    kp: {motor.kp}")
                    print(f"    kd: {motor.kd}")
                    print(f"    tau: {motor.tau}")
                    print()
            else:
                print("\n  (所有电机模式为0，无有效数据)")

        # 每50条显示一次统计
        if self.count % 50 == 0:
            active_count = sum(1 for m in msg.motor_cmd if m.mode != 0 or m.q != 0)
            print(f"[{self.count}] 活跃电机: {active_count}/{len(msg.motor_cmd)}")

def main():
    print("📡 监听 /lowcmd 话题...")
    print("   每10条消息显示一次详细信息")
    print("   按 Ctrl+C 停止")
    print("")

    rclpy.init()
    echoer = LowCmdEchoer()

    try:
        rclpy.spin(echoer)
    except KeyboardInterrupt:
        print(f"\n\n📊 总计接收 {echoer.count} 条消息")

    echoer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
