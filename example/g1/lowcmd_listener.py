#!/usr/bin/env python3
"""
监听 /lowcmd topic 的 ROS2 消息
显示接收到的电机命令数据
"""

import rclpy
from rclpy.node import Node
from xixilowcmd.msg import LowCmd
import time


class LowCmdListener(Node):
    def __init__(self):
        super().__init__('lowcmd_listener')

        # 订阅 /lowcmd topic
        self.subscription = self.create_subscription(
            LowCmd,
            '/lowcmd',
            self.lowcmd_callback,
            10  # QoS depth
        )

        self.count = 0
        self.start_time = time.time()

        self.get_logger().info('LowCmd Listener 已启动')
        self.get_logger().info('监听 topic: /lowcmd')

    def lowcmd_callback(self, msg):
        self.count += 1
        current_time = time.time()
        elapsed = current_time - self.start_time

        # 每 50 条消息显示一次统计，或者前 5 条消息
        if self.count <= 5 or self.count % 50 == 0:
            rate = self.count / elapsed if elapsed > 0 else 0

            print(f"\n{'='*60}")
            print(f"消息 #{self.count} (时间: {elapsed:.2f}s, 频率: {rate:.1f} Hz)")
            print(f"{'='*60}")

            # 统计活跃电机 (mode != 0)
            active_motors = []
            for cmd in msg.motor_cmd:
                if cmd.mode != 0:
                    active_motors.append({
                        'id': cmd.id,
                        'q': cmd.q,
                        'dq': cmd.dq,
                        'tau': cmd.tau,
                        'kp': cmd.kp,
                        'kd': cmd.kd,
                        'mode': cmd.mode
                    })

            if active_motors:
                print(f"\n活跃电机数量: {len(active_motors)}")
                print(f"\n{'ID':<4} {'位置(q)':<10} {'速度(dq)':<10} {'力矩(tau)':<10} {'kp':<8} {'kd':<8} {'mode':<6}")
                print("-" * 66)

                for motor in active_motors:
                    print(f"{motor['id']:<4} "
                          f"{motor['q']:<10.4f} "
                          f"{motor['dq']:<10.4f} "
                          f"{motor['tau']:<10.4f} "
                          f"{motor['kp']:<8.2f} "
                          f"{motor['kd']:<8.2f} "
                          f"{motor['mode']:<6}")
            else:
                print("\n无活跃电机 (所有电机 mode=0)")

            # 显示统计信息
            inactive_count = len([cmd for cmd in msg.motor_cmd if cmd.mode == 0])
            print(f"\n总电机数: {len(msg.motor_cmd)} (活跃: {len(active_motors)}, 非活跃: {inactive_count})")

        # 简洁模式：每 10 条显示一行点
        elif self.count % 10 == 0:
            print(".", end="", flush=True)


def main(args=None):
    rclpy.init(args=args)

    lowcmd_listener = LowCmdListener()

    print("\n" + "="*60)
    print("  LowCmd Topic 监听器")
    print("="*60)
    print("\n按 Ctrl+C 停止监听\n")

    try:
        rclpy.spin(lowcmd_listener)
    except KeyboardInterrupt:
        print("\n\n监听已停止")

        # 显示最终统计
        elapsed = time.time() - lowcmd_listener.start_time
        rate = lowcmd_listener.count / elapsed if elapsed > 0 else 0
        print(f"\n最终统计:")
        print(f"  总消息数: {lowcmd_listener.count}")
        print(f"  总时间:   {elapsed:.2f} 秒")
        print(f"  平均频率: {rate:.1f} Hz")

    finally:
        lowcmd_listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
