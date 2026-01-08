#!/usr/bin/env python3
"""
监听ROS2 /lowcmd话题，显示30个电机的数据
"""

import rclpy
from rclpy.node import Node
from xixilowcmd.msg import LowCmd
import time

class LowCmdMonitor(Node):
    def __init__(self):
        super().__init__('lowcmd_monitor')
        self.subscription = self.create_subscription(
            LowCmd,
            '/lowcmd',
            self.lowcmd_callback,
            10
        )
        self.msg_count = 0
        self.start_time = time.time()

        # 关键脚踝关节
        self.key_motors = [4, 5, 10, 11]

        self.get_logger().info("=" * 60)
        self.get_logger().info("   ROS2 /lowcmd 监听器")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"   监听30个电机 (motor_id: 1-30)")
        self.get_logger().info(f"   关键脚踝关节: {self.key_motors}")
        self.get_logger().info("")

    def lowcmd_callback(self, msg):
        self.msg_count += 1
        current_time = time.time()
        elapsed = current_time - self.start_time

        # 每秒显示一次频率
        if self.msg_count % 50 == 0:
            freq = self.msg_count / elapsed
            self.get_logger().info(f"📊 已接收 {self.msg_count} 条消息, 频率: {freq:.1f} Hz")

        # 前5条消息显示完整详情
        if self.msg_count <= 5:
            self.print_detailed_info(msg)

        # 每100条消息显示关键电机状态
        elif self.msg_count % 100 == 0:
            self.print_key_motors(msg)

    def print_detailed_info(self, msg):
        """打印详细的电机命令信息"""
        print(f"\n{'='*70}")
        print(f"📨 消息 #{self.msg_count} - 包含 {len(msg.motor_cmd)} 个电机命令")
        print(f"{'='*70}")

        # 显示所有电机（按行分组）
        for motor in msg.motor_cmd:
            motor_id = motor.id
            mode_str = self.get_mode_string(motor.mode)

            # 关键电机或非零值电机显示详细信息
            if motor_id in self.key_motors or motor.mode != 0:
                print(f"  Motor {motor_id:2d}: mode={mode_str:6s} | "
                      f"q={motor.q:7.4f} | dq={motor.dq:6.3f} | "
                      f"tau={motor.tau:6.3f} | kp={motor.kp:5.1f} | kd={motor.kd:4.1f}")

        # 统计活跃电机
        active_count = sum(1 for m in msg.motor_cmd if m.mode != 0)
        print(f"\n  活跃电机: {active_count}/{len(msg.motor_cmd)}")

    def print_key_motors(self, msg):
        """打印关键脚踝电机状态"""
        print(f"\n[{self.msg_count}] 关键脚踝电机状态:")

        for motor_id in self.key_motors:
            # 查找对应motor_id的电机
            motor_data = None
            for motor in msg.motor_cmd:
                if motor.id == motor_id:
                    motor_data = motor
                    break

            if motor_data:
                print(f"  Motor {motor_id}: q={motor_data.q:7.4f}, "
                      f"dq={motor_data.dq:6.3f}, tau={motor_data.tau:6.3f}, "
                      f"mode={motor_data.mode}, kp={motor_data.kp:5.1f}, kd={motor_data.kd:4.1f}")

    def get_mode_string(self, mode):
        """获取模式字符串"""
        mode_map = {
            0: "无控制",
            1: "位置",
            2: "速度",
            3: "力矩"
        }
        return mode_map.get(mode, f"模式{mode}")

def main(args=None):
    rclpy.init(args=args)

    monitor = LowCmdMonitor()

    try:
        print("🎯 开始监听 /lowcmd 话题...")
        print("📡 按 Ctrl+C 停止监听")
        print("")

        rclpy.spin(monitor)

    except KeyboardInterrupt:
        print("\n\n🛑 停止监听")

        # 显示统计信息
        elapsed = time.time() - monitor.start_time
        print(f"\n📊 统计信息:")
        print(f"   总消息数: {monitor.msg_count}")
        print(f"   运行时间: {elapsed:.1f} 秒")
        if elapsed > 0:
            print(f"   平均频率: {monitor.msg_count/elapsed:.1f} Hz")

    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
