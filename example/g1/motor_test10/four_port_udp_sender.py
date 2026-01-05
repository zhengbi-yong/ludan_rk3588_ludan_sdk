#!/usr/bin/env python3
"""
四端口电机测试发送器
发送 UDP 数据到 listener，测试四个端口 (8000, 8001, 8002, 8003) 的数据下发

端口映射:
  端口 8000 (CAN0): 电机 1, 2, 3
  端口 8001 (CAN1): 电机 4
  端口 8002 (CAN2): 电机 5
  端口 8003 (CAN3): 电机 6
"""

import socket
import json
import time
import math
import argparse
from dataclasses import dataclass
from typing import Dict, List

@dataclass
class MotorConfig:
    id: int
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    kp: float = 20.0
    kd: float = 2.0
    mode: int = 1

class FourPortMotorTester:
    def __init__(self, target_ip="127.0.0.1", target_port=8888):
        self.target_ip = target_ip
        self.target_port = target_port
        self.socket = None
        self.message_count = 0

        # 创建 UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"✓ UDP Socket created: {self.target_ip}:{self.target_port}")

    def build_message(self, motors: List[MotorConfig]) -> dict:
        """构建 ROS2 兼容的 JSON 消息"""
        motor_cmd_dict = {}
        for motor in motors:
            motor_cmd_dict[str(motor.id)] = {
                "q": motor.q,
                "dq": motor.dq,
                "tau": motor.tau,
                "mode": motor.mode,
                "kp": motor.kp,
                "kd": motor.kd
            }

        message = {
            "timestamp": time.time(),
            "sequence": self.message_count,
            "motor_cmd": motor_cmd_dict,
            "mode_pr": 0,
            "mode_machine": 1
        }

        return message

    def send_message(self, message: dict) -> bool:
        """发送单条消息"""
        try:
            json_data = json.dumps(message, ensure_ascii=False)
            data_bytes = json_data.encode('utf-8')
            self.socket.sendto(data_bytes, (self.target_ip, self.target_port))
            self.message_count += 1
            return True
        except Exception as e:
            print(f"❌ Send failed: {e}")
            return False

    def send_single_test(self, motors: List[MotorConfig]):
        """发送单次测试数据包"""
        message = self.build_message(motors)

        print("\n" + "=" * 70)
        print("📤 发送 UDP 测试包")
        print("=" * 70)
        print(f"目标: {self.target_ip}:{self.target_port}")
        print(f"序列号: {self.message_count}")
        print(f"\n电机命令 ({len(motors)} 个):")
        print("-" * 70)
        print(f"{'ID':<4} {'位置(q)':<10} {'速度(dq)':<10} {'力矩':<10} {'Kp':<8} {'Kd':<8} {'Mode':<6}")
        print("-" * 70)
        for m in motors:
            print(f"{m.id:<4} {m.q:<10.4f} {m.dq:<10.3f} {m.tau:<10.3f} {m.kp:<8.1f} {m.kd:<8.1f} {m.mode:<6}")
        print("=" * 70)

        # 显示端口映射
        print("\n📋 端口映射:")
        port_mapping = self.get_port_mapping(motors)
        for port, motor_ids in port_mapping.items():
            if motor_ids:
                print(f"  端口 {port}: 电机 {motor_ids}")

        self.send_message(message)
        print(f"\n✓ 消息已发送 (总计: {self.message_count})")

    def get_port_mapping(self, motors: List[MotorConfig]) -> Dict[str, List[int]]:
        """获取端口映射"""
        mapping = {"8000": [], "8001": [], "8002": [], "8003": []}

        for motor in motors:
            if motor.id in [1, 2, 3]:
                mapping["8000"].append(motor.id)
            elif motor.id == 4:
                mapping["8001"].append(motor.id)
            elif motor.id == 5:
                mapping["8002"].append(motor.id)
            elif motor.id == 6:
                mapping["8003"].append(motor.id)

        return mapping

    def test_single_port(self, port_num: int):
        """测试单个端口"""
        port_motor_map = {
            0: [MotorConfig(id=1, q=0.5), MotorConfig(id=2, q=0.3), MotorConfig(id=3, q=-0.2)],
            1: [MotorConfig(id=4, q=0.5)],
            2: [MotorConfig(id=5, q=0.5)],
            3: [MotorConfig(id=6, q=0.5)]
        }

        if port_num not in port_motor_map:
            print(f"❌ 无效的端口号: {port_num}")
            return

        motors = port_motor_map[port_num]
        print(f"\n🔧 测试端口 800{port_num}")
        self.send_single_test(motors)

    def test_all_ports(self):
        """测试所有四个端口"""
        motors = [
            # 端口 8000 (CAN0): 电机 1, 2, 3
            MotorConfig(id=1, q=0.5, dq=0.0),
            MotorConfig(id=2, q=0.3, dq=0.0),
            MotorConfig(id=3, q=-0.2, dq=0.0),
            # 端口 8001 (CAN1): 电机 4
            MotorConfig(id=4, q=0.8, dq=0.0),
            # 端口 8002 (CAN2): 电机 5
            MotorConfig(id=5, q=0.6, dq=0.0),
            # 端口 8003 (CAN3): 电机 6
            MotorConfig(id=6, q=0.4, dq=0.0),
        ]

        print("\n" + "🎯" * 35)
        print("   测试所有四个端口 (8000, 8001, 8002, 8003)")
        print("🎯" * 35)
        self.send_single_test(motors)

    def send_continuous(self, duration_seconds: int = 10, frequency: int = 50):
        """持续发送测试数据"""
        motors = [
            MotorConfig(id=1, q=0.5), MotorConfig(id=2, q=0.3), MotorConfig(id=3, q=-0.2),
            MotorConfig(id=4, q=0.5), MotorConfig(id=5, q=0.5), MotorConfig(id=6, q=0.5),
        ]

        interval = 1.0 / frequency
        end_time = time.time() + duration_seconds

        print("\n" + "=" * 70)
        print(f"🔄 持续发送测试: {duration_seconds}秒 @ {frequency}Hz")
        print("=" * 70)

        try:
            while time.time() < end_time:
                # 更新位置（正弦波）
                current_time = time.time()
                for i, motor in enumerate(motors):
                    phase = i * (2 * math.pi / 6)
                    motor.q = 0.3 * math.sin(2 * math.pi * 0.5 * current_time + phase)
                    motor.dq = 0.3 * 2 * math.pi * 0.5 * math.cos(2 * math.pi * 0.5 * current_time + phase)

                message = self.build_message(motors)
                self.send_message(message)

                if self.message_count % 25 == 0:
                    port_map = self.get_port_mapping(motors)
                    port_str = ", ".join([f"800{k}={len(v)}" for k, v in port_map.items() if v])
                    print(f"[{self.message_count:4d}] {port_str}")

                time.sleep(interval)

            print(f"\n✓ 发送完成: 总计 {self.message_count} 条消息")

        except KeyboardInterrupt:
            print(f"\n⚠️  中断. 总计发送: {self.message_count}")

    def send_sine_wave(self, duration_seconds: int = 10):
        """发送正弦波测试数据（各电机相位不同）"""
        motors = [
            MotorConfig(id=1, q=0.0), MotorConfig(id=2, q=0.0), MotorConfig(id=3, q=0.0),
            MotorConfig(id=4, q=0.0), MotorConfig(id=5, q=0.0), MotorConfig(id=6, q=0.0),
        ]

        print("\n" + "=" * 70)
        print(f"〰️  正弦波测试: {duration_seconds}秒, 各电机相位不同")
        print("=" * 70)

        try:
            start_time = time.time()
            while time.time() - start_time < duration_seconds:
                current_time = time.time() - start_time

                # 为每个电机生成正弦波（相位差 60 度）
                for i, motor in enumerate(motors):
                    phase = i * (2 * math.pi / 6)  # 60度相位差
                    motor.q = 0.3 * math.sin(2 * math.pi * 0.5 * current_time + phase)
                    motor.dq = 0.3 * 2 * math.pi * 0.5 * math.cos(2 * math.pi * 0.5 * current_time + phase)
                    motor.kp = 20.0
                    motor.kd = 2.0

                message = self.build_message(motors)
                self.send_message(message)

                if self.message_count % 25 == 0:
                    pos_str = ", ".join([f"ID{m.id}={m.q:+.2f}" for m in motors])
                    print(f"[{self.message_count:4d}] {pos_str}")

                time.sleep(0.02)  # 50Hz

            print(f"\n✓ 发送完成: 总计 {self.message_count} 条消息")

        except KeyboardInterrupt:
            print(f"\n⚠️  中断. 总计发送: {self.message_count}")

    def close(self):
        if self.socket:
            self.socket.close()
            print("\n✓ UDP Socket 已关闭")


def main():
    parser = argparse.ArgumentParser(
        description="四端口电机测试发送器 - 测试端口 8000, 8001, 8002, 8003",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  # 测试所有四个端口
  python four_port_udp_sender.py --all

  # 测试单个端口 (0-3)
  python four_port_udp_sender.py --port 0
  python four_port_udp_sender.py --port 1

  # 持续发送 (50Hz, 10秒)
  python four_port_udp_sender.py --continuous 10

  # 正弦波测试
  python four_port_udp_sender.py --sine 10

端口映射:
  端口 8000 (CAN0): 电机 1, 2, 3
  端口 8001 (CAN1): 电机 4
  端口 8002 (CAN2): 电机 5
  端口 8003 (CAN3): 电机 6
        """
    )
    parser.add_argument("--target", default="127.0.0.1", help="目标 IP 地址")
    parser.add_argument("--port", type=int, choices=[0, 1, 2, 3],
                       help="测试单个端口 (0=8000, 1=8001, 2=8002, 3=8003)")
    parser.add_argument("--all", action="store_true", help="测试所有四个端口")
    parser.add_argument("--continuous", type=int, metavar="SECONDS",
                       help="持续发送 N 秒 (50Hz)")
    parser.add_argument("--sine", type=int, metavar="SECONDS",
                       help="正弦波测试 N 秒")

    args = parser.parse_args()

    tester = FourPortMotorTester(args.target)

    try:
        if args.all:
            tester.test_all_ports()
        elif args.port is not None:
            tester.test_single_port(args.port)
        elif args.continuous:
            tester.send_continuous(args.continuous)
        elif args.sine:
            tester.send_sine_wave(args.sine)
        else:
            # 默认：测试所有端口
            print("未指定模式，默认测试所有端口...\n")
            tester.test_all_ports()

    finally:
        tester.close()


if __name__ == "__main__":
    main()
