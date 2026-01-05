#!/usr/bin/env python3
"""
UDP电机控制数据包发送器
模仿 Jetson 发送 LowCmd 数据到 UDP 8888 端口
"""

import socket
import json
import time
import math
import argparse

class UDPMotorTestSender:
    def __init__(self, target_ip="127.0.0.1", target_port=8888):
        self.target_ip = target_ip
        self.target_port = target_port
        self.socket = None
        self.message_count = 0

        # 创建 UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"UDP Socket created: {self.target_ip}:{self.target_port}")

    def build_message(self, motor_commands):
        """
        构建 xixilowcmd 格式的 JSON 消息

        motor_commands: 字典列表，格式:
            [
                {"id": 4, "motor_id": 13, "q": 1.0, "dq": 0.0, "tau": 0.0, "kp": 1.0, "kd": 1.0},
                ...
            ]
        """
        motor_cmd_dict = {}

        for cmd in motor_commands:
            motor_key = str(cmd["id"])
            motor_cmd_dict[motor_key] = {
                "q": cmd["q"],
                "dq": cmd["dq"],
                "tau": cmd["tau"],
                "mode": 1,  # 使能模式
                "kp": cmd["kp"],
                "kd": cmd["kd"]
            }

        message = {
            "timestamp": time.time(),
            "sequence": self.message_count,
            "motor_cmd": motor_cmd_dict,
            "mode_pr": 0,
            "mode_machine": 1
        }

        return message

    def send_message(self, message):
        """发送单条消息"""
        try:
            json_data = json.dumps(message, ensure_ascii=False)
            data_bytes = json_data.encode('utf-8')

            self.socket.sendto(data_bytes, (self.target_ip, self.target_port))
            self.message_count += 1

            return True
        except Exception as e:
            print(f"Send failed: {e}")
            return False

    def send_single_test(self):
        """
        发送单次测试数据包
        电机配置:
            - ID 4:  motor_id=13, q=1, dq=0, tau=0, kp=1, kd=1
            - ID 5:  motor_id=22, q=1, dq=0, tau=0, kp=1, kd=1
            - ID 6:  motor_id=30, q=1, dq=0, tau=0, kp=1, kd=1
        """
        motor_commands = [
            {"id": 4,  "motor_id": 13, "q": 1.0, "dq": 0.0, "tau": 0.0, "kp": 1.0, "kd": 1.0},
            {"id": 5,  "motor_id": 22, "q": 1.0, "dq": 0.0, "tau": 0.0, "kp": 1.0, "kd": 1.0},
            {"id": 6,  "motor_id": 30, "q": 1.0, "dq": 0.0, "tau": 0.0, "kp": 1.0, "kd": 1.0},
        ]

        message = self.build_message(motor_commands)

        print("\n" + "="*60)
        print("Sending UDP Test Packet")
        print("="*60)
        print(f"Target: {self.target_ip}:{self.target_port}")
        print(f"Timestamp: {message['timestamp']:.3f}")
        print(f"Sequence: {message['sequence']}")
        print("\nMotor Commands:")
        for cmd in motor_commands:
            print(f"  Motor ID {cmd['id']:2d}: q={cmd['q']:.1f}, dq={cmd['dq']:.1f}, "
                  f"tau={cmd['tau']:.1f}, kp={cmd['kp']:.1f}, kd={cmd['kd']:.1f}")
        print("\nJSON Message:")
        print(json.dumps(message, indent=2, ensure_ascii=False))
        print("="*60)

        self.send_message(message)
        print(f"\nMessage sent successfully (count: {self.message_count})")

    def send_continuous(self, duration_seconds=10, frequency=50):
        """
        持续发送测试数据包

        Args:
            duration_seconds: 发送持续时间（秒）
            frequency: 发送频率（Hz）
        """
        motor_commands = [
            {"id": 4,  "motor_id": 13, "q": 1.0, "dq": 0.0, "tau": 0.0, "kp": 1.0, "kd": 1.0},
            {"id": 5,  "motor_id": 22, "q": 1.0, "dq": 0.0, "tau": 0.0, "kp": 1.0, "kd": 1.0},
            {"id": 6,  "motor_id": 30, "q": 1.0, "dq": 0.0, "tau": 0.0, "kp": 1.0, "kd": 1.0},
        ]

        interval = 1.0 / frequency
        end_time = time.time() + duration_seconds

        print("\n" + "="*60)
        print(f"Continuous UDP Test: {duration_seconds}s @ {frequency}Hz")
        print("="*60)
        print(f"Target: {self.target_ip}:{self.target_port}")
        print(f"Motor Commands: 3 motors (ID 4, 5, 6)")
        print("="*60 + "\n")

        try:
            while time.time() < end_time:
                message = self.build_message(motor_commands)
                self.send_message(message)

                if self.message_count % 10 == 0:
                    print(f"[{self.message_count:4d}] Sent at {time.time():.3f}")

                time.sleep(interval)

            print(f"\nTotal messages sent: {self.message_count}")
            print(f"Actual frequency: {self.message_count / duration_seconds:.1f} Hz")

        except KeyboardInterrupt:
            print(f"\nInterrupted. Total sent: {self.message_count}")

    def send_sine_wave(self, duration_seconds=10, frequency=0.5, amplitude=0.3):
        """
        发送正弦波测试数据包（位置随时间变化）

        Args:
            duration_seconds: 发送持续时间（秒）
            frequency: 正弦波频率（Hz）
            amplitude: 正弦波幅值（弧度）
        """
        interval = 0.02  # 50Hz
        end_time = time.time() + duration_seconds

        print("\n" + "="*60)
        print(f"Sine Wave UDP Test: {duration_seconds}s @ {frequency}Hz, {amplitude}rad")
        print("="*60)
        print(f"Target: {self.target_ip}:{self.target_port}")
        print("="*60 + "\n")

        try:
            while time.time() < end_time:
                current_time = time.time()

                # 为3个电机生成正弦波位置命令
                motor_commands = []
                for idx, (motor_id, motor_id_value) in enumerate([(4, 13), (5, 22), (6, 30)]):
                    phase_offset = idx * (2 * math.pi / 3)  # 120度相位差
                    phase = 2 * math.pi * frequency * current_time + phase_offset
                    pos = amplitude * math.sin(phase)
                    vel = amplitude * 2 * math.pi * frequency * math.cos(phase)

                    motor_commands.append({
                        "id": motor_id,
                        "motor_id": motor_id_value,
                        "q": pos,
                        "dq": vel,
                        "tau": 0.0,
                        "kp": 20.0,
                        "kd": 2.0
                    })

                message = self.build_message(motor_commands)
                self.send_message(message)

                if self.message_count % 25 == 0:
                    pos_str = ", ".join([f"ID{m['id']}={m['q']:+.3f}" for m in motor_commands])
                    print(f"[{self.message_count:4d}] {pos_str}")

                time.sleep(interval)

            print(f"\nTotal messages sent: {self.message_count}")

        except KeyboardInterrupt:
            print(f"\nInterrupted. Total sent: {self.message_count}")

    def close(self):
        if self.socket:
            self.socket.close()
            print("\nUDP Socket closed")


def main():
    parser = argparse.ArgumentParser(description="UDP Motor Test Sender")
    parser.add_argument("--target", default="127.0.0.1", help="Target IP address")
    parser.add_argument("--port", type=int, default=8888, help="Target UDP port")
    parser.add_argument("--single", action="store_true", help="Send single test packet")
    parser.add_argument("--continuous", type=int, metavar="SECONDS", help="Send continuously for N seconds")
    parser.add_argument("--sine", type=int, metavar="SECONDS", help="Send sine wave test for N seconds")

    args = parser.parse_args()

    sender = UDPMotorTestSender(args.target, args.port)

    try:
        if args.single:
            # 发送单次测试
            sender.send_single_test()

        elif args.continuous:
            # 持续发送
            sender.send_continuous(duration_seconds=args.continuous, frequency=50)

        elif args.sine:
            # 正弦波测试
            sender.send_sine_wave(duration_seconds=args.sine)

        else:
            # 默认：发送单次测试
            print("No mode specified. Use --single, --continuous, or --sine")
            print("\nExample usage:")
            print("  Single packet:  python udp_test_sender.py --single")
            print("  Continuous:     python udp_test_sender.py --continuous 10")
            print("  Sine wave:      python udp_test_sender.py --sine 10")
            print("\nSending single test packet by default...\n")
            sender.send_single_test()

    finally:
        sender.close()


if __name__ == "__main__":
    main()
