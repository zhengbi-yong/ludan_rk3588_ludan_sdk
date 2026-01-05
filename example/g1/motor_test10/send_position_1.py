#!/usr/bin/env python3
"""
发送位置=1, 速度=0, kp=1, kd=1, 力矩=0 的测试命令
"""

import socket
import json
import time
import argparse
from dataclasses import dataclass
from typing import List

@dataclass
class MotorConfig:
    """电机配置"""
    id: int           # 电机 ID
    q: float          # 位置 (弧度)
    dq: float         # 速度 (弧度/秒)
    tau: float        # 力矩 (N·m)
    kp: float         # 位置增益
    kd: float         # 速度增益
    mode: int = 1     # 模式 (1=活跃)


def create_motor_message(motor_configs: List[MotorConfig], sequence: int = 0) -> dict:
    """创建 ROS2 电机消息"""
    motor_cmd = {}

    for config in motor_configs:
        motor_cmd[str(config.id)] = {
            "q": config.q,
            "dq": config.dq,
            "tau": config.tau,
            "mode": config.mode,
            "kp": config.kp,
            "kd": config.kd
        }

    message = {
        "timestamp": time.time(),
        "sequence": sequence,
        "motor_cmd": motor_cmd,
        "mode_pr": 0,
        "mode_machine": 1
    }

    return message


def send_udp_message(message: dict, target_ip: str, target_port: int):
    """发送 UDP 消息"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    json_data = json.dumps(message, ensure_ascii=False)
    data_bytes = json_data.encode('utf-8')

    sock.sendto(data_bytes, (target_ip, target_port))
    sock.close()

    return len(data_bytes)


def main():
    parser = argparse.ArgumentParser(description='发送位置=1的测试命令')
    parser.add_argument('--target', default='127.0.0.1', help='目标 IP 地址')
    parser.add_argument('--port', type=int, default=8888, help='目标端口')
    parser.add_argument('--continuous', type=int, metavar='SECONDS',
                        help='持续发送模式 (指定秒数, 50Hz)')
    parser.add_argument('--once', action='store_true', help='只发送一次')

    args = parser.parse_args()

    # 电机配置：位置=1, 速度=0, kp=1, kd=1, 力矩=0
    motors = [
        MotorConfig(id=1, q=1.0, dq=0.0, tau=0.0, kp=1.0, kd=1.0),
        MotorConfig(id=2, q=1.0, dq=0.0, tau=0.0, kp=1.0, kd=1.0),
        MotorConfig(id=3, q=1.0, dq=0.0, tau=0.0, kp=1.0, kd=1.0),
        MotorConfig(id=4, q=1.0, dq=0.0, tau=0.0, kp=1.0, kd=1.0),
        MotorConfig(id=5, q=1.0, dq=0.0, tau=0.0, kp=1.0, kd=1.0),
        MotorConfig(id=6, q=1.0, dq=0.0, tau=0.0, kp=1.0, kd=1.0),
    ]

    print("=" * 60)
    print("  发送位置=1 测试命令")
    print("=" * 60)
    print(f"\n参数:")
    print(f"  位置 (q):    1.0 弧度")
    print(f"  速度 (dq):   0.0 弧度/秒")
    print(f"  力矩 (tau):  0.0 N·m")
    print(f"  位置增益 kp: 1.0")
    print(f"  速度增益 kd: 1.0")
    print(f"\n目标: {args.target}:{args.port}")
    print(f"电机: {', '.join(str(m.id) for m in motors)}")
    print()

    if args.once:
        # 单次发送
        msg = create_motor_message(motors, sequence=0)
        size = send_udp_message(msg, args.target, args.port)
        print(f"✓ 已发送 {size} 字节")
        print()

    elif args.continuous:
        # 持续发送
        duration = args.continuous
        frequency = 50  # 50Hz
        interval = 1.0 / frequency

        print(f"持续发送模式: {duration} 秒 @ {frequency}Hz")
        print(f"总计: {duration * frequency} 条消息")
        print()

        start_time = time.time()
        sequence = 0

        try:
            while time.time() - start_time < duration:
                msg = create_motor_message(motors, sequence=sequence)
                send_udp_message(msg, args.target, args.port)

                sequence += 1
                elapsed = time.time() - start_time
                remaining = duration - elapsed

                # 每 50 条消息显示一次进度
                if sequence % 50 == 0:
                    print(f"[{elapsed:.1f}s] 已发送 {sequence} 条消息 (剩余 {remaining:.1f}s)")

                time.sleep(max(0, interval - (time.time() - start_time - (sequence / frequency))))

            print(f"\n✓ 发送完成！总计 {sequence} 条消息")

        except KeyboardInterrupt:
            print(f"\n✗ 用户中断，已发送 {sequence} 条消息")

    else:
        # 默认：显示发送预览
        msg = create_motor_message(motors, sequence=0)
        print("消息预览:")
        print(json.dumps(msg, indent=2, ensure_ascii=False))
        print()
        print("使用 --once 发送一次，或 --continuous SECONDS 持续发送")


if __name__ == "__main__":
    main()
