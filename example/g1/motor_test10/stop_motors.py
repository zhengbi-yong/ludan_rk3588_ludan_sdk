#!/usr/bin/env python3
"""
停止电机控制 - 发送 mode=0 的消息来取消活跃状态
"""

import socket
import json
import time

def send_stop_command(target_ip="127.0.0.1", target_port=8888):
    """发送停止命令（所有电机 mode=0）"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 构建 mode=0 的消息
    motor_cmd = {}
    for i in range(1, 7):  # 电机 1-6
        motor_cmd[str(i)] = {
            "q": 0.0,
            "dq": 0.0,
            "tau": 0.0,
            "mode": 0,  # ← 关键：mode=0 表示非活跃
            "kp": 0.0,
            "kd": 0.0
        }

    message = {
        "timestamp": time.time(),
        "sequence": 0,
        "motor_cmd": motor_cmd,
        "mode_pr": 0,
        "mode_machine": 1
    }

    # 发送 3 次（确保收到）
    for i in range(3):
        json_data = json.dumps(message, ensure_ascii=False)
        data_bytes = json_data.encode('utf-8')
        sock.sendto(data_bytes, (target_ip, target_port))
        print(f"✓ 发送停止命令 #{i+1}")
        time.sleep(0.1)

    sock.close()
    print("\n✓ 所有电机已设置为非活跃状态")
    print("✓ Controller 将停止发送这些电机的数据")

if __name__ == "__main__":
    print("=" * 60)
    print("  停止电机控制 - 取消活跃状态")
    print("=" * 60)
    print("\n发送 mode=0 的消息，Controller 将停止发送数据\n")
    send_stop_command()
    print("\n" + "=" * 60)
