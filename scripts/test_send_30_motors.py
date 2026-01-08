#!/usr/bin/env python3
"""
测试脚本：发送30个电机的数据到UDP 8888端口
用于验证listener_8888_to_ros2.py是否正确处理motor_id 1-30
"""

import socket
import json
import time

def send_test_data():
    # UDP配置
    UDP_IP = "127.0.0.1"  # 本地测试
    UDP_PORT = 8888

    # 创建UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 创建30个电机的测试数据 (motor_id: 1-30)
    motor_cmd = {}

    for motor_id in range(1, 31):
        if motor_id in [12, 21, 30]:  # 只给3个电机设置非零值
            motor_cmd[str(motor_id)] = {
                'mode': 1,
                'q': float(motor_id) * 0.1,  # 1.2, 2.1, 3.0
                'dq': 0.5,
                'tau': 0.1,
                'kp': 20.0,
                'kd': 2.0
            }
        else:
            motor_cmd[str(motor_id)] = {
                'mode': 0,
                'q': 0.0,
                'dq': 0.0,
                'tau': 0.0,
                'kp': 0.0,
                'kd': 0.0
            }

    # 创建消息
    message = {
        'timestamp': time.time(),
        'sequence': 1,
        'mode_pr': 0,
        'mode_machine': 1,
        'motor_cmd': motor_cmd
    }

    # 转换为JSON
    json_str = json.dumps(message)
    json_bytes = json_str.encode('utf-8')

    print("=" * 60)
    print("🧪 测试发送30个电机数据到UDP 8888端口")
    print("=" * 60)
    print(f"目标: {UDP_IP}:{UDP_PORT}")
    print(f"数据大小: {len(json_bytes)} 字节")
    print(f"活跃电机: 12 (q=1.2), 21 (q=2.1), 30 (q=3.0)")
    print("")

    # 发送数据
    for i in range(5):  # 发送5次
        sock.sendto(json_bytes, (UDP_IP, UDP_PORT))
        print(f"✅ 发送第 {i+1} 条消息")
        time.sleep(0.1)

    print("")
    print("✅ 测试完成！请检查listener日志和/lowcmd话题")
    print("   预期结果: motor_cmd中id应该是1-30，其中id=30的q=3.0")

    sock.close()

if __name__ == "__main__":
    send_test_data()
