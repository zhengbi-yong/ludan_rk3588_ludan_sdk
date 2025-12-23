#!/usr/bin/env python3
"""
测试脚本：直接发送使能命令到192.168.1.5:8002
绕过ROS2，直接测试周立功设备连接
"""

import socket
import time
import struct

def send_enable_command(motor_id, command_value):
    """直接发送使能命令到周立功设备"""

    # 映射motor_id到CAN ID
    can_id = motor_id & 0xFF

    # 映射command值到实际的CAN frame值
    # 1 -> 0xFC (使能)
    # 0 -> 0xFD (禁使能)
    # 其他 -> 保持原值
    if command_value == 1:
        command_byte = 0xFC
        cmd_name = "使能"
    elif command_value == 0:
        command_byte = 0xFD
        cmd_name = "禁使能"
    else:
        command_byte = command_value & 0xFF
        cmd_name = f"自定义(0x{command_byte:02X})"

    # 创建CAN frame包
    # 格式: [CAN_ID(1 byte) | DLC(1 byte) | DATA(8 bytes)]
    packet = struct.pack(
        'BB8B',
        can_id,                    # CAN ID
        8,                         # DLC
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command_byte  # DATA
    )

    print("=" * 60)
    print(f"发送使能命令到 192.168.1.5:8002")
    print("=" * 60)
    print(f"Motor ID : {motor_id}")
    print(f"CAN ID   : 0x{can_id:02X}")
    print(f"命令     : {cmd_name}")
    print(f"数据长度 : {len(packet)} 字节")
    print(f"Hex数据  : {' '.join([f'{b:02X}' for b in packet])}")
    print("=" * 60)

    try:
        # 创建TCP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)

        # 连接到周立功设备
        print("正在连接到 192.168.1.5:8002...")
        sock.connect(('192.168.1.5', 8002))
        print("✓ 连接成功")

        # 发送数据
        print("正在发送数据...")
        sock.sendall(packet)
        print(f"✓ 已发送 {len(packet)} 字节")

        # 等待一下
        time.sleep(0.1)

        # 关闭连接
        sock.close()
        print("✓ 连接已关闭")

        print("\n✅ 使能命令发送成功！")
        return True

    except socket.timeout:
        print("❌ 连接超时")
        return False
    except ConnectionRefusedError:
        print("❌ 连接被拒绝")
        return False
    except Exception as e:
        print(f"❌ 错误: {e}")
        return False

def send_continuous_test(motor_id, count=5):
    """连续发送多个测试包"""
    print(f"\n连续发送 {count} 个测试包...")

    for i in range(count):
        print(f"\n--- 测试包 {i+1}/{count} ---")
        send_enable_command(motor_id, 1)
        time.sleep(0.5)

if __name__ == '__main__':
    import sys

    print("""
    周立功设备使能命令测试工具
    ===========================

    此工具直接连接到192.168.1.5:8002并发送使能命令
    用于测试周立功设备是否正常响应
    """)

    if len(sys.argv) < 2:
        print("使用方法:")
        print("  python3 test_send_enable.py <motor_id> [command]")
        print("")
        print("示例:")
        print("  python3 test_send_enable.py 5 1     # 使能电机5")
        print("  python3 test_send_enable.py 5 0     # 禁使能电机5")
        print("  python3 test_send_enable.py 5 1 10  # 连续发送10次使能命令")
        print("")
        sys.exit(1)

    motor_id = int(sys.argv[1])
    command = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    count = int(sys.argv[3]) if len(sys.argv) > 3 else 1

    if count > 1:
        send_continuous_test(motor_id, count)
    else:
        send_enable_command(motor_id, command)
