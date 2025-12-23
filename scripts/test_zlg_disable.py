#!/usr/bin/env python3
"""
使用 ZLG 网络包格式发送失能命令进行测试

端口映射：
- 8000 = CAN0
- 8001 = CAN1
- 8002 = CAN2
- 8003 = CAN3
"""

import socket
import struct
import sys

# 端口到 CAN 通道的映射
PORT_TO_CHANNEL = {
    8000: 0,  # CAN0
    8001: 1,  # CAN1
    8002: 2,  # CAN2
    8003: 3,  # CAN3
}

def get_channel_from_port(port):
    """根据端口号获取 CAN 通道"""
    if port in PORT_TO_CHANNEL:
        return PORT_TO_CHANNEL[port]
    return None

def build_zlg_packet(can_id, data, channel=0):
    """
    构建 ZLG CAN 网络包
    格式: 55 [type(3)] [length(2)] [timestamp(8)] [can_id(4)] [frame_info(2)] [channel(1)] [dlc(1)] [data(8)] [checksum(1)]
    """
    # 固定数据长度
    DATA_LENGTH = 0x0018  # 24 字节

    packet = bytearray()

    # 1. Header: 0x55
    packet.append(0x55)

    # 2. Type: 0x00 0x00 0x00 (CAN报文类型，固定00)
    packet.extend([0x00, 0x00, 0x00])

    # 3. Data length: 大端序
    packet.extend([(DATA_LENGTH >> 8) & 0xFF, DATA_LENGTH & 0xFF])

    # 4. Timestamp: 8字节全0
    packet.extend([0x00] * 8)

    # 5. CAN ID: 4字节大端序
    packet.extend([(can_id >> 24) & 0xFF, (can_id >> 16) & 0xFF, (can_id >> 8) & 0xFF, can_id & 0xFF])

    # 6. Frame info: 0x00 0x00
    packet.extend([0x00, 0x00])

    # 7. Channel
    packet.append(channel)

    # 8. DLC
    dlc = len(data)
    if dlc > 8:
        dlc = 8
    packet.append(dlc)

    # 9. Data: 固定8字节
    for i in range(8):
        if i < len(data):
            packet.append(data[i])
        else:
            packet.append(0x00)

    # 10. XOR Checksum: 字节1到字节29的异或
    checksum = 0
    for i in range(1, len(packet)):
        checksum ^= packet[i]
    packet.append(checksum)

    return bytes(packet)

def send_disable_command(motor_id, port=8000):
    """发送失能命令"""

    channel = get_channel_from_port(port)
    if channel is None:
        print(f"❌ 错误: 端口 {port} 不在有效范围内 (8000-8003)")
        return False

    can_id = motor_id
    # 失能帧: FF FF FF FF FF FF FF FD
    disable_data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]

    zlg_packet = build_zlg_packet(can_id, disable_data, channel)

    print("=" * 70)
    print(f"发送 ZLG 失能命令到 192.168.1.5:{port}")
    print("=" * 70)
    print(f"Motor ID : {motor_id}")
    print(f"CAN ID   : 0x{can_id:02X}")
    print(f"Channel  : {channel} (CAN{channel})")
    print(f"Port     : {port}")
    print(f"数据长度 : {len(zlg_packet)} 字节")
    print(f"Hex数据  : {' '.join([f'{b:02X}' for b in zlg_packet])}")
    print("=" * 70)

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)

        print(f"正在连接到 192.168.1.5:{port}...")
        sock.connect(('192.168.1.5', port))
        print("✓ 连接成功")

        print("正在发送数据...")
        sock.sendall(zlg_packet)
        print(f"✓ 已发送 {len(zlg_packet)} 字节")

        sock.close()
        print("✓ 连接已关闭")
        print("\n✅ 失能命令发送成功！")
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

def send_enable_command(motor_id, port=8000):
    """发送使能命令"""

    channel = get_channel_from_port(port)
    if channel is None:
        print(f"❌ 错误: 端口 {port} 不在有效范围内 (8000-8003)")
        return False

    can_id = motor_id
    # 使能帧: FF FF FF FF FF FF FF FC
    enable_data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]

    zlg_packet = build_zlg_packet(can_id, enable_data, channel)

    print("=" * 70)
    print(f"发送 ZLG 使能命令到 192.168.1.5:{port}")
    print("=" * 70)
    print(f"Motor ID : {motor_id}")
    print(f"CAN ID   : 0x{can_id:02X}")
    print(f"Channel  : {channel} (CAN{channel})")
    print(f"Port     : {port}")
    print(f"数据长度 : {len(zlg_packet)} 字节")
    print(f"Hex数据  : {' '.join([f'{b:02X}' for b in zlg_packet])}")
    print("=" * 70)

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)

        print(f"正在连接到 192.168.1.5:{port}...")
        sock.connect(('192.168.1.5', port))
        print("✓ 连接成功")

        print("正在发送数据...")
        sock.sendall(zlg_packet)
        print(f"✓ 已发送 {len(zlg_packet)} 字节")

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

def test_all_channels(motor_id, enable=False):
    """测试所有 CAN 通道"""
    print("\n" + "=" * 70)
    cmd_name = "使能" if enable else "失能"
    print(f"测试所有 CAN 通道 - {cmd_name}命令")
    print("=" * 70)

    for port in [8000, 8001, 8002, 8003]:
        channel = PORT_TO_CHANNEL[port]
        print(f"\n--- 测试 CAN{channel} (端口 {port}) ---")
        if enable:
            send_enable_command(motor_id, port)
        else:
            send_disable_command(motor_id, port)
        print()

if __name__ == '__main__':
    print("""
    ZLG 网络包格式电机命令测试工具
    ================================

    端口映射：
    - 8000 = CAN0
    - 8001 = CAN1
    - 8002 = CAN2
    - 8003 = CAN3
    """)

    if len(sys.argv) < 2:
        print("使用方法:")
        print("  python3 test_zlg_disable.py <motor_id> [port] [enable]")
        print("")
        print("示例:")
        print("  python3 test_zlg_disable.py 1          # 失能电机1，使用CAN0(端口8000)")
        print("  python3 test_zlg_disable.py 1 8003     # 失能电机1，使用CAN3(端口8003)")
        print("  python3 test_zlg_disable.py 1 8002     # 失能电机1，使用CAN2(端口8002)")
        print("  python3 test_zlg_disable.py 1 all      # 测试所有CAN通道的失能")
        print("  python3 test_zlg_disable.py 1 all e    # 测试所有CAN通道的使能")
        print("")
        sys.exit(1)

    motor_id = int(sys.argv[1])

    if len(sys.argv) >= 3 and sys.argv[2] == "all":
        enable = len(sys.argv) >= 4 and sys.argv[3] == "e"
        test_all_channels(motor_id, enable)
    else:
        port = int(sys.argv[2]) if len(sys.argv) > 2 else 8000
        enable = len(sys.argv) >= 4 and sys.argv[3] == "e"
        if enable:
            send_enable_command(motor_id, port)
        else:
            send_disable_command(motor_id, port)
