#!/usr/bin/env python3
"""
DM 电机标零脚本
直接通过 ZLG CAN 发送标零命令: FF FF FF FF FF FF FF FE
"""

import sys
import os

# 添加 ZLG CAN SDK 路径
sys.path.append('/home/linaro/ludan_sdk/LINUX_CANFDNET_V1.3.0.5_example/src')

try:
    from zlgcan import *
except ImportError:
    print("错误: 无法导入 zlgcan 模块")
    print("请确保 ZLG CAN SDK 已正确安装")
    sys.exit(1)

# 电机到端口的映射 (motor_id 1-30)
PORT_CONFIGS = [
    (8000, 8, 0, 0),   # Motors 1-8   -> CAN 0
    (8001, 8, 8, 1),   # Motors 9-16  -> CAN 0
    (8002, 8, 16, 2),  # Motors 17-24 -> CAN 0
    (8003, 6, 24, 3)   # Motors 25-30 -> CAN 0
]

def get_port_config(motor_id):
    """根据电机ID获取端口配置"""
    for port, count, offset, channel in PORT_CONFIGS:
        if offset < motor_id <= offset + count:
            local_can_id = motor_id - offset
            return port, channel, local_can_id
    return None, None, None

def send_zero_command(motor_ids, zlg_ip="192.168.1.200"):
    """
    发送标零命令到指定电机

    Args:
        motor_ids: 电机ID列表
        zlg_ip: ZLG CAN 卡 IP 地址
    """
    # 打开设备
    device_handle = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, 0, 0)
    if device_handle == INVALID_DEVICE_HANDLE:
        print(f"错误: 无法打开 ZCAN 设备")
        return False

    print(f"已打开 ZCAN 设备, 连接到 {zlg_ip}")

    # 为每个端口初始化 CAN 通道
    port_handles = {}
    for port, channel, _ in set([(p, c, _) for p, c, _ in [get_port_config(mid) for mid in motor_ids]]):
        if (port, channel) not in port_handles:
            # 初始化 CAN 通道
            init_config = ZCAN_CHANNEL_INIT_CONFIG()
            init_config.can_type = TYPE_CANFD
            init_config.canfd.acc_code = 0
            init_config.canfd.acc_mask = 0
            init_config.canfd.abit_timing = 1000000  # 1 Mbps
            init_config.canfd.dbit_timing = 5000000   # 5 Mbps
            init_config.canfd.brp = 0
            init_config.canfd.filter = 0
            init_config.canfd.mode = 0

            channel_handle = ZCAN_InitCAN(device_handle, channel, init_config)
            if channel_handle == INVALID_CHANNEL_HANDLE:
                print(f"错误: 无法初始化 CAN 通道 {channel}")
                ZCAN_CloseDevice(device_handle)
                return False

            # 设置目标 IP 和端口
            ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, channel, CMD_DESIP, zlg_ip)
            port_val = port
            ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, channel, CMD_DESPORT, port_val)

            # 启动 CAN
            if ZCAN_StartCAN(channel_handle) == STATUS_ERR:
                print(f"错误: 无法启动 CAN 通道 {channel}")
                ZCAN_CloseDevice(device_handle)
                return False

            port_handles[(port, channel)] = channel_handle
            print(f"已初始化端口 {port}, 通道 {channel}")

    # 发送标零命令
    success_count = 0
    for motor_id in motor_ids:
        port, channel, local_can_id = get_port_config(motor_id)
        if port is None:
            print(f"警告: 无效的电机 ID {motor_id}")
            continue

        channel_handle = port_handles[(port, channel)]

        # 构造标零命令数据帧: FF FF FF FF FF FF FF FE
        transmit_data = ZCAN_Transmit_Data()
        transmit_data.frame.can_id = MAKE_CAN_ID(local_can_id, 0, 0, 0)
        transmit_data.frame.can_dlc = 8
        transmit_data.frame.data[0] = 0xFF
        transmit_data.frame.data[1] = 0xFF
        transmit_data.frame.data[2] = 0xFF
        transmit_data.frame.data[3] = 0xFF
        transmit_data.frame.data[4] = 0xFF
        transmit_data.frame.data[5] = 0xFF
        transmit_data.frame.data[6] = 0xFF
        transmit_data.frame.data[7] = 0xFE  # 标零命令标识
        transmit_data.transmit_type = 0

        # 发送
        ret = ZCAN_Transmit(channel_handle, transmit_data, 1)
        if ret == 1:
            print(f"✓ 已发送标零命令到电机 {motor_id} (端口 {port}, CAN ID {local_can_id})")
            success_count += 1
        else:
            print(f"✗ 发送失败: 电机 {motor_id}")

    # 清理资源
    for channel_handle in port_handles.values():
        ZCAN_ResetCAN(channel_handle)
    ZCAN_CloseDevice(device_handle)

    print(f"\n完成: 成功发送 {success_count}/{len(motor_ids)} 个标零命令")
    return success_count == len(motor_ids)

def main():
    if len(sys.argv) < 2:
        print("DM 电机标零脚本")
        print("\n用法:")
        print("  python3 send_motor_zero.py <motor_id> [<motor_id> ...]")
        print("\n示例:")
        print("  python3 send_motor_zero.py 1           # 标零电机 1")
        print("  python3 send_motor_zero.py 1 2 3       # 标零电机 1,2,3")
        print("  python3 send_motor_zero.py 1-30        # 标零电机 1-30")
        print("\n选项:")
        print("  --ip <address>    ZLG CAN 卡 IP 地址 (默认: 192.168.1.200)")
        sys.exit(1)

    # 解析参数
    motor_ids = []
    zlg_ip = "192.168.1.200"

    i = 1
    while i < len(sys.argv):
        arg = sys.argv[i]

        if arg == '--ip':
            if i + 1 < len(sys.argv):
                zlg_ip = sys.argv[i + 1]
                i += 2
                continue
            else:
                print("错误: --ip 需要指定 IP 地址")
                sys.exit(1)

        if '-' in arg:
            # 范围模式，如 "1-10"
            try:
                start, end = map(int, arg.split('-'))
                motor_ids.extend(range(start, end + 1))
            except ValueError:
                print(f"错误: 无效的范围格式 '{arg}'")
                sys.exit(1)
        else:
            # 单个ID
            try:
                motor_ids.append(int(arg))
            except ValueError:
                print(f"错误: 无效的电机 ID '{arg}'")
                sys.exit(1)
        i += 1

    if not motor_ids:
        print("错误: 没有指定电机 ID")
        sys.exit(1)

    # 发送标零命令
    print(f"\n准备发送标零命令到 {len(motor_ids)} 个电机...")
    print(f"电机 ID: {sorted(set(motor_ids))}")
    print(f"ZLG IP: {zlg_ip}\n")

    success = send_zero_command(motor_ids, zlg_ip)
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
