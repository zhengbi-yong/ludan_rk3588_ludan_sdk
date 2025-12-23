#!/usr/bin/env python3
"""
监控发送到周立功设备(192.168.1.5:8003)的数据包
使用scapy捕获和解析TCP流量
支持 ZLG 网络包格式 (per zlg_desc.txt)
"""

import sys
import time
import struct
from datetime import datetime

# 尝试导入scapy
try:
    from scapy.all import sniff, TCP, Raw, IP
    HAS_SCAPY = True
except ImportError:
    HAS_SCAPY = False
    print("⚠️  scapy未安装，将使用tcpdump模式")
    print("   安装: sudo apt-get install python3-scapy")
    print("")


def calc_xor_checksum(data):
    """计算 XOR 校验"""
    checksum = 0
    for b in data:
        checksum ^= b
    return checksum


def parse_zlg_packet(data, show_errors=True):
    """
    解析 ZLG 网络包格式 (27 bytes)
    Format per zlg_desc.txt:
    55 | 00 00 00 | 18 00 | [8字节时间戳] | [4字节CAN_ID] | 00 00 | [通道] | [DLC] | [8字节数据] | [校验]

    容错解析：即使长度不是27，也尝试解析，暴露错误信息
    """
    result = {
        'valid': False,
        'errors': [],
        'warnings': [],
        'raw_length': len(data),
        'raw_hex': ' '.join([f'{b:02X}' for b in data])
    }

    # 检查包头
    if len(data) < 1:
        result['errors'].append("数据为空")
        return result

    if data[0] != 0x55:
        result['errors'].append(f"包头错误: 0x{data[0]:02X} (期望 0x55)")
        return result

    result['header'] = data[0]

    # 检查最小长度
    if len(data) < 27:
        result['errors'].append(f"长度不足: {len(data)} 字节 (期望 27 字节)")

    try:
        # 解析各个字段（尽可能解析）
        result['type'] = data[1:4] if len(data) >= 4 else b'\x00\x00\x00'

        if len(data) >= 6:
            result['data_length'] = struct.unpack('<H', data[4:6])[0]
        else:
            result['data_length'] = 0
            result['errors'].append("无法解析数据长度字段")

        if len(data) >= 14:
            result['timestamp'] = data[6:14]
        else:
            result['timestamp'] = b'\x00' * 8
            result['errors'].append(f"时间戳字段不完整 (仅有 {max(0, len(data)-6)} 字节)")

        if len(data) >= 18:
            result['can_id'] = struct.unpack('<I', data[14:18])[0]
        else:
            result['can_id'] = 0
            result['errors'].append(f"CAN ID字段不完整 (仅有 {max(0, len(data)-14)} 字节)")

        if len(data) >= 20:
            result['frame_info'] = data[18:20]
        else:
            result['frame_info'] = b'\x00\x00'
            result['errors'].append(f"帧信息字段不完整 (仅有 {max(0, len(data)-18)} 字节)")

        if len(data) >= 21:
            result['channel'] = data[20]
        else:
            result['channel'] = 0
            result['errors'].append("通道字段缺失")

        if len(data) >= 22:
            result['dlc'] = data[21]
        else:
            result['dlc'] = 0
            result['errors'].append("DLC字段缺失")

        # 数据字段（8字节）
        if len(data) >= 30:
            result['data'] = data[22:30]
        elif len(data) >= 22:
            result['data'] = data[22:] + b'\x00' * (8 - (len(data) - 22))
            result['warnings'].append(f"数据字段不完整 (仅有 {len(data)-22} 字节)")
        else:
            result['data'] = b'\x00' * 8
            result['errors'].append(f"数据字段缺失 (仅有 {max(0, len(data)-22)} 字节)")

        # 校验和
        if len(data) >= 27:
            result['checksum'] = data[26]
            # 计算校验和
            calc_checksum = calc_xor_checksum(data[1:26])
            result['checksum_valid'] = (result['checksum'] == calc_checksum)
            if not result['checksum_valid']:
                result['errors'].append(f"校验和错误: 接收=0x{result['checksum']:02X}, 计算=0x{calc_checksum:02X}")
        else:
            result['checksum'] = 0
            result['checksum_valid'] = False
            result['errors'].append("校验和字段缺失")

        # 判断命令类型
        can_data = result.get('data', b'\x00' * 8)
        if can_data == bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]):
            result['cmd_type'] = '使能'
            result['command'] = 'ENABLE'
        elif can_data == bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]):
            result['cmd_type'] = '禁使能'
            result['command'] = 'DISABLE'
        elif len(can_data) >= 7 and all(b == 0xFF for b in can_data[:7]):
            result['cmd_type'] = '自定义命令'
            result['command'] = f'CUSTOM(0x{can_data[7]:02X})'
        else:
            result['cmd_type'] = '数据帧'
            result['command'] = 'DATA'

        # 判断整体是否有效
        result['valid'] = (len(result['errors']) == 0)

    except Exception as e:
        result['errors'].append(f"解析异常: {str(e)}")

    return result


def parse_old_can_frame(data):
    """解析旧的简单 CAN frame 格式 (10 bytes) - 兼容旧格式"""
    if len(data) < 10:
        return None

    try:
        can_id = data[0]
        dlc = data[1]
        frame_data = data[2:10]

        result = {
            'format': 'OLD',
            'can_id': can_id,
            'dlc': dlc,
            'data': frame_data,
            'raw_hex': ' '.join([f'{b:02X}' for b in data[:10]])
        }

        if frame_data == bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]):
            result['cmd_type'] = '使能'
            result['command'] = 'ENABLE'
        elif frame_data == bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]):
            result['cmd_type'] = '禁使能'
            result['command'] = 'DISABLE'
        else:
            result['cmd_type'] = '数据帧'
            result['command'] = 'DATA'

        return result
    except Exception as e:
        return None


def packet_callback(packet):
    """处理捕获的数据包"""
    if not packet.haslayer(TCP):
        return

    tcp = packet[TCP]
    ip = packet[IP]

    # 只检查发送到192.168.1.5:8003的包
    if ip.dst != '192.168.1.5' or tcp.dport != 8003:
        return

    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

    # 检查是否有payload
    if tcp.haslayer(Raw):
        payload = tcp[Raw].load
        data = bytes(payload)

        print(f"\n{'='*70}")
        print(f"⏰ {timestamp} → {ip.src}:{tcp.sport} → {ip.dst}:{tcp.dport}")
        print(f"📦 原始数据长度: {len(data)} 字节")
        print(f"{'='*70}")

        # 打印原始 hex（方便调试）
        raw_hex_preview = ' '.join([f'{b:02X}' for b in data[:60]])
        if len(data) > 60:
            raw_hex_preview += ' ...'
        print(f"  原始Hex: {raw_hex_preview}")

        # 尝试解析为 ZLG 网络包格式 (包头 0x55)
        if len(data) >= 1 and data[0] == 0x55:
            zlg_info = parse_zlg_packet(data)

            # 显示解析结果
            status_icon = "✓" if zlg_info['valid'] else "⚠️"
            print(f"\n  📋 ZLG 网络包解析 {status_icon}")

            if zlg_info['valid']:
                # 正常解析，显示精简信息
                print(f"    CAN ID     : 0x{zlg_info['can_id']:08X} (电机 {zlg_info['can_id'] & 0xFF})")
                print(f"    通道       : CAN{zlg_info['channel']}")
                print(f"    DLC        : {zlg_info['dlc']}")
                print(f"    数据       : {' '.join([f'{b:02X}' for b in zlg_info['data']])}")
                print(f"    命令类型   : {zlg_info['cmd_type']}")
                print(f"    校验和     : 0x{zlg_info['checksum']:02X} ✓")
            else:
                # 有错误，显示详细信息
                print(f"    长度       : {zlg_info['raw_length']} 字节 (期望 27 字节)")
                print(f"    包头       : 0x{zlg_info.get('header', 0):02X}")

                if 'can_id' in zlg_info:
                    print(f"    CAN ID     : 0x{zlg_info['can_id']:08X}")
                if 'channel' in zlg_info:
                    print(f"    通道       : CAN{zlg_info['channel']}")
                if 'dlc' in zlg_info:
                    print(f"    DLC        : {zlg_info['dlc']}")
                if 'data' in zlg_info:
                    print(f"    数据       : {' '.join([f'{b:02X}' for b in zlg_info['data']])}")
                if 'cmd_type' in zlg_info:
                    print(f"    命令类型   : {zlg_info['cmd_type']}")
                if 'checksum' in zlg_info:
                    checksum_status = "✓" if zlg_info.get('checksum_valid', False) else "✗"
                    print(f"    校验和     : 0x{zlg_info['checksum']:02X} {checksum_status}")

                # 显示错误
                if zlg_info['errors']:
                    print(f"\n    ❌ 错误:")
                    for err in zlg_info['errors']:
                        print(f"       - {err}")

                # 显示警告
                if zlg_info['warnings']:
                    print(f"\n    ⚠️  警告:")
                    for warn in zlg_info['warnings']:
                        print(f"       - {warn}")

        # 尝试解析旧的简单格式 (10 bytes)
        elif len(data) >= 10:
            old_info = parse_old_can_frame(data)
            if old_info:
                print(f"\n  📋 旧格式 CAN Frame:")
                print(f"    CAN ID     : 0x{old_info['can_id']:02X} (电机 {old_info['can_id']})")
                print(f"    DLC        : {old_info['dlc']}")
                print(f"    数据       : {' '.join([f'{b:02X}' for b in old_info['data']])}")
                print(f"    命令类型   : {old_info['cmd_type']}")
                print(f"    完整Hex    : {old_info['raw_hex']}")
            else:
                print(f"\n  📋 无法识别的数据格式")
                print(f"    原始Hex: {' '.join([f'{b:02X}' for b in data])}")
        else:
            print(f"\n  📋 数据太短，无法解析")
            print(f"    原始Hex: {' '.join([f'{b:02X}' for b in data])}")

        # 如果有多个包拼接在一起，继续解析
        if len(data) > 27:
            print(f"\n  🔍 检测到可能的多包数据，尝试解析...")
            offset = 0
            packet_num = 1
            while offset < len(data):
                if offset < len(data) and data[offset] == 0x55:
                    end_pos = min(offset + 27, len(data))
                    zlg_info = parse_zlg_packet(data[offset:end_pos])
                    if zlg_info:
                        status = "✓" if zlg_info['valid'] else "⚠️"
                        can_id_hex = zlg_info.get('can_id', 0)
                        print(f"     包 #{packet_num}: CAN_ID=0x{can_id_hex:02X} {zlg_info.get('cmd_type', '?')} [{zlg_info['raw_length']}B] {status}")
                        offset += 27
                        packet_num += 1
                    else:
                        offset += 1
                else:
                    offset += 1


def main():
    print("========================================")
    print("监控发送到 192.168.1.5:8003 的数据")
    print("支持 ZLG 网络包格式 (27字节)")
    print("========================================")
    print("")

    if HAS_SCAPY:
        print("✓ 使用scapy模式")
        print("✓ 开始捕获数据包...")
        print("  按 Ctrl+C 停止")
        print("")

        # 开始捕获
        try:
            sniff(
                filter="tcp dst port 8003 and dst host 192.168.1.5",
                prn=packet_callback,
                store=False
            )
        except KeyboardInterrupt:
            print("\n\n停止监控")
    else:
        print("❌ scapy未安装")
        print("")
        print("请安装scapy:")
        print("  sudo apt-get install python3-scapy")
        print("")
        print("或者使用shell脚本:")
        print("  sudo ./scripts/monitor_zhilgong_traffic.sh")


if __name__ == '__main__':
    main()
