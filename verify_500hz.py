#!/usr/bin/env python3
"""
500Hz插值验证工具
专门用于验证motor_controller是否真的以500Hz发送
并展示线性插值的实际效果
"""

import subprocess
import re
import time
import struct
from collections import defaultdict, deque
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np

class InterpolationValidator:
    def __init__(self):
        self.dds_messages = []  # 存储DDS消息时间戳
        self.can_frames = []    # 存储CAN帧时间戳和数据
        self.motor_data = defaultdict(lambda: {'timestamps': [], 'positions': []})

        self.start_time = None
        self.dds_count = 0
        self.can_count = 0

        # 电机ID映射
        self.motor_names = {
            0x201: "LeftAnkleRoll",
            0x202: "LeftAnklePitch",
            0x203: "RightAnkleRoll",
            0x204: "RightAnklePitch"
        }

    def parse_can_data(self, data):
        """解析CAN数据为位置值"""
        try:
            # 解析位置数据 (前2字节, little-endian)
            pos_int = struct.unpack('<h', data[0:2])[0]
            pos = pos_int * 12.5 / 32767.0  # 转换为rad
            return pos
        except:
            return None

    def start_monitoring(self, duration=10):
        """开始监控CAN总线"""
        print(f"🔍 开始监控500Hz插值效果 ({duration}秒)")
        print("=" * 60)

        self.start_time = time.time()
        end_time = self.start_time + duration

        try:
            # 启动candump监控
            candump = subprocess.Popen(
                ['candump', 'can0', '-tA'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                bufsize=1
            )

            print("📊 实时统计:")
            print("-" * 60)

            last_print_time = self.start_time
            dds_recent_count = 0
            can_recent_count = 0

            while time.time() < end_time:
                line = candump.stdout.readline()
                if not line:
                    break

                # 解析candump输出
                match = re.match(r'\(.*?\)\s+can0\s+([0-9A-F]+)\s+\[(\d+)\]\s+(.+)', line.strip())
                if match:
                    can_id = int(match.group(1), 16)
                    dlc = int(match.group(2))
                    data_str = match.group(3)
                    timestamp = time.time()

                    # 只处理我们关心的电机ID
                    if 0x201 <= can_id <= 0x204:
                        self.can_count += 1
                        can_recent_count += 1

                        # 解析位置数据
                        data_bytes = bytes.fromhex(data_str.replace(' ', ''))
                        pos = self.parse_can_data(data_bytes)

                        if pos is not None:
                            motor_name = self.motor_names.get(can_id, f"Motor_{can_id}")
                            self.motor_data[can_id]['timestamps'].append(timestamp)
                            self.motor_data[can_id]['positions'].append(pos)
                            self.can_frames.append({
                                'timestamp': timestamp,
                                'can_id': can_id,
                                'position': pos,
                                'raw_data': data_str
                            })

                # 每秒显示统计信息
                current_time = time.time()
                if current_time - last_print_time >= 1.0:
                    elapsed = current_time - self.start_time
                    can_hz = self.can_count / elapsed if elapsed > 0 else 0

                    print(f"\r⏱️  {elapsed:5.1f}s | CAN: {self.can_count:5d}帧 ({can_hz:5.1f}Hz) | "
                          f"最近1s: {can_recent_count:3d}帧 | 收集位置点: {len(self.can_frames):6d}", end='', flush=True)

                    can_recent_count = 0
                    last_print_time = current_time

        except KeyboardInterrupt:
            print(f"\n⏹️  用户中断监控")
        finally:
            if 'candump' in locals():
                candump.terminate()

        print(f"\n\n📊 监控完成!")
        print(f"总时间: {time.time() - self.start_time:.1f}秒")
        print(f"总CAN帧数: {self.can_count}")
        print(f"平均CAN频率: {self.can_count / (time.time() - self.start_time):.1f} Hz")

    def analyze_interpolation(self):
        """分析插值效果"""
        print(f"\n🔬 插值效果分析:")
        print("=" * 60)

        for can_id in [0x201, 0x202, 0x203, 0x204]:
            if can_id in self.motor_data:
                timestamps = self.motor_data[can_id]['timestamps']
                positions = self.motor_data[can_id]['positions']

                if len(timestamps) < 2:
                    continue

                motor_name = self.motor_names[can_id]
                print(f"\n🔧 {motor_name} (ID: 0x{can_id:03X}):")

                # 计算时间间隔
                intervals = []
                for i in range(1, len(timestamps)):
                    interval = (timestamps[i] - timestamps[i-1]) * 1000  # 转换为ms
                    intervals.append(interval)

                if intervals:
                    avg_interval = np.mean(intervals)
                    std_interval = np.std(intervals)
                    min_interval = np.min(intervals)
                    max_interval = np.max(intervals)

                    print(f"   📏 平均间隔: {avg_interval:.2f}ms (期望: 2.00ms)")
                    print(f"   📊 标准差: {std_interval:.2f}ms")
                    print(f"   📉 最小间隔: {min_interval:.2f}ms")
                    print(f"   📈 最大间隔: {max_interval:.2f}ms")

                    # 计算500Hz准确性
                    target_interval = 2.0  # 2ms = 500Hz
                    accuracy = (1 - abs(avg_interval - target_interval) / target_interval) * 100
                    print(f"   🎯 500Hz准确性: {accuracy:.1f}%")

                    # 分析位置变化
                    pos_changes = []
                    for i in range(1, len(positions)):
                        pos_change = abs(positions[i] - positions[i-1])
                        pos_changes.append(pos_change)

                    if pos_changes:
                        avg_change = np.mean(pos_changes)
                        print(f"   📍 平均位置变化: {avg_change:.6f} rad")

                        # 检测是否有插值特征
                        if avg_change > 0.0001 and std_interval < 0.5:
                            print(f"   ✅ 检测到插值特征 (小步长 + 稳定间隔)")
                        elif std_interval > 1.0:
                            print(f"   ⚠️  间隔不稳定，可能需要调优")
                        else:
                            print(f"   🤔 需要更多数据进行分析")

    def show_interpolation_demo(self, motor_id=0x201):
        """展示插值效果的数值演示"""
        print(f"\n🎭 插值效果数值演示:")
        print("=" * 60)

        if motor_id not in self.motor_data:
            print(f"❌ 未找到电机数据 (ID: 0x{motor_id:03X})")
            return

        timestamps = self.motor_data[motor_id]['timestamps']
        positions = self.motor_data[motor_id]['positions']

        if len(positions) < 10:
            print(f"⚠️  数据点不足，需要至少10个点进行演示")
            return

        # 显示前10个数据点
        motor_name = self.motor_names.get(motor_id, f"Motor_{motor_id}")
        print(f"📍 {motor_name} 前10个插值数据点:")

        for i in range(min(10, len(positions))):
            rel_time = (timestamps[i] - timestamps[0]) * 1000  # 相对时间(ms)
            print(f"   #{i+1:2d}: {rel_time:6.1f}ms → {positions[i]:+8.6f} rad")

        # 计算插值密度
        total_time = (timestamps[-1] - timestamps[0])
        if total_time > 0:
            points_per_second = len(positions) / total_time
            print(f"\n📊 插值密度: {points_per_second:.1f} 点/秒")

            if points_per_second > 400 and points_per_second < 600:
                print(f"✅ 插值频率正常 (目标: 500 Hz)")
            else:
                print(f"⚠️  插值频率异常")

    def generate_report(self):
        """生成详细报告"""
        print(f"\n📋 详细验证报告:")
        print("=" * 60)

        total_time = time.time() - self.start_time if self.start_time else 0

        print(f"📊 基本统计:")
        print(f"   监控时长: {total_time:.1f} 秒")
        print(f"   CAN总帧数: {self.can_count}")
        print(f"   平均频率: {self.can_count / total_time:.1f} Hz")

        # 验证500Hz
        expected_frames = total_time * 500
        actual_frames = self.can_count
        accuracy = (actual_frames / expected_frames) * 100 if expected_frames > 0 else 0

        print(f"\n🎯 500Hz验证:")
        print(f"   期望帧数: {expected_frames:.0f}")
        print(f"   实际帧数: {actual_frames}")
        print(f"   准确率: {accuracy:.1f}%")

        if accuracy >= 95:
            print(f"   ✅ 500Hz发送验证通过!")
        elif accuracy >= 80:
            print(f"   ⚠️  基本达到500Hz，但可能需要优化")
        else:
            print(f"   ❌ 未达到500Hz要求")

        # 插值效果验证
        print(f"\n🔧 插值效果验证:")

        has_interpolation = False
        for can_id in self.motor_data:
            timestamps = self.motor_data[can_id]['timestamps']
            if len(timestamps) > 2:
                intervals = [(timestamps[i] - timestamps[i-1]) * 1000 for i in range(1, min(10, len(timestamps)))]
                avg_interval = np.mean(intervals) if intervals else 0

                if 1.5 <= avg_interval <= 2.5:  # 接近2ms
                    motor_name = self.motor_names.get(can_id, f"Motor_{can_id}")
                    print(f"   ✅ {motor_name}: 插值正常 ({avg_interval:.2f}ms)")
                    has_interpolation = True
                else:
                    motor_name = self.motor_names.get(can_id, f"Motor_{can_id}")
                    print(f"   ⚠️  {motor_name}: 插值异常 ({avg_interval:.2f}ms)")

        if not has_interpolation:
            print(f"   ❌ 未检测到正常的插值效果")

def main():
    validator = InterpolationValidator()

    print("🚀 500Hz插值验证工具")
    print("=" * 60)
    print("此工具将:")
    print("1. 监控CAN总线10秒")
    print("2. 分析500Hz发送准确性")
    print("3. 验证线性插值效果")
    print("4. 生成详细验证报告")
    print("=" * 60)

    try:
        # 监控10秒
        validator.start_monitoring(duration=10)

        # 分析插值效果
        validator.analyze_interpolation()

        # 展示数值演示
        validator.show_interpolation_demo(motor_id=0x201)

        # 生成最终报告
        validator.generate_report()

    except Exception as e:
        print(f"❌ 错误: {e}")

if __name__ == "__main__":
    main()