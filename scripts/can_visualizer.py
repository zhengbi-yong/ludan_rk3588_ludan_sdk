#!/usr/bin/env python3
"""
CAN帧可视化工具
实时监控和显示CAN通信，按电机ID分组显示
"""

import sys
import subprocess
import re
import time
import struct
from collections import defaultdict, deque
from datetime import datetime

class CANVisualizer:
    def __init__(self):
        self.motor_data = defaultdict(lambda: {
            'count': 0,
            'last_pos': 0.0,
            'last_vel': 0.0,
            'last_tau': 0.0,
            'last_kp': 0.0,
            'last_kd': 0.0,
            'timestamps': deque(maxlen=100),
            'last_update': 0
        })

        self.total_count = 0
        self.start_time = time.time()

        # CAN ID到电机名称的映射
        self.motor_names = {
            0x201: "Motor 1 (LeftAnkleRoll)",
            0x202: "Motor 2 (LeftAnklePitch)",
            0x203: "Motor 3 (RightAnkleRoll)",
            0x204: "Motor 4 (RightAnklePitch)"
        }

        # 颜色代码
        self.colors = {
            'reset': '\033[0m',
            'red': '\033[91m',
            'green': '\033[92m',
            'yellow': '\033[93m',
            'blue': '\033[94m',
            'magenta': '\033[95m',
            'cyan': '\033[96m',
            'white': '\033[97m',
            'bold': '\033[1m',
            'dim': '\033[2m'
        }

    def parse_motor_data(self, can_id, data):
        """解析CAN帧数据为电机参数"""
        try:
            # 解析8字节数据 (MIT模式协议)
            p_int = struct.unpack('<h', data[0:2])[0]  # Position (int16)
            v_int = struct.unpack('<h', data[2:4])[0]  # Velocity (int16)
            t_int = struct.unpack('<h', data[4:6])[0]  # Torque (int16)
            kp_int = struct.unpack('<h', data[6:8])[0]  # Kp (int16)

            # 转换回浮点值
            pos = p_int * 12.5 / 32767.0
            vel = v_int * 30.0 / 32767.0
            tau = t_int * 10.0 / 32767.0
            kp = kp_int * 500.0 / 32767.0

            return pos, vel, tau, kp
        except:
            return 0.0, 0.0, 0.0, 0.0

    def update_motor_data(self, can_id, data):
        """更新电机数据"""
        pos, vel, tau, kp = self.parse_motor_data(can_id, data)

        motor = self.motor_data[can_id]
        motor['last_pos'] = pos
        motor['last_vel'] = vel
        motor['last_tau'] = tau
        motor['last_kp'] = kp
        motor['last_kd'] = 1.0  # 默认值，实际可能需要从数据解析
        motor['count'] += 1
        motor['timestamps'].append(time.time())
        motor['last_update'] = time.time()

        self.total_count += 1

    def calculate_frequency(self, can_id):
        """计算特定电机的频率"""
        timestamps = self.motor_data[can_id]['timestamps']
        if len(timestamps) < 2:
            return 0.0

        time_span = timestamps[-1] - timestamps[0]
        if time_span > 0:
            return len(timestamps) / time_span
        return 0.0

    def display_header(self):
        """显示标题栏"""
        elapsed = time.time() - self.start_time
        total_hz = self.total_count / elapsed if elapsed > 0 else 0

        print(f"\n{self.colors['bold']}")
        print("╔════════════════════════════════════════════════════════════════╗")
        print("║                    CAN帧可视化监控器                      ║")
        print(f"║ 总帧数: {self.total_count:6d}    总频率: {total_hz:6.1f} Hz    运行时间: {elapsed:5.1f}s     ║")
        print("╚════════════════════════════════════════════════════════════════╝")
        print(f"{self.colors['reset']}")

    def display_motor_status(self):
        """显示电机状态"""
        print(f"{self.colors['cyan']}┌─────────────────────┬─────────┬───────────┬───────────┬──────────┬─────────┬──────────┐{self.colors['reset']}")
        print(f"{self.colors['cyan']}│ 电机ID/名称        │  帧数   │  频率(Hz) │  位置(rad)│ 速度(rad/s)│  扭矩(Nm)│    Kp    │{self.colors['reset']}")
        print(f"{self.colors['cyan']}├─────────────────────┼─────────┼───────────┼───────────┼──────────┼─────────┼──────────┤{self.colors['reset']}")

        # 按CAN ID排序显示
        for can_id in sorted(self.motor_data.keys()):
            motor = self.motor_data[can_id]
            freq = self.calculate_frequency(can_id)
            motor_name = self.motor_names.get(can_id, f"Motor {can_id-0x200}")

            # 颜色编码：有数据时显示绿色，无数据时显示红色
            color = self.colors['green'] if motor['count'] > 0 else self.colors['red']

            print(f"{color}│ {motor_name:<17} │ {motor['count']:7d} │ {freq:9.1f} │ "
                  f"{motor['last_pos']:9.3f} │ {motor['last_vel']:8.3f} │ "
                  f"{motor['last_tau']:7.2f} │ {motor['last_kp']:7.1f} │{self.colors['reset']}")

        print(f"{self.colors['cyan']}└─────────────────────┴─────────┴───────────┴───────────┴──────────┴─────────┴──────────┘{self.colors['reset']}")

    def display_latest_frames(self, num_frames=5):
        """显示最新的CAN帧"""
        print(f"\n{self.colors['yellow']}📡 最新CAN帧 (最近{num_frames}帧):{self.colors['reset']}")
        print(f"{self.colors['dim']}┌─────────────┬─────┬────────────────────────────────────────┬────────────────┐{self.colors['reset']}")
        print(f"{self.colors['dim']}│    时间     │ ID  │              原始数据                 │   解析值      │{self.colors['reset']}")
        print(f"{self.colors['dim']}├─────────────┼─────┼────────────────────────────────────────┼────────────────┤{self.colors['reset']}")

        # 这里我们需要从最近的数据中提取，简化显示
        for can_id in sorted(self.motor_data.keys())[-num_frames:]:
            motor = self.motor_data[can_id]
            if motor['count'] > 0:
                time_str = f"{time.time() - self.start_time:7.3f}s"
                motor_name = self.motor_names.get(can_id, f"M{can_id-0x200}")

                print(f"│ {time_str} │ {can_id:3X} │ P:{motor['last_pos']:+.3f} V:{motor['last_vel']:+.3f} "
                      f"T:{motor['last_tau']:+.2f} Kp:{motor['last_kp']:.1f} │ {motor_name:<14} │")

    def run(self):
        """主运行循环"""
        try:
            # 启动candump进程
            candump = subprocess.Popen(
                ['candump', 'can0', '-tA'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                bufsize=1
            )

            print(f"{self.colors['green']}✅ CAN可视化工具已启动{self.colors['reset']}")
            print(f"{self.colors['dim']}监控接口: can0{self.colors['reset']}")
            print(f"{self.colors['dim']}按 Ctrl+C 退出{self.colors['reset']}\n")

            for line in iter(candump.stdout.readline, ''):
                # 解析candump输出
                # 格式: (timestamp) can0  ID [DLC]  data
                match = re.match(r'\(.*?\)\s+can0\s+([0-9A-F]+)\s+\[(\d+)\]\s+(.+)', line.strip())
                if match:
                    can_id = int(match.group(1), 16)
                    dlc = int(match.group(2))
                    data_str = match.group(3)

                    # 解析数据字节
                    data_bytes = bytes.fromhex(data_str.replace(' ', ''))

                    # 只处理我们关心的电机ID范围
                    if 0x201 <= can_id <= 0x204:
                        self.update_motor_data(can_id, data_bytes)

                    # 每100帧更新一次显示
                    if self.total_count % 100 == 0:
                        print("\033[2J\033[H", end="")  # 清屏并移到顶部
                        self.display_header()
                        self.display_motor_status()

        except KeyboardInterrupt:
            print(f"\n{self.colors['yellow']}🛑 用户中断，退出监控{self.colors['reset']}")
        except Exception as e:
            print(f"{self.colors['red']}❌ 错误: {e}{self.colors['reset']}")
        finally:
            if 'candump' in locals():
                candump.terminate()

            # 显示最终统计
            print(f"\n{self.colors['bold']}📊 最终统计:{self.colors['reset']}")
            print(f"总帧数: {self.total_count}")
            print(f"运行时间: {time.time() - self.start_time:.1f} 秒")
            print(f"平均频率: {self.total_count / (time.time() - self.start_time):.1f} Hz")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        interface = sys.argv[1]
    else:
        interface = "can0"

    visualizer = CANVisualizer()
    visualizer.run()