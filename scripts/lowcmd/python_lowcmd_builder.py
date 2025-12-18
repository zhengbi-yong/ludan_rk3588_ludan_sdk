#!/usr/bin/env python3
"""
Python LowCmd Message Builder
展示如何在Python中构建/lowcmd话题消息

Author: Claude Code Assistant
Date: 2025-12-17
"""

import struct
import numpy as np
import time
from typing import List, Dict

class CRC32:
    """CRC32计算工具类"""

    @staticmethod
    def calculate(data_bytes: bytes) -> int:
        """计算CRC32校验值"""
        polynomial = 0x04c11db7
        crc = 0xFFFFFFFF

        for byte in data_bytes:
            crc ^= (byte << 24)
            for _ in range(8):
                if crc & 0x80000000:
                    crc = (crc << 1) ^ polynomial
                else:
                    crc <<= 1
                crc &= 0xFFFFFFFF

        return crc

class MotorCmd:
    """电机命令数据结构"""

    def __init__(self, motor_id: int = 0):
        self.motor_id = motor_id
        self.mode = 1           # 0=禁用, 1=启用
        self.q = 0.0           # 目标位置 (弧度)
        self.dq = 0.0          # 目标速度 (弧度/秒)
        self.tau = 0.0         # 前馈力矩 (Nm)
        self.kp = 100.0        # 位置增益
        self.kd = 3.0          # 速度增益
        self.reserve = [0, 0, 0]  # 保留字段

    def set_sine_wave(self, amplitude: float, frequency: float, phase_offset: float = 0.0):
        """设置为正弦波模式"""
        self.amplitude = amplitude
        self.frequency = frequency
        self.phase_offset = phase_offset
        self.start_time = time.time()

    def update_sine_position(self):
        """更新正弦波位置"""
        if hasattr(self, 'start_time'):
            current_time = time.time() - self.start_time
            phase = 2 * np.pi * self.frequency * current_time + self.phase_offset
            self.q = self.amplitude * np.sin(phase)
            self.dq = self.amplitude * 2 * np.pi * self.frequency * np.cos(phase)

    def pack(self) -> bytes:
        """打包为二进制格式"""
        # MotorCmd二进制格式: mode(1) + q(4) + dq(4) + tau(4) + kp(4) + kd(4) + reserve(12) = 33字节
        data = struct.pack('<B', self.mode)
        data += struct.pack('<f', self.q)
        data += struct.pack('<f', self.dq)
        data += struct.pack('<f', self.tau)
        data += struct.pack('<f', self.kp)
        data += struct.pack('<f', self.kd)
        data += struct.pack('<III', *self.reserve)
        return data

class LowCmdBuilder:
    """LowCmd消息构建器"""

    def __init__(self, robot_type: str = "hg"):  # "hg" for H1/G1, "go" for GO2/B2
        self.robot_type = robot_type
        self.mode_pr = 1          # PR模式
        self.mode_machine = 1     # 机器类型 (1=G1)
        self.reserve = [0, 0, 0, 0]
        self.crc = 0

        # 初始化电机命令数组
        if robot_type == "hg":
            self.num_motors = 35  # H1/G1有35个电机
        else:
            self.num_motors = 20  # GO2/B2有20个电机

        self.motor_commands = [MotorCmd(i) for i in range(self.num_motors)]

    def set_motor_command(self, motor_id: int, **kwargs):
        """设置特定电机的命令"""
        if 0 <= motor_id < self.num_motors:
            motor = self.motor_commands[motor_id]
            for key, value in kwargs.items():
                if hasattr(motor, key):
                    setattr(motor, key, value)

    def set_sine_wave_motors(self, motor_ids: List[int], amplitude: float = 0.3,
                           frequency: float = 0.5):
        """为指定电机设置正弦波运动"""
        for i, motor_id in enumerate(motor_ids):
            if 0 <= motor_id < self.num_motors:
                phase_offset = i * (2 * np.pi / len(motor_ids))
                self.motor_commands[motor_id].set_sine_wave(
                    amplitude, frequency, phase_offset
                )

    def update_positions(self):
        """更新所有电机的位置（用于正弦波等动态命令）"""
        for motor in self.motor_commands:
            if hasattr(motor, 'update_sine_position'):
                motor.update_sine_position()

    def to_dict(self) -> Dict:
        """转换为字典格式（用于ROS1桥接）"""
        result = {
            'mode_pr': self.mode_pr,
            'mode_machine': self.mode_machine,
            'motors': []
        }

        for motor in self.motor_commands:
            motor_dict = {
                'id': motor.motor_id,
                'mode': motor.mode,
                'q': motor.q,
                'dq': motor.dq,
                'tau': motor.tau,
                'kp': motor.kp,
                'kd': motor.kd
            }
            result['motors'].append(motor_dict)

        return result

    def pack(self) -> bytes:
        """打包为二进制格式"""
        # LowCmd头部
        data = struct.pack('<BB', self.mode_pr, self.mode_machine)

        # 电机命令
        for motor in self.motor_commands:
            data += motor.pack()

        # 保留字段和CRC
        data += struct.pack('<IIII', *self.reserve)

        # 计算CRC (暂时使用魔数)
        self.crc = 0x12345678  # CRC32.calculate(data)
        data += struct.pack('<I', self.crc)

        return data

    def create_pose_stamped_message(self):
        """创建用于ROS1桥接的PoseStamped消息数据"""
        # 提取前4个电机的位置信息映射到pose
        pose_data = {
            'header': {
                'stamp': time.time(),
                'frame_id': 'robot_base'
            },
            'pose': {
                'position': {
                    'x': self.motor_commands[0].q if len(self.motor_commands) > 0 else 0.0,
                    'y': self.motor_commands[1].q if len(self.motor_commands) > 1 else 0.0,
                    'z': self.motor_commands[2].q if len(self.motor_commands) > 2 else 0.0
                },
                'orientation': {
                    'x': self.motor_commands[3].q if len(self.motor_commands) > 3 else 0.0,
                    'y': 0.0,
                    'z': 0.0,
                    'w': 1.0
                }
            }
        }
        return pose_data

def create_lowcmd_for_g1() -> LowCmdBuilder:
    """创建G1机器人的LowCmd消息"""
    lowcmd = LowCmdBuilder("hg")  # H1/G1类型

    # 设置模式
    lowcmd.mode_pr = 1      # PR模式
    lowcmd.mode_machine = 1 # G1类型

    # 为脚踝关节设置正弦波 (基于之前的映射)
    # 左脚踝: 关节4 (pitch), 5 (roll)
    # 右脚踝: 关节10 (pitch), 11 (roll)
    lowcmd.set_sine_wave_motors(
        motor_ids=[4, 5, 10, 11],
        amplitude=0.3,  # 0.3弧度
        frequency=0.5   # 0.5Hz
    )

    return lowcmd

def create_lowcmd_for_sine_test() -> LowCmdBuilder:
    """创建正弦波测试用的LowCmd消息"""
    lowcmd = LowCmdBuilder("hg")

    # 设置模式
    lowcmd.mode_pr = 1
    lowcmd.mode_machine = 1

    # 为前几个关节设置不同相位的正弦波
    lowcmd.set_sine_wave_motors(
        motor_ids=[0, 1, 2, 3],
        amplitude=0.2,
        frequency=1.0
    )

    # 设置其他电机参数
    for i in range(lowcmd.num_motors):
        lowcmd.set_motor_command(i, kp=80.0, kd=2.0)  # 小齿轮参数

    return lowcmd

def main():
    """主函数 - 演示LowCmd消息的创建和使用"""
    print("=" * 60)
    print("    Python LowCmd Message Builder Demo")
    print("=" * 60)

    # 创建正弦波测试消息
    lowcmd = create_lowcmd_for_sine_test()

    print(f"Robot Type: {lowcmd.robot_type}")
    print(f"Number of Motors: {lowcmd.num_motors}")
    print(f"Mode PR: {lowcmd.mode_pr}")
    print(f"Mode Machine: {lowcmd.mode_machine}")
    print()

    # 更新位置并显示前几个电机的状态
    print("Motor Commands (first 5):")
    print("-" * 80)
    print(f"{'ID':>3} | {'Mode':>5} | {'Position':>10} | {'Velocity':>10} | {'Kp':>8} | {'Kd':>8}")
    print("-" * 80)

    for i in range(min(5, len(lowcmd.motor_commands))):
        motor = lowcmd.motor_commands[i]
        print(f"{motor.motor_id:>3} | {motor.mode:>5} | {motor.q:>10.4f} | {motor.dq:>10.4f} | {motor.kp:>8.1f} | {motor.kd:>8.1f}")

    print()

    # 转换为字典格式
    lowcmd_dict = lowcmd.to_dict()
    print(f"Dictionary format size: {len(str(lowcmd_dict))} characters")
    print(f"Binary packed size: {len(lowcmd.pack())} bytes")

    print()

    # 创建ROS1桥接消息
    pose_msg = lowcmd.create_pose_stamped_message()
    print("ROS1 PoseStamped message:")
    print(f"  Position: x={pose_msg['pose']['position']['x']:.4f}, "
          f"y={pose_msg['pose']['position']['y']:.4f}, "
          f"z={pose_msg['pose']['position']['z']:.4f}")
    print(f"  Orientation: x={pose_msg['pose']['orientation']['x']:.4f}")

    print()
    print("=" * 60)
    print("Demo completed successfully!")
    print("=" * 60)

if __name__ == "__main__":
    main()