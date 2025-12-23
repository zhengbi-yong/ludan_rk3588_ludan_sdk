#!/usr/bin/env python3
"""
电机使能工具 - 使用MotorCmd格式直接使能单个电机

使用方法:
  python3 enable_motor.py 4 1    # 使能电机4
  python3 enable_motor.py 4 0    # 禁使能电机4
  python3 enable_motor.py 4 252  # 发送自定义指令252 (0xFC) 给电机4

监控工具:
  sudo ./scripts/monitor_simple.sh          # 监控发送到192.168.1.5:8002的数据
  sudo ./scripts/monitor_zhilgong_traffic.sh # 详细hex格式监控
  python3 ./scripts/monitor_zhilgong.py     # Python解析监控(需要scapy)
"""

import sys
import rclpy
from rclpy.node import Node
from xixilowcmd.msg import MotorCmd

def send_motor_enable(motor_id, command_value):
    """发送电机使能指令 - 使用MotorCmd格式，直接使能单个电机"""

    # 初始化ROS2
    rclpy.init()

    try:
        # 创建发布节点
        node = rclpy.create_node('motor_enable_publisher')
        publisher = node.create_publisher(MotorCmd, '/motor_enable', 1)

        # 创建MotorCmd消息
        msg = MotorCmd()
        msg.id = motor_id
        msg.q = float(command_value)  # 使用q字段传递使能指令
        msg.dq = 0.0
        msg.tau = 0.0
        msg.kp = 0.0
        msg.kd = 0.0
        msg.mode = 0

        print(f"📤 发送MotorCmd消息: id={msg.id}, q={msg.q}")

        # 发布消息
        publisher.publish(msg)

        # 等待消息传递完成
        import time
        time.sleep(0.2)  # 等待200ms确保消息被接收

        # 显示发送信息
        command_names = {
            0: "禁使能",
            1: "使能"
        }

        # 映射到实际的CAN frame值
        can_frame_values = {
            0: 0xFD,  # 禁使能
            1: 0xFC   # 使能
        }
        can_frame_value = can_frame_values.get(command_value, command_value)
        command_name = command_names.get(command_value, f"自定义指令({command_value})")

        print(f"✅ 已发送使能指令: Motor {motor_id} -> {command_name}")
        print(f"   CAN Frame: FF FF FF FF FF FF FF {can_frame_value:02X}")

    finally:
        rclpy.shutdown()

def send_multiple_enable(commands_list):
    """批量发送多个电机使能指令"""
    import time

    # 初始化ROS2
    rclpy.init()

    try:
        # 创建发布节点
        node = rclpy.create_node('motor_enable_publisher')
        publisher = node.create_publisher(MotorCmd, '/motor_enable', 10)

        print(f"📤 批量发送 {len(commands_list)} 个使能指令")

        for cmd in commands_list:
            motor_id = cmd["motor_id"]
            command_value = cmd["command"]

            # 创建MotorCmd消息
            msg = MotorCmd()
            msg.id = motor_id
            msg.q = float(command_value)
            msg.dq = 0.0
            msg.tau = 0.0
            msg.kp = 0.0
            msg.kd = 0.0
            msg.mode = 0

            # 发布消息
            publisher.publish(msg)

            command_name = "使能" if command_value == 1 else ("禁使能" if command_value == 0 else f"自定义({command_value})")
            print(f"   Motor {motor_id} -> {command_name}")

            # 短暂延迟
            time.sleep(0.05)

        print(f"✅ 已批量发送 {len(commands_list)} 个使能指令")

    finally:
        rclpy.shutdown()

def main():
    if len(sys.argv) == 3:
        # 单个电机使能模式: python3 enable_motor.py 4 1
        try:
            motor_id = int(sys.argv[1])
            command_value = int(sys.argv[2])

            # 验证参数范围
            if not (0 <= motor_id <= 29):
                print(f"❌ 错误: motor_id 必须在 0-29 范围内，当前值: {motor_id}")
                sys.exit(1)

            if not (0 <= command_value <= 255):
                print(f"❌ 错误: command 必须在 0-255 范围内，当前值: {command_value}")
                sys.exit(1)

            # 发送使能指令
            send_motor_enable(motor_id, command_value)

        except ValueError as e:
            print(f"❌ 参数错误: {e}")
            sys.exit(1)
        except Exception as e:
            print(f"❌ 执行错误: {e}")
            sys.exit(1)

    elif len(sys.argv) == 2 and sys.argv[1] == "--batch":
        # 批量模式示例
        commands = [
            {"motor_id": 4, "command": 1},   # 使能电机4
            {"motor_id": 5, "command": 1},   # 使能电机5
            {"motor_id": 10, "command": 0},  # 禁使能电机10
            {"motor_id": 11, "command": 0}    # 禁使能电机11
        ]
        send_multiple_enable(commands)

    else:
        print("使用方法:")
        print("  python3 enable_motor.py <motor_id> <command>          # 单个电机使能")
        print("  python3 enable_motor.py --batch                     # 批量使能示例")
        print()
        print("参数说明:")
        print("  motor_id: 电机ID (0-29)")
        print("  command: 使能指令")
        print("    0 = 禁使能")
        print("    1 = 使能")
        print("    其他 = 自定义指令")
        print()
        print("示例:")
        print("  python3 enable_motor.py 4 1     # 使能电机4")
        print("  python3 enable_motor.py 4 0     # 禁使能电机4")
        print("  python3 enable_motor.py 4 252   # 自定义指令0xFC")
        print("  python3 enable_motor.py --batch # 批量使能脚踝关节")

if __name__ == '__main__':
    main()
