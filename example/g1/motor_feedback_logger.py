#!/usr/bin/env python3
"""
motor_feedback_logger.py

ROS2订阅节点，用于记录电机反馈数据到带时间戳的日志文件。

订阅topic: /motor_feedback
消息类型: multi_port_motor_feedback::msg::LowState

运行方式:
    1. 首先source ROS2环境:
       source /home/linaro/ludan_sdk/multi_port_motor_feedback/multi_port_motor_feedback_ros2/install/setup.bash

    2. 启动电机反馈发布节点:
       ros2 run multi_port_motor_feedback can_motor_feedback_publisher

    3. 运行本脚本:
       python3 motor_feedback_logger.py

    4. 按Ctrl+C停止记录，日志将自动保存
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from multi_port_motor_feedback.msg import LowState
import datetime
import os
import sys
import argparse


class MotorFeedbackLogger(Node):
    """电机反馈数据记录器"""

    def __init__(self, log_dir=None):
        super().__init__('motor_feedback_logger')

        # 设置日志目录
        if log_dir is None:
            log_dir = os.path.dirname(os.path.abspath(__file__))

        self.log_dir = log_dir
        self.log_file = None
        self.msg_count = 0
        self.start_time = None

        # 创建QoS配置（与发布端匹配）
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_profile.durability = DurabilityPolicy.VOLATILE

        # 创建订阅者
        self.subscription = self.create_subscription(
            LowState,
            '/motor_feedback',
            self.feedback_callback,
            qos_profile
        )

        # 创建日志文件
        self._create_log_file()

        self.get_logger().info('=' * 60)
        self.get_logger().info('Motor Feedback Logger Started')
        self.get_logger().info(f'Subscribing to: /motor_feedback')
        self.get_logger().info(f'Log directory: {self.log_dir}')
        self.get_logger().info(f'Log file: {os.path.basename(self.log_file_path)}')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Waiting for motor feedback data...')
        self.get_logger().info('Press Ctrl+C to stop and save log')

    def _create_log_file(self):
        """创建带时间戳的日志文件"""
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        log_filename = f'motor_feedback_log_{timestamp}.log'
        self.log_file_path = os.path.join(self.log_dir, log_filename)

        try:
            self.log_file = open(self.log_file_path, 'w', encoding='utf-8')
            self._write_log_header()
        except Exception as e:
            self.get_logger().error(f'Failed to create log file: {e}')
            sys.exit(1)

    def _write_log_header(self):
        """写入日志文件头部"""
        self.log_file.write('#' * 80 + '\n')
        self.log_file.write('# Motor Feedback Data Log\n')
        self.log_file.write(f'# Created: {datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
        self.log_file.write('# Topic: /motor_feedback\n')
        self.log_file.write('# Message Type: multi_port_motor_feedback::msg::LowState\n')
        self.log_file.write('#' * 80 + '\n')
        self.log_file.write('\n')
        self.log_file.flush()

    def feedback_callback(self, msg):
        """处理接收到的电机反馈数据"""
        # 记录开始时间
        if self.start_time is None:
            self.start_time = datetime.datetime.now()

        self.msg_count += 1

        # 获取时间戳
        # 1. ROS消息时间戳（纳秒）
        ros_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        # 2. 系统当前时间戳
        system_timestamp = datetime.datetime.now()

        # 解析消息并写入日志
        self._write_motor_data(msg, ros_timestamp, system_timestamp)

        # 每100条消息打印一次进度
        if self.msg_count % 100 == 0:
            elapsed = (datetime.datetime.now() - self.start_time).total_seconds()
            rate = self.msg_count / elapsed if elapsed > 0 else 0
            self.get_logger().info(
                f'Received {self.msg_count} messages | '
                f'Elapsed: {elapsed:.1f}s | '
                f'Rate: {rate:.1f} Hz'
            )

    def _write_motor_data(self, msg, ros_timestamp, system_timestamp):
        """将电机数据写入日志文件"""
        # 写入时间戳行
        log_line = (
            f'[Timestamp] System: {system_timestamp.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]} | '
            f'ROS: {ros_timestamp:.9f} s | '
            f'Msg Count: {self.msg_count}\n'
        )
        self.log_file.write(log_line)

        # 写入30个电机的数据
        for i in range(30):
            motor = msg.motor_state[i]
            motor_line = (
                f'  Motor[{motor.id:2d}] | '
                f'State: {motor.state:2d} | '
                f'Pos: {motor.position:8.3f} rad | '
                f'Vel: {motor.velocity:7.2f} rad/s | '
                f'Eff: {motor.effort:7.2f} Nm | '
                f'T_MOS: {motor.temp_mos:3d} C | '
                f'T_Rotor: {motor.temp_rotor:3d} C\n'
            )
            self.log_file.write(motor_line)

        self.log_file.write('\n')
        self.log_file.flush()

    def shutdown(self):
        """关闭日志记录器"""
        self.get_logger().info(f'Total messages received: {self.msg_count}')

        if self.start_time:
            elapsed = (datetime.datetime.now() - self.start_time).total_seconds()
            rate = self.msg_count / elapsed if elapsed > 0 else 0
            self.get_logger().info(f'Total elapsed time: {elapsed:.2f} seconds')
            self.get_logger().info(f'Average rate: {rate:.2f} Hz')

        if self.log_file:
            # 写入日志尾部
            self.log_file.write('#' * 80 + '\n')
            self.log_file.write(f'# Log ended: {datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
            self.log_file.write(f'# Total messages: {self.msg_count}\n')
            self.log_file.write('#' * 80 + '\n')
            self.log_file.close()
            self.get_logger().info(f'Log saved to: {self.log_file_path}')


def main(args=None):
    """主函数"""
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='Motor Feedback Logger')
    parser.add_argument(
        '--log-dir',
        type=str,
        default=None,
        help='Directory to save log files (default: current directory)'
    )
    parsed_args = parser.parse_args(args)

    # 初始化ROS2
    rclpy.init(args=args)

    try:
        # 创建日志记录器节点
        logger = MotorFeedbackLogger(log_dir=parsed_args.log_dir)

        # 运行节点
        rclpy.spin(logger)

    except KeyboardInterrupt:
        print('\n\nShutdown requested by user (Ctrl+C)')
    except Exception as e:
        print(f'\n\nError: {e}')
    finally:
        # 清理资源
        if 'logger' in locals():
            logger.shutdown()
        rclpy.shutdown()
        print('\nMotor Feedback Logger stopped.')


if __name__ == '__main__':
    main()
