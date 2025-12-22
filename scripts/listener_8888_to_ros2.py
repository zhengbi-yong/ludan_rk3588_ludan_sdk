#!/usr/bin/env python3
"""
监听8888端口接收Jetson的LowCmd数据并转发到本地ROS2的/lowcmd topic
"""

import socket
import json
import time
import signal
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt8MultiArray, UInt32
from sensor_msgs.msg import JointState
from xixilowcmd.msg import LowCmd, MotorCmd  # 使用 xixilowcmd 消息格式

class LowCmdUDPToROS2(Node):
    def __init__(self, port=8888):
        super().__init__('lowcmd_udp_to_ros2')

        self.port = port
        self.running = True
        self.message_count = 0
        self.sequence = 0

        # 创建UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('0.0.0.0', port))
        self.socket.settimeout(1.0)  # 1秒超时

        # 创建ROS2发布器
        self.lowcmd_pub = self.create_publisher(LowCmd, '/lowcmd', 10)
        self.positions_pub = self.create_publisher(Float32MultiArray, '/lowcmd_positions', 10)
        self.velocities_pub = self.create_publisher(Float32MultiArray, '/lowcmd_velocities', 10)
        self.efforts_pub = self.create_publisher(Float32MultiArray, '/lowcmd_efforts', 10)
        self.motor_modes_pub = self.create_publisher(UInt8MultiArray, '/lowcmd_motor_modes', 10)
        self.joint_states_pub = self.create_publisher(JointState, '/lowcmd_joint_states', 10)
        self.mode_pr_pub = self.create_publisher(UInt32, '/lowcmd_mode_pr', 10)
        self.mode_machine_pub = self.create_publisher(UInt32, '/lowcmd_mode_machine', 10)

        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)

        self.get_logger().info(f"🎯 启动UDP监听器，端口: {port}")
        self.get_logger().info(f"📍 绑定地址: 0.0.0.0:{port}")
        self.get_logger().info("⏳ 等待Jetson发送数据...")
        self.get_logger().info("🔊 按 Ctrl+C 停止监听")
        self.get_logger().info("=" * 60)

    def signal_handler(self, signum, frame):
        self.get_logger().info("🛑 接收到停止信号，正在关闭...")
        self.running = False

    def start(self):
        start_time = time.time()

        while self.running and rclpy.ok():
            try:
                # 接收数据
                data, addr = self.socket.recvfrom(4096)

                if data:
                    self.message_count += 1
                    current_time = time.time()
                    elapsed_time = current_time - start_time

                    try:
                        # 尝试解析JSON数据
                        message = json.loads(data.decode('utf-8'))

                        self.get_logger().debug(f"📨 消息 #{self.message_count} 来自 {addr[0]}:{addr[1]}")

                        # 转发到ROS2 topics
                        self.forward_to_ros2(message)

                        # 计算频率
                        if self.message_count > 0:
                            frequency = self.message_count / elapsed_time
                            if self.message_count % 50 == 0:  # 每50条消息显示一次
                                self.get_logger().info(f"📊 已处理 {self.message_count} 条消息, 频率: {frequency:.1f} Hz")

                    except json.JSONDecodeError:
                        self.get_logger().warning(f"⚠️ JSON解析错误，原始数据长度: {len(data)} 字节")

            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"❌ 错误: {e}")
                break

        # 显示统计信息
        self.show_statistics(start_time)

    def forward_to_ros2(self, message):
        """将接收到的消息转发到ROS2 topics"""
        try:
            timestamp = message.get('timestamp', time.time())
            self.sequence = message.get('sequence', self.sequence + 1)

            # 提取positions数据
            positions = message.get('positions', {})
            velocities = message.get('velocities', {})
            efforts = message.get('efforts', {})
            motor_modes = message.get('motor_modes', [])
            gains = message.get('gains', {})  # 添加增益数据解析

            # 创建JointState消息
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.header.frame_id = "lowcmd"
            joint_state.name = []
            joint_state.position = []
            joint_state.velocity = []
            joint_state.effort = []

            # 填充关节数据
            for joint_id in sorted(positions.keys(), key=lambda x: int(x) if x.isdigit() else x):
                pos_data = positions.get(joint_id, {})
                vel_data = velocities.get(joint_id, {})
                eff_data = efforts.get(joint_id, {})

                joint_state.name.append(f"joint_{joint_id}")
                joint_state.position.append(pos_data.get('q', 0.0))
                joint_state.velocity.append(vel_data.get('dq', 0.0) if isinstance(vel_data, dict) else float(vel_data.get('dq', 0.0)))
                joint_state.effort.append(eff_data.get('tau', 0.0) if isinstance(eff_data, dict) else float(eff_data.get('tau', 0.0)))

            self.joint_states_pub.publish(joint_state)

            # 创建Float32MultiArray消息
            positions_msg = Float32MultiArray()
            velocities_msg = Float32MultiArray()
            efforts_msg = Float32MultiArray()

            # 填充位置数据 (假设30个关节)
            positions_msg.data = [0.0] * 30
            velocities_msg.data = [0.0] * 30
            efforts_msg.data = [0.0] * 30

            for i, joint_id in enumerate(range(30)):
                joint_key = str(i)
                if joint_key in positions:
                    pos_data = positions[joint_key]
                    positions_msg.data[i] = pos_data.get('q', 0.0)

                    vel_data = velocities.get(joint_key, {})
                    if isinstance(vel_data, dict):
                        velocities_msg.data[i] = vel_data.get('dq', 0.0)

                    eff_data = efforts.get(joint_key, {})
                    if isinstance(eff_data, dict):
                        efforts_msg.data[i] = eff_data.get('tau', 0.0)

            self.positions_pub.publish(positions_msg)
            self.velocities_pub.publish(velocities_msg)
            self.efforts_pub.publish(efforts_msg)

            # 创建电机模式消息
            motor_modes_msg = UInt8MultiArray()
            if motor_modes:
                motor_modes_msg.data = motor_modes
            else:
                # 如果没有提供模式，使用默认值
                motor_modes_msg.data = [0] * 30
            self.motor_modes_pub.publish(motor_modes_msg)

            # 发布模式信息
            mode_pr_msg = UInt32()
            mode_pr_msg.data = message.get('mode_pr', 0)
            self.mode_pr_pub.publish(mode_pr_msg)

            mode_machine_msg = UInt32()
            mode_machine_msg.data = message.get('mode_machine', 0)
            self.mode_machine_pub.publish(mode_machine_msg)

            # 创建 xixilowcmd 格式的 LowCmd 消息
            try:
                lowcmd_msg = LowCmd()

                # 创建 MotorCmd 数组 (30个电机)
                motor_cmd_array = []
                for i in range(30):
                    joint_key = str(i)

                    # 从原始消息中提取数据
                    pos_data = positions.get(joint_key, {})
                    vel_data = velocities.get(joint_key, {})
                    eff_data = efforts.get(joint_key, {})
                    gain_data = gains.get(joint_key, {})

                    motor_cmd = MotorCmd()
                    motor_cmd.id = i  # 电机ID (0-29)

                    # 设置模式
                    if i < len(motor_modes_msg.data):
                        motor_cmd.mode = motor_modes_msg.data[i]
                    else:
                        motor_cmd.mode = pos_data.get('mode', 0) if isinstance(pos_data, dict) else 0

                    # 设置位置、速度、力矩
                    if isinstance(pos_data, dict):
                        motor_cmd.q = pos_data.get('q', 0.0)
                        motor_cmd.dq = vel_data.get('dq', 0.0) if isinstance(vel_data, dict) else 0.0
                        motor_cmd.tau = eff_data.get('tau', 0.0) if isinstance(eff_data, dict) else 0.0
                        motor_cmd.kp = gain_data.get('kp', 0.0) if isinstance(gain_data, dict) else 0.0
                        motor_cmd.kd = gain_data.get('kd', 0.0) if isinstance(gain_data, dict) else 0.0
                    else:
                        # 回退到数组数据
                        motor_cmd.q = positions_msg.data[i] if i < len(positions_msg.data) else 0.0
                        motor_cmd.dq = velocities_msg.data[i] if i < len(velocities_msg.data) else 0.0
                        motor_cmd.tau = efforts_msg.data[i] if i < len(efforts_msg.data) else 0.0
                        motor_cmd.kp = 0.0
                        motor_cmd.kd = 0.0

                    motor_cmd_array.append(motor_cmd)

                lowcmd_msg.motor_cmd = motor_cmd_array

                self.lowcmd_pub.publish(lowcmd_msg)
                self.get_logger().debug(f"✅ 成功发布 xixilowcmd/LowCmd 消息 (包含{len(motor_cmd_array)}个电机命令)")
            except Exception as e:
                self.get_logger().error(f"❌ 创建 xixilowcmd/LowCmd 消息失败: {e}")

            self.get_logger().debug(f"✅ 成功转发消息到ROS2 topics")

        except Exception as e:
            self.get_logger().error(f"❌ 转发到ROS2失败: {e}")

    def get_local_ip(self):
        """获取本机IP地址"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "未知"

    def show_statistics(self, start_time):
        total_time = time.time() - start_time
        self.get_logger().info("=" * 60)
        self.get_logger().info("📊 监听统计:")
        self.get_logger().info(f"   总时间: {total_time:.1f} 秒")
        self.get_logger().info(f"   接收消息: {self.message_count} 条")
        if total_time > 0:
            self.get_logger().info(f"   平均频率: {self.message_count/total_time:.1f} Hz")
        self.get_logger().info("🔚 监听器已停止")

def main():
    print("🚀 LowCmd UDP to ROS2 转发器")
    print("🎯 接收Jetson UDP数据并转发到本地ROS2 topics")
    print()

    # 初始化ROS2
    rclpy.init()

    try:
        # 检查端口参数
        port = 8888
        if len(sys.argv) > 1:
            try:
                port = int(sys.argv[1])
            except ValueError:
                print("❌ 端口参数必须是数字")
                return 1

        node = LowCmdUDPToROS2(port)

        # 使用rclpy.spin来处理ROS2回调
        try:
            import threading
            # 在单独的线程中运行UDP监听
            udp_thread = threading.Thread(target=node.start)
            udp_thread.daemon = True
            udp_thread.start()

            # 主线程处理ROS2
            rclpy.spin(node)

        except KeyboardInterrupt:
            pass
        finally:
            node.running = False
            node.destroy_node()

    except Exception as e:
        print(f"❌ 错误: {e}")
        return 1
    finally:
        rclpy.shutdown()

    return 0

if __name__ == "__main__":
    exit(main())