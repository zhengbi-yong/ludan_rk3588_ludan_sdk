#!/usr/bin/env python3
"""
监听8888端口接收Jetson的LowCmd数据并转发到本地ROS2的/lowcmd topic
"""

import socket
import json
import time
import signal
import sys
import math
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

                        # 每条消息都打印详细的调试信息
                        if self.message_count <= 10 or self.message_count % 100 == 0:
                            self.get_logger().info(f"📨 JSON消息 #{self.message_count} 详细检查:")
                            self.get_logger().info(f"   来源: {addr[0]}:{addr[1]}")
                            self.get_logger().info(f"   时间戳: {message.get('timestamp', 'N/A')}")
                            self.get_logger().info(f"   序列号: {message.get('sequence', 'N/A')}")

                            # 检查是否使用新的xixilowcmd格式
                            if 'motor_cmd' in message:
                                motor_cmd_dict = message['motor_cmd']
                                self.get_logger().info(f"   ✅ 检测到xixilowcmd格式，包含 {len(motor_cmd_dict)} 个电机命令")

                                # 强制打印完整结构（前3条消息）
                                if self.message_count <= 3:
                                    self.get_logger().info(f"   完整JSON结构:")
                                    json_str = json.dumps(message, indent=2, ensure_ascii=False)
                                    for line in json_str.split('\n'):
                                        self.get_logger().info(f"     {line}")

                                # 检查关键脚踝关节 (4, 5, 10, 11)
                                key_motors = [4, 5, 10, 11]
                                for motor_id in key_motors:
                                    motor_key = str(motor_id)
                                    if motor_key in motor_cmd_dict:
                                        motor_data = motor_cmd_dict[motor_key]
                                        if isinstance(motor_data, dict):
                                            q_val = motor_data.get('q', 0.0)
                                            dq_val = motor_data.get('dq', 0.0)
                                            tau_val = motor_data.get('tau', 0.0)
                                            mode_val = motor_data.get('mode', 0)
                                            kp_val = motor_data.get('kp', 0.0)
                                            kd_val = motor_data.get('kd', 0.0)

                                            # 检查是否所有值都为零
                                            is_all_zero = (abs(q_val) < 0.001 and abs(dq_val) < 0.001 and abs(tau_val) < 0.001)
                                            status = "⚠️  全为零值!" if is_all_zero else "✅ 有有效数据"

                                            self.get_logger().info(f"     Motor {motor_id}: q={q_val:8.4f}, dq={dq_val:6.3f}, tau={tau_val:6.3f}, mode={mode_val}, kp={kp_val:4.1f}, kd={kd_val:4.1f} {status}")
                                        else:
                                            self.get_logger().warning(f"     Motor {motor_id}: 数据格式错误: {type(motor_data)}")
                                    else:
                                        self.get_logger().warning(f"     Motor {motor_id}: 缺失")

                                # 统计非零电机
                                non_zero_count = 0
                                for motor_key, motor_data in motor_cmd_dict.items():
                                    if isinstance(motor_data, dict) and abs(motor_data.get('q', 0.0)) >= 0.001:
                                        non_zero_count += 1
                                self.get_logger().info(f"   总计: {non_zero_count}/{len(motor_cmd_dict)} 个电机有非零位置")

                            else:
                                # 旧格式或其他格式
                                self.get_logger().warning(f"   ❌ 未检测到'motor_cmd'字段，可能使用旧格式")
                                self.get_logger().info(f"   可用字段: {list(message.keys())}")

                                # 检查旧格式的positions字段
                                if 'positions' in message:
                                    positions = message['positions']
                                    self.get_logger().info(f"   检测到旧格式positions字段，包含 {len(positions)} 个电机")

                            # 检查其他字段
                            for field in ['velocities', 'efforts', 'gains', 'motor_modes']:
                                if field in message:
                                    field_data = message[field]
                                    if isinstance(field_data, dict):
                                        self.get_logger().info(f"   {field} 字段: {len(field_data)} 个条目")
                                    elif isinstance(field_data, list):
                                        self.get_logger().info(f"   {field} 字段: {len(field_data)} 个元素")

                            self.get_logger().info("=" * 80)

                        # 转发到ROS2 topics
                        self.forward_to_ros2(message)

                        # 计算频率
                        if self.message_count > 0:
                            frequency = self.message_count / elapsed_time
                            if self.message_count % 50 == 0:  # 每50条消息显示一次
                                self.get_logger().info(f"📊 已处理 {self.message_count} 条JSON消息, 频率: {frequency:.1f} Hz")

                    except json.JSONDecodeError:
                        # JSON解析失败，记录原始数据并生成测试数据
                        self.get_logger().warning(f"📨 UDP数据 #{self.message_count} 来自 {addr[0]}:{addr[1]} (JSON解析失败)")
                        self.get_logger().warning(f"   原始数据长度: {len(data)} 字节")

                        # 显示原始数据的前50字节（十六进制）
                        hex_data = data[:50].hex()
                        self.get_logger().warning(f"   原始数据(hex): {hex_data}")

                        # 尝试检测是否为其他格式
                        try:
                            data_str = data.decode('utf-8', errors='ignore')
                            if '{' in data_str and '}' in data_str:
                                self.get_logger().warning(f"   原始文本片段: {data_str[:100]}...")
                            else:
                                self.get_logger().warning(f"   非文本数据，可能为二进制格式")
                        except:
                            self.get_logger().warning(f"   纯二进制数据")

                        # 生成基于时间的正弦波测试数据作为备用
                        test_message = self.generate_sine_wave_test_data(current_time)
                        self.get_logger().info(f"✅ 使用备用测试数据代替无法解析的UDP数据")

                        # 转发到ROS2 topics
                        self.forward_to_ros2(test_message)

                        # 计算频率
                        if self.message_count > 0:
                            frequency = self.message_count / elapsed_time
                            if self.message_count % 50 == 0:  # 每50条消息显示一次
                                self.get_logger().info(f"📊 已处理 {self.message_count} 条UDP消息(测试数据), 频率: {frequency:.1f} Hz")

            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"❌ 错误: {e}")
                break

        # 显示统计信息
        self.show_statistics(start_time)

    def generate_sine_wave_test_data(self, current_time):
        """生成基于时间的正弦波测试数据"""
        # 正弦波参数（与 Jetson 脚本保持一致）
        sine_amplitude = 0.3  # 弧度
        sine_frequency = 0.5   # Hz
        target_joints = [4, 5, 10, 11]  # 脚踝关节

        message = {
            'timestamp': current_time,
            'sequence': self.message_count,
            'mode_pr': 0,  # PR模式
            'mode_machine': 1,  # G1机器人
            'positions': {},
            'velocities': {},
            'efforts': {},
            'gains': {},
            'motor_modes': []
        }

        # 为所有30个电机生成数据 (ID 0-29，对应 motor_controller 的 1-30)
        for motor_id in range(30):
            # 设置基本的电机参数
            kp = 20.0 if motor_id in target_joints else 10.0  # 位置增益
            kd = 2.0 if motor_id in target_joints else 1.0   # 速度增益

            message['gains'][str(motor_id)] = {'kp': kp, 'kd': kd}
            message['motor_modes'].append(0 if motor_id not in target_joints else 1)

            if motor_id in target_joints:
                # 为目标脚踝关节生成正弦波
                joint_index = target_joints.index(motor_id)
                phase_offset = joint_index * (2 * math.pi / len(target_joints))
                phase = 2 * math.pi * sine_frequency * current_time + phase_offset

                pos = sine_amplitude * math.sin(phase)
                vel = sine_amplitude * 2 * math.pi * sine_frequency * math.cos(phase)
                effort = 0.0  # 无额外力矩

                message['positions'][str(motor_id)] = {
                    'q': pos, 'dq': vel, 'tau': effort, 'mode': 1
                }
                message['velocities'][str(motor_id)] = {
                    'dq': vel
                }
                message['efforts'][str(motor_id)] = {
                    'tau': effort
                }
            else:
                # 其他电机保持零位
                message['positions'][str(motor_id)] = {
                    'q': 0.0, 'dq': 0.0, 'tau': 0.0, 'mode': 0
                }
                message['velocities'][str(motor_id)] = {
                    'dq': 0.0
                }
                message['efforts'][str(motor_id)] = {
                    'tau': 0.0
                }

        return message

    def forward_to_ros2(self, message):
        """将接收到的消息转发到ROS2 topics"""
        try:
            timestamp = message.get('timestamp', time.time())
            self.sequence = message.get('sequence', self.sequence + 1)

            # 检查是否为新的 xixilowcmd 格式
            if 'motor_cmd' in message:
                # 新的 xixilowcmd 格式处理
                motor_cmd_dict = message['motor_cmd']
                self.get_logger().debug(f"处理xixilowcmd格式，包含{len(motor_cmd_dict)}个电机")

                # 直接创建 LowCmd 消息
                try:
                    lowcmd_msg = LowCmd()

                    # 创建 30 个电机命令（ROS2 要求固定长度）
                    ros_motor_cmds = []
                    for i in range(30):
                        motor_cmd = MotorCmd()
                        motor_cmd.id = i

                        # 从字典中查找对应ID的电机
                        motor_key = str(i)
                        if motor_key in motor_cmd_dict:
                            # 从UDP数据获取电机信息
                            jetson_motor = motor_cmd_dict[motor_key]
                            if isinstance(jetson_motor, dict):
                                motor_cmd.mode = jetson_motor.get('mode', 0)
                                motor_cmd.q = jetson_motor.get('q', 0.0)
                                motor_cmd.dq = jetson_motor.get('dq', 0.0)
                                motor_cmd.tau = jetson_motor.get('tau', 0.0)
                                motor_cmd.kp = jetson_motor.get('kp', 0.0)
                                motor_cmd.kd = jetson_motor.get('kd', 0.0)

                                if self.message_count <= 10 and i in [4, 5, 6, 10, 11]:
                                    self.get_logger().info(f"✅ Motor {i}: q={motor_cmd.q:.4f}, dq={motor_cmd.dq:.3f}, tau={motor_cmd.tau:.3f}, mode={motor_cmd.mode}, kp={motor_cmd.kp:.1f}, kd={motor_cmd.kd:.1f}")
                            else:
                                # 数据格式错误，使用默认值
                                self.get_logger().warning(f"电机{i}数据格式错误: {type(jetson_motor)}")
                                motor_cmd.mode = 0
                                motor_cmd.q = 0.0
                                motor_cmd.dq = 0.0
                                motor_cmd.tau = 0.0
                                motor_cmd.kp = 0.0
                                motor_cmd.kd = 0.0
                        else:
                            # 该电机ID不在消息中，使用 mode=0 标记为非活跃
                            motor_cmd.mode = 0
                            motor_cmd.q = 0.0
                            motor_cmd.dq = 0.0
                            motor_cmd.tau = 0.0
                            motor_cmd.kp = 0.0
                            motor_cmd.kd = 0.0

                        ros_motor_cmds.append(motor_cmd)

                    lowcmd_msg.motor_cmd = ros_motor_cmds
                    self.lowcmd_pub.publish(lowcmd_msg)

                    if self.message_count <= 10:
                        self.get_logger().info(f"✅ 成功发布 xixilowcmd/LowCmd 消息 (包含{len(ros_motor_cmds)}个电机命令)")

                    # 同时创建其他格式的消息以保持兼容性
                    self.create_compatibility_messages(motor_cmd_dict, message)

                except Exception as e:
                    self.get_logger().error(f"❌ 创建 xixilowcmd/LowCmd 消息失败: {e}")

            else:
                # 旧格式处理（兼容性）
                self.get_logger().debug("处理旧格式消息")
                self.forward_old_format(message)

        except Exception as e:
            self.get_logger().error(f"❌ 转发到ROS2失败: {e}")

    def create_compatibility_messages(self, motor_cmd_dict, original_message):
        """创建兼容性消息（JointState、Float32MultiArray等）"""
        try:
            # 创建JointState消息
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.header.frame_id = "lowcmd"
            joint_state.name = []
            joint_state.position = []
            joint_state.velocity = []
            joint_state.effort = []

            # 创建Float32MultiArray消息
            positions_msg = Float32MultiArray()
            velocities_msg = Float32MultiArray()
            efforts_msg = Float32MultiArray()
            motor_modes_msg = UInt8MultiArray()

            # 初始化30个电机的数据
            positions_msg.data = [0.0] * 30
            velocities_msg.data = [0.0] * 30
            efforts_msg.data = [0.0] * 30
            motor_modes_msg.data = [0] * 30

            # 从motor_cmd字典填充数据
            for i in range(30):
                motor_key = str(i)
                if motor_key in motor_cmd_dict and isinstance(motor_cmd_dict[motor_key], dict):
                    motor_data = motor_cmd_dict[motor_key]

                    # JointState
                    joint_state.name.append(f"joint_{i}")
                    joint_state.position.append(motor_data.get('q', 0.0))
                    joint_state.velocity.append(motor_data.get('dq', 0.0))
                    joint_state.effort.append(motor_data.get('tau', 0.0))

                    # Float32MultiArray
                    positions_msg.data[i] = motor_data.get('q', 0.0)
                    velocities_msg.data[i] = motor_data.get('dq', 0.0)
                    efforts_msg.data[i] = motor_data.get('tau', 0.0)
                    motor_modes_msg.data[i] = motor_data.get('mode', 0)

            # 发布兼容性消息
            self.joint_states_pub.publish(joint_state)
            self.positions_pub.publish(positions_msg)
            self.velocities_pub.publish(velocities_msg)
            self.efforts_pub.publish(efforts_msg)
            self.motor_modes_pub.publish(motor_modes_msg)

            # 发布模式信息
            mode_pr_msg = UInt32()
            mode_pr_msg.data = original_message.get('mode_pr', 0)
            self.mode_pr_pub.publish(mode_pr_msg)

            mode_machine_msg = UInt32()
            mode_machine_msg.data = original_message.get('mode_machine', 0)
            self.mode_machine_pub.publish(mode_machine_msg)

        except Exception as e:
            self.get_logger().error(f"❌ 创建兼容性消息失败: {e}")

    def forward_old_format(self, message):
        """处理旧格式消息（兼容性）"""
        try:
            # 提取旧格式数据
            positions = message.get('positions', {})
            velocities = message.get('velocities', {})
            efforts = message.get('efforts', {})
            motor_modes = message.get('motor_modes', [])
            gains = message.get('gains', {})

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
            motor_modes_msg = UInt8MultiArray()

            # 填充数据 (30个关节)
            positions_msg.data = [0.0] * 30
            velocities_msg.data = [0.0] * 30
            efforts_msg.data = [0.0] * 30

            if motor_modes:
                motor_modes_msg.data = motor_modes
            else:
                motor_modes_msg.data = [0] * 30

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

            # 发布消息
            self.positions_pub.publish(positions_msg)
            self.velocities_pub.publish(velocities_msg)
            self.efforts_pub.publish(efforts_msg)
            self.motor_modes_pub.publish(motor_modes_msg)

            # 发布模式信息
            mode_pr_msg = UInt32()
            mode_pr_msg.data = message.get('mode_pr', 0)
            self.mode_pr_pub.publish(mode_pr_msg)

            mode_machine_msg = UInt32()
            mode_machine_msg.data = message.get('mode_machine', 0)
            self.mode_machine_pub.publish(mode_machine_msg)

            # 创建LowCmd消息（从旧格式转换）
            try:
                lowcmd_msg = LowCmd()
                motor_cmd_array = []

                for i in range(30):
                    joint_key = str(i)
                    pos_data = positions.get(joint_key, {})
                    vel_data = velocities.get(joint_key, {})
                    eff_data = efforts.get(joint_key, {})
                    gain_data = gains.get(joint_key, {})

                    motor_cmd = MotorCmd()
                    motor_cmd.id = i
                    motor_cmd.mode = motor_modes_msg.data[i] if i < len(motor_modes_msg.data) else pos_data.get('mode', 0)
                    motor_cmd.q = pos_data.get('q', 0.0)
                    motor_cmd.dq = vel_data.get('dq', 0.0) if isinstance(vel_data, dict) else 0.0
                    motor_cmd.tau = eff_data.get('tau', 0.0) if isinstance(eff_data, dict) else 0.0
                    motor_cmd.kp = gain_data.get('kp', 0.0) if isinstance(gain_data, dict) else 0.0
                    motor_cmd.kd = gain_data.get('kd', 0.0) if isinstance(gain_data, dict) else 0.0

                    motor_cmd_array.append(motor_cmd)

                lowcmd_msg.motor_cmd = motor_cmd_array
                self.lowcmd_pub.publish(lowcmd_msg)

            except Exception as e:
                self.get_logger().error(f"❌ 从旧格式创建LowCmd消息失败: {e}")

        except Exception as e:
            self.get_logger().error(f"❌ 处理旧格式消息失败: {e}")

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