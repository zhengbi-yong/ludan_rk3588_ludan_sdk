#!/usr/bin/env python3
"""
使用ROS2原生DDS能力直接桥接xixiLowCmd
这避免了复杂的消息转换
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from cyclonedds.core import QoS, Policy, DDSBase
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import DataReader
from cyclonedds.pub import DataWriter
import time

class NativeDDSBridge(Node):
    def __init__(self):
        super().__init__('native_dds_bridge')

        # 获取当前域ID
        domain_id = self.get_parameter('domain_id').value if self.has_parameter('domain_id') else 0

        # 直接使用CycloneDDS原生API
        try:
            self.get_logger().info('使用原生CycloneDDS API...')

            # 创建域参与者
            self.participant = DomainParticipant(domain_id)

            # 创建原生话题
            self.topic = Topic(self.participant, 'lowcmd', 'xixiLowCmd::LowCmd')

            # 创建读者直接订阅DDS
            self.reader = DataReader(self.participant, self.topic)

            self.get_logger().info('原生DDS订阅器创建成功')

        except ImportError:
            self.get_logger().warn('CycloneDDS Python API不可用，使用ROS2回退方法')
            self.setup_ros2_fallback()

        # 创建ROS2发布器用于Foxglove
        self.setup_ros2_publishers()

    def setup_ros2_fallback(self):
        """ROS2回退方案"""
        from rclpy.qos import QoSProfile

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.VOLATILE

        # 尝试直接订阅原始类型
        try:
            self.dds_sub = self.create_subscription(
                'xixiLowCmd::LowCmd',
                '/lowcmd',
                self.dds_callback,
                qos
            )
            self.get_logger().info('ROS2原生DDS订阅成功')
        except Exception as e:
            self.get_logger().error(f'ROS2 DDS订阅失败: {e}')

    def setup_ros2_publishers(self):
        """设置ROS2发布器用于Foxglove"""
        from std_msgs.msg import ByteMultiArray
        from sensor_msgs.msg import JointState

        qos = QoSProfile(depth=10)

        # 原始字节发布器
        self.raw_pub = self.create_publisher(ByteMultiArray, '/lowcmd_raw', qos)

        # 关节状态发布器
        self.joint_pub = self.create_publisher(JointState, '/lowcmd_joints', qos)

    def dds_callback(self, msg):
        """处理DDS消息"""
        try:
            # 发布原始数据
            self.publish_raw_data(msg)

            # 发布关节状态
            self.publish_joint_states(msg)

        except Exception as e:
            self.get_logger().error(f'处理DDS消息错误: {e}')

    def publish_raw_data(self, msg):
        """发布原始字节数据"""
        from std_msgs.msg import ByteMultiArray

        raw_msg = ByteMultiArray()

        # 如果消息支持序列化
        if hasattr(msg, 'serialize'):
            raw_bytes = msg.serialize()
            raw_msg.data = list(raw_bytes)
        else:
            # 回退到基本数据
            raw_msg.data = b'xixi_lowcmd_data'

        self.raw_pub.publish(raw_msg)

    def publish_joint_states(self, msg):
        """发布关节状态用于Foxglove可视化"""
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header

        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = "robot_base"

        # 关节名称
        joint_names = [
            f'joint_{i}' for i in range(30)  # 简化的关节命名
        ]

        # 提取数据
        if hasattr(msg, 'motor_cmd'):
            positions = []
            velocities = []
            effort = []

            for i, motor in enumerate(msg.motor_cmd[:30]):
                joint_state.name.append(joint_names[i])
                positions.append(float(motor.q) if hasattr(motor, 'q') else 0.0)
                velocities.append(float(motor.dq) if hasattr(motor, 'dq') else 0.0)
                effort.append(float(motor.tau) if hasattr(motor, 'tau') else 0.0)

            joint_state.position = positions
            joint_state.velocity = velocities
            joint_state.effort = effort

        self.joint_pub.publish(joint_state)

    def run_native_loop(self):
        """使用原生DDS循环"""
        while rclpy.ok():
            try:
                # 从原生DDS读取数据
                for msg in self.reader.take_iter(timeout=0.1):
                    self.get_logger().info('收到原生DDS消息')
                    self.dds_callback(msg)
            except Exception as e:
                self.get_logger().error(f'原生DDS读取错误: {e}')
                time.sleep(0.1)

def main():
    rclpy.init()

    bridge = NativeDDSBridge()

    try:
        if hasattr(bridge, 'reader'):
            # 使用原生DDS循环
            bridge.run_native_loop()
        else:
            # 使用ROS2 spin
            rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info('关闭中...')
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()