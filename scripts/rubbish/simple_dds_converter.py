#!/usr/bin/env python3
"""
Simple DDS to ROS2 converter for xixiLowCmd::LowCmd
Converts DDS messages to ROS2 format for Foxglove visualization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float32MultiArray, UInt8MultiArray, UInt32
import time
import sys

class SimpleDDSConverter(Node):
    def __init__(self):
        super().__init__('simple_dds_converter')

        # Publishers for different message formats (Foxglove compatible)
        self.joint_state_pub = self.create_publisher(
            JointState, '/lowcmd_joint_states', 10)
        self.positions_pub = self.create_publisher(
            Float32MultiArray, '/lowcmd_positions', 10)
        self.velocities_pub = self.create_publisher(
            Float32MultiArray, '/lowcmd_velocities', 10)
        self.efforts_pub = self.create_publisher(
            Float32MultiArray, '/lowcmd_efforts', 10)
        self.motor_modes_pub = self.create_publisher(
            UInt8MultiArray, '/lowcmd_motor_modes', 10)
        self.mode_pr_pub = self.create_publisher(
            UInt32, '/lowcmd_mode_pr', 10)
        self.mode_machine_pub = self.create_publisher(
            UInt32, '/lowcmd_mode_machine', 10)

        # Joint names for 30 joints (update based on your robot)
        self.joint_names = [
            f'joint_{i:02d}' for i in range(30)
        ]

        # Message counter
        self.message_count = 0

        # Try to subscribe to existing /lowcmd topic (if any)
        self.setup_fallback_subscription()

        # Start mock data timer
        self.create_timer(1.0, self.publish_mock_data)

        self.get_logger().info('Simple DDS to ROS2 converter started')
        self.get_logger().info('Publishing mock data every second')

    def setup_fallback_subscription(self):
        """Try to subscribe to existing ROS2 topics"""
        try:
            # Try different possible message types for /lowcmd
            self.lowcmd_sub = self.create_subscription(
                'sensor_msgs/msg/JointState',
                '/lowcmd',
                self.ros2_callback,
                10
            )
            self.get_logger().info('Subscribed to /lowcmd as JointState')
        except Exception as e:
            self.get_logger().warning(f'Could not subscribe to /lowcmd: {e}')

    def ros2_callback(self, msg):
        """Callback for ROS2 messages"""
        try:
            self.message_count += 1
            current_time = self.get_clock().now().to_msg()

            # Convert incoming message to our format
            motor_cmds = []
            for i in range(min(30, len(msg.name))):
                motor_cmds.append({
                    'mode': 1,
                    'position': msg.position[i] if i < len(msg.position) else 0.0,
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                    'kp': 50.0,
                    'kd': 5.0,
                    'torque': msg.effort[i] if i < len(msg.effort) else 0.0
                })

            self.publish_all_formats(motor_cmds, 1, 1, 0, current_time)

        except Exception as e:
            self.get_logger().error(f'Error in callback: {e}')

    def publish_mock_data(self):
        """Publish mock data for testing"""
        current_time = self.get_clock().now().to_msg()

        # Generate mock motor commands
        mock_motor_cmds = []
        for i in range(30):
            mock_motor_cmds.append({
                'mode': 1 if i % 3 == 0 else 0,  # Enable every 3rd motor
                'position': 0.1 * i,
                'velocity': 0.05 * i,
                'kp': 50.0 + i,
                'kd': 5.0 + i * 0.1,
                'torque': 0.01 * i
            })

        # Publish in all formats
        self.publish_all_formats(mock_motor_cmds, 1, 2, 12345, current_time)

        self.message_count += 1
        if self.message_count % 5 == 0:
            enabled_count = sum(1 for cmd in mock_motor_cmds if cmd['mode'] == 1)
            self.get_logger().info(
                f'Published mock data #{self.message_count}: '
                f'{enabled_count}/30 joints enabled'
            )

    def publish_all_formats(self, motor_cmds, mode_pr, mode_machine, crc, timestamp):
        """Publish data in multiple formats for Foxglove compatibility"""

        # 1. JointState format (standard ROS2)
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = timestamp
        joint_state.name = self.joint_names[:30]
        joint_state.position = [cmd['position'] for cmd in motor_cmds]
        joint_state.velocity = [cmd['velocity'] for cmd in motor_cmds]
        joint_state.effort = [cmd['torque'] for cmd in motor_cmds]
        self.joint_state_pub.publish(joint_state)

        # 2. Individual Float32MultiArray formats
        positions = Float32MultiArray()
        positions.data = [cmd['position'] for cmd in motor_cmds]
        self.positions_pub.publish(positions)

        velocities = Float32MultiArray()
        velocities.data = [cmd['velocity'] for cmd in motor_cmds]
        self.velocities_pub.publish(velocities)

        efforts = Float32MultiArray()
        efforts.data = [cmd['torque'] for cmd in motor_cmds]
        self.efforts_pub.publish(efforts)

        # 3. Motor modes
        motor_modes = UInt8MultiArray()
        motor_modes.data = [cmd['mode'] for cmd in motor_cmds]
        self.motor_modes_pub.publish(motor_modes)

        # 4. Control modes
        mode_pr_msg = UInt32()
        mode_pr_msg.data = mode_pr
        self.mode_pr_pub.publish(mode_pr_msg)

        mode_machine_msg = UInt32()
        mode_machine_msg.data = mode_machine
        self.mode_machine_pub.publish(mode_machine_msg)


def main():
    rclpy.init()

    try:
        converter = SimpleDDSConverter()
        print('Simple DDS to ROS2 converter running...')
        print('Available topics:')
        print('  /lowcmd_joint_states (sensor_msgs/JointState)')
        print('  /lowcmd_positions (std_msgs/Float32MultiArray)')
        print('  /lowcmd_velocities (std_msgs/Float32MultiArray)')
        print('  /lowcmd_efforts (std_msgs/Float32MultiArray)')
        print('  /lowcmd_motor_modes (std_msgs/UInt8MultiArray)')
        print('  /lowcmd_mode_pr (std_msgs/UInt32)')
        print('  /lowcmd_mode_machine (std_msgs/UInt32)')
        print('\nPress Ctrl+C to stop')

        rclpy.spin(converter)

    except KeyboardInterrupt:
        print('\nShutting down converter...')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()