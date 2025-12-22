#!/usr/bin/env python3
"""
DDS xixiLowCmd::LowCmd to ROS2 converter
Converts DDS LowCmd messages to standard ROS2 format for Foxglove visualization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float32MultiArray, UInt8MultiArray, UInt32
from builtin_interfaces.msg import Time
import struct
import time
import os
import sys

class DDSToROS2Converter(Node):
    def __init__(self):
        super().__init__('dds_lowcmd_converter')

        # ROS2 publishers for different message formats
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

        # Joint names for 30 joints (adjust based on your robot)
        self.joint_names = [
            'left_hip_yaw_joint', 'left_hip_roll_joint', 'left_hip_pitch_joint',
            'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
            'right_hip_yaw_joint', 'right_hip_roll_joint', 'right_hip_pitch_joint',
            'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
            'torso_joint', 'left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
            'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_pitch_joint',
            'left_wrist_roll_joint', 'right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
            'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_pitch_joint',
            'right_wrist_roll_joint', 'head_pitch_joint', 'head_yaw_joint',
            'left_gripper_joint', 'right_gripper_joint', 'waist_joint'
        ]

        # Try different DDS readers based on what's available
        self.dds_reader_type = None
        self.setup_dds_reader()

        # Start mock data timer if DDS setup fails
        if self.dds_reader_type is None:
            self.create_timer(1.0, self.publish_mock_data)

        # Statistics
        self.message_count = 0
        self.create_timer(10.0, self.print_statistics)

    def setup_dds_reader(self):
        """Setup DDS reader using available libraries"""
        # Try cyclonedds
        try:
            import cyclonedds
            from cyclonedds.domain import DomainParticipant
            from cyclonedds.sub import DataReader
            from cyclonedds.topic import Topic
            from cyclonedds.core import QoS

            self.participant = DomainParticipant()
            self.topic = Topic(
                self.participant,
                'lowcmd',
                'xixiLowCmd::LowCmd'
            )
            self.reader = DataReader(
                self.participant,
                self.topic,
                QoS(reliability=cyclonedds.core.QoSPolicy.Reliability.BestEffort)
            )
            self.dds_reader_type = 'cyclonedds'
            self.get_logger().info('DDS reader (cyclonedds) for xixiLowCmd::LowCmd initialized')
            return
        except ImportError:
            self.get_logger().warning('cyclonedds not available, trying alternatives...')
        except Exception as e:
            self.get_logger().warning(f'Failed to initialize cyclonedds: {e}')

        # Try fastdds
        try:
            from fastdds import DomainParticipant, Subscriber, Topic
            # Implementation for FastDDS would go here
            self.get_logger().warning('FastDDS support not implemented')
        except ImportError:
            self.get_logger().warning('FastDDS not available')

        # Try opendds
        try:
            import opendds
            # Implementation for OpenDDS would go here
            self.get_logger().warning('OpenDDS support not implemented')
        except ImportError:
            self.get_logger().warning('OpenDDS not available')

        # Try to read from existing ROS2 topic as fallback
        try:
            self.ros2_subscriber = self.create_subscription(
                # Try to subscribe to whatever is publishing /lowcmd
                # This is a fallback mechanism
                'sensor_msgs/msg/JointState',  # Will be replaced at runtime
                '/lowcmd',
                self.ros2_callback,
                10
            )
            self.dds_reader_type = 'ros2_fallback'
            self.get_logger().info('Using ROS2 fallback reader for /lowcmd')
            return
        except Exception as e:
            self.get_logger().warning(f'ROS2 fallback failed: {e}')

        self.get_logger().error('Could not setup any DDS reader')

    def publish_mock_data(self):
        """Publish mock data for testing when DDS is not available"""
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

    def publish_all_formats(self, motor_cmds, mode_pr, mode_machine, crc, timestamp):
        """Publish data in multiple formats for Foxglove compatibility"""

        # 1. JointState format (standard ROS2)
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = timestamp
        joint_state.name = self.joint_names[:30]  # Ensure we have exactly 30 joints
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

        # Log summary
        enabled_count = sum(1 for cmd in motor_cmds if cmd['mode'] == 1)
        self.get_logger().debug(
            f'Published: {enabled_count}/30 joints enabled, '
            f'mode_pr={mode_pr}, mode_machine={mode_machine}'
        )

    def print_statistics(self):
        """Print conversion statistics"""
        self.get_logger().info(f'Processed {self.message_count} DDS messages')

    def ros2_callback(self, msg):
        """Fallback callback for ROS2 messages"""
        try:
            self.message_count += 1
            current_time = self.get_clock().now().to_msg()

            # Try to parse the incoming message
            if hasattr(msg, 'name') and len(msg.name) > 0:
                # It's already a JointState-like message
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
            self.get_logger().error(f'Error in ROS2 fallback callback: {e}')

    def parse_motor_cmd(self, motor_cmd_data):
        """Parse motor command from binary data"""
        # Based on your IDL: mode(1) + q(4) + dq(4) + kp(4) + kd(4) + tau(4) = 21 bytes per motor
        motor_cmds = []

        # Assuming motor_cmd_data is 30 * 21 = 630 bytes
        motor_size = 21  # bytes per motor command

        for i in range(30):
            offset = i * motor_size
            if offset + motor_size <= len(motor_cmd_data):
                # Unpack binary data according to IDL structure
                motor_data = motor_cmd_data[offset:offset + motor_size]
                mode = motor_data[0]
                q, dq, kp, kd, tau = struct.unpack('<fffff', motor_data[1:21])

                motor_cmds.append({
                    'mode': mode,
                    'position': q,
                    'velocity': dq,
                    'kp': kp,
                    'kd': kd,
                    'torque': tau
                })

        return motor_cmds

    def run(self):
        """Main conversion loop"""
        self.get_logger().info('Starting DDS message processing loop...')

        while rclpy.ok():
            try:
                # Read DDS messages
                for sample in self.reader.take_iter(timeout=1.0):
                    self.process_dds_message(sample)

                # Spin ROS2
                rclpy.spin_once(self, timeout_sec=0.1)

            except Exception as e:
                self.get_logger().error(f'Error in main loop: {e}')
                time.sleep(0.1)

    def process_dds_message(self, sample):
        """Process incoming DDS message and convert to ROS2"""
        try:
            # Extract data from DDS sample
            # Structure based on your IDL: mode_pr(1) + mode_machine(1) + motor_cmd[30](630) + crc(4)

            if hasattr(sample, 'mode_pr') and hasattr(sample, 'mode_machine'):
                # DDS sample is structured object
                mode_pr = sample.mode_pr
                mode_machine = sample.mode_machine
                crc = getattr(sample, 'crc', 0)

                motor_cmds = []
                if hasattr(sample, 'motor_cmd'):
                    for motor_cmd in sample.motor_cmd:
                        motor_cmds.append({
                            'mode': motor_cmd.mode,
                            'position': motor_cmd.q,
                            'velocity': motor_cmd.dq,
                            'kp': motor_cmd.kp,
                            'kd': motor_cmd.kd,
                            'torque': motor_cmd.tau
                        })
            else:
                # Fallback: try to parse as binary data
                self.get_logger().warning('Received binary data, structure parsing needed')
                return

            # Convert to ROS2 JointState
            joint_state = JointState()
            joint_state.header = Header()
            joint_state.header.stamp = self.get_clock().now().to_msg()

            joint_state.name = self.joint_names
            joint_state.position = [cmd['position'] for cmd in motor_cmds]
            joint_state.velocity = [cmd['velocity'] for cmd in motor_cmds]
            joint_state.effort = [cmd['torque'] for cmd in motor_cmds]

            # Add mode information to joint names for debugging
            if mode_pr != 0 or mode_machine != 0:
                for i, name in enumerate(joint_state.name):
                    if i < len(motor_cmds) and motor_cmds[i]['mode'] == 1:
                        joint_state.name[i] = f"{name}_enabled"
                    else:
                        joint_state.name[i] = f"{name}_disabled"

            # Publish converted data
            self.joint_state_pub.publish(joint_state)

            # Log info
            self.get_logger().info(
                f'Converted DDS LowCmd: mode_pr={mode_pr}, mode_machine={mode_machine}, '
                f'enabled_joints={sum(1 for cmd in motor_cmds if cmd["mode"] == 1)}/30'
            )

        except Exception as e:
            self.get_logger().error(f'Error processing DDS message: {e}')

  def run(self):
        """Main conversion loop"""
        self.get_logger().info('Starting DDS message processing loop...')

        if self.dds_reader_type == 'cyclonedds':
            self.run_cyclonedds()
        elif self.dds_reader_type == 'ros2_fallback':
            # ROS2 fallback will use callbacks, just spin
            try:
                rclpy.spin(self)
            except KeyboardInterrupt:
                self.get_logger().info('Shutting down DDS to ROS2 converter...')
        else:
            # No DDS reader available, just spin for mock data
            self.get_logger().info('No DDS reader available, publishing mock data only')
            try:
                rclpy.spin(self)
            except KeyboardInterrupt:
                self.get_logger().info('Shutting down DDS to ROS2 converter...')

    def run_cyclonedds(self):
        """Run with cyclonedds reader"""
        while rclpy.ok():
            try:
                # Read DDS messages
                for sample in self.reader.take_iter(timeout=1.0):
                    self.process_dds_message(sample)

                # Spin ROS2
                rclpy.spin_once(self, timeout_sec=0.1)

            except Exception as e:
                self.get_logger().error(f'Error in main loop: {e}')
                time.sleep(0.1)

    def process_dds_message(self, sample):
        """Process incoming DDS message and convert to ROS2"""
        try:
            self.message_count += 1
            current_time = self.get_clock().now().to_msg()

            # Extract data from DDS sample
            if hasattr(sample, 'mode_pr') and hasattr(sample, 'mode_machine'):
                # DDS sample is structured object
                mode_pr = sample.mode_pr
                mode_machine = sample.mode_machine
                crc = getattr(sample, 'crc', 0)

                motor_cmds = []
                if hasattr(sample, 'motor_cmd'):
                    for motor_cmd in sample.motor_cmd:
                        motor_cmds.append({
                            'mode': motor_cmd.mode,
                            'position': motor_cmd.q,
                            'velocity': motor_cmd.dq,
                            'kp': motor_cmd.kp,
                            'kd': motor_cmd.kd,
                            'torque': motor_cmd.tau
                        })
            else:
                self.get_logger().warning('Unknown DDS sample structure')
                return

            # Publish in all formats
            self.publish_all_formats(motor_cmds, mode_pr, mode_machine, crc, current_time)

            # Log info at lower frequency
            if self.message_count % 10 == 0:
                enabled_count = sum(1 for cmd in motor_cmds if cmd['mode'] == 1)
                self.get_logger().info(
                    f'Processed {self.message_count} DDS messages: '
                    f'{enabled_count}/30 joints enabled, '
                    f'mode_pr={mode_pr}, mode_machine={mode_machine}'
                )

        except Exception as e:
            self.get_logger().error(f'Error processing DDS message: {e}')

    def run(self):
        """Main conversion loop"""
        self.get_logger().info('Starting DDS message processing loop...')

        if self.dds_reader_type == 'cyclonedds':
            self.run_cyclonedds()
        elif self.dds_reader_type == 'ros2_fallback':
            # ROS2 fallback will use callbacks, just spin
            try:
                rclpy.spin(self)
            except KeyboardInterrupt:
                self.get_logger().info('Shutting down DDS to ROS2 converter...')
        else:
            # No DDS reader available, just spin for mock data
            self.get_logger().info('No DDS reader available, publishing mock data only')
            try:
                rclpy.spin(self)
            except KeyboardInterrupt:
                self.get_logger().info('Shutting down DDS to ROS2 converter...')

    def run_cyclonedds(self):
        """Run with cyclonedds reader"""
        while rclpy.ok():
            try:
                # Read DDS messages
                for sample in self.reader.take_iter(timeout=1.0):
                    self.process_dds_message(sample)

                # Spin ROS2
                rclpy.spin_once(self, timeout_sec=0.1)

            except Exception as e:
                self.get_logger().error(f'Error in main loop: {e}')
                time.sleep(0.1)


def main():
    rclpy.init()

    try:
        converter = DDSToROS2Converter()
        converter.run()
    except KeyboardInterrupt:
        print('Shutting down DDS to ROS2 converter...')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()