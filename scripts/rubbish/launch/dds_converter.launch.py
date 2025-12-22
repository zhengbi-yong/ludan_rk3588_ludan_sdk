#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    dds_domain_arg = DeclareLaunchArgument(
        'dds_domain',
        default_value='0',
        description='DDS domain ID'
    )

    joint_state_topic_arg = DeclareLaunchArgument(
        'joint_state_topic',
        default_value='/lowcmd_joint_states',
        description='ROS2 topic for joint state commands'
    )

    # DDS to ROS2 converter node
    dds_converter_node = Node(
        package='rclpy_components',
        executable='component_container_isolated',
        name='dds_converter_container',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    # Alternative: Run Python script directly
    converter_script_node = Node(
        package='rclpy_components',
        executable='component_container_isolated',
        name='dds_lowcmd_converter_node',
        output='screen',
        arguments=['python3', '/home/linaro/motor_dds_proj/src/dds_to_ros2_converter.py'],
        emulate_tty=True,
        parameters=[
            {'dds_domain': LaunchConfiguration('dds_domain')},
            {'joint_state_topic': LaunchConfiguration('joint_state_topic')}
        ]
    )

    return LaunchDescription([
        dds_domain_arg,
        joint_state_topic_arg,
        converter_script_node,
    ])