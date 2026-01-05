#!/usr/bin/env python3
"""
Motor Feedback Launch File
Launches both publisher and subscriber nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    zlg_ip_arg = DeclareLaunchArgument(
        'zlg_ip',
        default_value='192.168.1.5',
        description='ZLG device IP address'
    )

    zlg_port_arg = DeclareLaunchArgument(
        'zlg_port',
        default_value='8002',
        description='ZLG device TCP port'
    )

    channel_arg = DeclareLaunchArgument(
        'channel',
        default_value='2',
        description='CAN channel number'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='Motor feedback publish rate (Hz)'
    )

    # Motor feedback publisher node
    publisher_node = Node(
        package='motor_feedback',
        executable='motor_feedback_publisher',
        name='motor_feedback_publisher',
        output='screen',
        parameters=[{
            'zlg_ip': LaunchConfiguration('zlg_ip'),
            'zlg_port': LaunchConfiguration('zlg_port'),
            'channel': LaunchConfiguration('channel'),
            'arb_baud': 1000000,
            'data_baud': 5000000,
            'publish_rate': LaunchConfiguration('publish_rate'),
        }]
    )

    # Motor feedback subscriber node
    subscriber_node = Node(
        package='motor_feedback',
        executable='motor_feedback_subscriber',
        name='motor_feedback_subscriber',
        output='screen',
        parameters=[{
            'display_rate': 1.0,
            'show_raw': False,
        }]
    )

    return LaunchDescription([
        zlg_ip_arg,
        zlg_port_arg,
        channel_arg,
        publish_rate_arg,
        publisher_node,
        subscriber_node,
    ])
