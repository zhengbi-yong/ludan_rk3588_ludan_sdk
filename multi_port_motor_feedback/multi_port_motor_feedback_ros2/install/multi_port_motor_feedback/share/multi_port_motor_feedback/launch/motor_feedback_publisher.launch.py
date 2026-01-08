from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    zlg_ip_arg = DeclareLaunchArgument(
        'zlg_ip',
        default_value='192.168.1.5',
        description='ZLG CAN device IP address'
    )

    # CAN motor feedback publisher node
    can_motor_feedback_publisher_node = Node(
        package='multi_port_motor_feedback',
        executable='can_motor_feedback_publisher',
        name='can_motor_feedback_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        arguments=['--zlg-ip', LaunchConfiguration('zlg_ip')],
    )

    return LaunchDescription([
        zlg_ip_arg,
        can_motor_feedback_publisher_node,
    ])
