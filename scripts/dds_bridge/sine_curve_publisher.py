#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import math
import time

class SineCurvePublisher(Node):
    def __init__(self):
        super().__init__('sine_curve_publisher')

        # Create publisher for PoseStamped messages
        self.publisher_ = self.create_publisher(PoseStamped, '/target_pose', 10)

        # Timer parameters
        self.timer_period = 0.1  # 10 Hz
        self.start_time = time.time()
        self.amplitude = 2.0  # meters
        self.frequency = 0.5   # Hz

        # Create timer
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('Sine Curve Publisher has been started')
        self.get_logger().info('Publishing to topic: /target_pose')
        self.get_logger().info(f'Amplitude: {self.amplitude}m, Frequency: {self.frequency}Hz')
        self.get_logger().info(f'Publish rate: {1/self.timer_period}Hz')

    def timer_callback(self):
        # Create PoseStamped message
        msg = PoseStamped()

        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        # Calculate sine wave position
        current_time = time.time() - self.start_time

        # X position follows sine wave
        msg.pose.position.x = self.amplitude * math.sin(2 * math.pi * self.frequency * current_time)

        # Y position follows cosine wave (creates circular motion)
        msg.pose.position.y = self.amplitude * math.cos(2 * math.pi * self.frequency * current_time)

        # Z position stays constant
        msg.pose.position.z = 1.0

        # Orientation (quaternion) - no rotation
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        # Publish the message
        self.publisher_.publish(msg)

        # Log the position
        self.get_logger().debug(f'Published position: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}')

def main(args=None):
    rclpy.init(args=args)

    sine_publisher = SineCurvePublisher()

    try:
        rclpy.spin(sine_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sine_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()