#!/usr/bin/env python3
"""
Simple DDS Topics Monitor
Listens to all DDS topics and displays their data
"""

import os
import sys
import time
import threading
from collections import defaultdict

# Add Unitree SDK to path
sys.path.insert(0, '/home/linaro/unitree_sdk2/include')
sys.path.insert(0, '/home/linaro/unitree_sdk2/thirdparty')

try:
    # Try to import Unitree SDK
    from unitree.robot.channel.channel_factory import ChannelFactory
    from unitree.robot.channel.dds_publisher import DdsPublisher
    from unitree.robot.channel.dds_subscriber import DdsSubscriber
    UNITREE_SDK_AVAILABLE = True
except ImportError as e:
    print(f"Unitree SDK not available: {e}")
    print("Will monitor using ROS2 topics instead")
    UNITREE_SDK_AVAILABLE = False

if not UNITREE_SDK_AVAILABLE:
    import rclpy
    from rclpy.node import Node

class DDSMonitor:
    def __init__(self):
        self.topic_counts = defaultdict(int)
        self.last_data = {}
        self.running = True

        if UNITREE_SDK_AVAILABLE:
            print("Using Unitree SDK for DDS monitoring")
            self.setup_unitree_sdk()
        else:
            print("Using ROS2 for topic monitoring")
            self.setup_ros2()

    def setup_unitree_sdk(self):
        """Setup Unitree SDK DDS monitoring"""
        try:
            # Initialize Unitree SDK
            ChannelFactory.Instance().Init(0, "lo")
            print("Unitree SDK initialized")

            # Known DDS topics for Unitree robots
            self.known_topics = [
                "lowcmd",
                "lowstate",
                "sport_mode",
                "bms_cmd",
                "bms_state",
                "wireless_controller",
                "imu_state"
            ]

            # Subscribe to all known topics
            for topic in self.known_topics:
                self.subscribe_topic(topic)

        except Exception as e:
            print(f"Failed to initialize Unitree SDK: {e}")
            self.setup_ros2()

    def setup_ros2(self):
        """Setup ROS2 topic monitoring as fallback"""
        rclpy.init()
        self.ros_node = rclpy.create_node('dds_monitor')
        print("ROS2 node created")

        # Get all available topics
        try:
            # Try to get topic list
            import subprocess
            result = subprocess.run(['ros2', 'topic', 'list'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                topics = result.stdout.strip().split('\n')
                print(f"Found {len(topics)} ROS2 topics")

                # Subscribe to DDS-related topics
                for topic in topics:
                    if 'lowcmd' in topic or 'dds' in topic or 'unitree' in topic:
                        self.subscribe_ros2_topic(topic)
            else:
                print("Could not get ROS2 topic list")

        except Exception as e:
            print(f"ROS2 setup error: {e}")

    def subscribe_topic(self, topic_name):
        """Subscribe to a Unitree DDS topic"""
        try:
            # Create subscriber with generic message handler
            subscriber = DdsSubscriber(topic_name, "string", self.unitree_callback)
            print(f"✓ Subscribed to DDS topic: {topic_name}")
        except Exception as e:
            print(f"✗ Failed to subscribe to {topic_name}: {e}")

    def subscribe_ros2_topic(self, topic_name):
        """Subscribe to a ROS2 topic"""
        try:
            import std_msgs
            # Use generic String message type as fallback
            self.ros_node.create_subscription(
                std_msgs.msg.String, topic_name, self.ros2_callback, 10)
            print(f"✓ Subscribed to ROS2 topic: {topic_name}")
        except Exception as e:
            print(f"✗ Failed to subscribe to ROS2 {topic_name}: {e}")

    def unitree_callback(self, topic_name, data):
        """Callback for Unitree DDS messages"""
        self.topic_counts[topic_name] += 1
        self.last_data[topic_name] = {
            'timestamp': time.time(),
            'data': str(data)[:100],  # First 100 chars
            'size': len(str(data))
        }

    def ros2_callback(self, msg):
        """Callback for ROS2 messages"""
        # Extract topic name from message or context
        topic_name = "unknown"
        if hasattr(msg, '_connection_header'):
            topic_name = msg._connection_header.get('topic', 'unknown')

        self.topic_counts[topic_name] += 1
        self.last_data[topic_name] = {
            'timestamp': time.time(),
            'data': str(msg)[:100],
            'size': len(str(msg))
        }

    def print_status(self):
        """Print monitoring status"""
        os.system('clear')
        print("=" * 60)
        print("        DDS TOPICS MONITOR")
        print("=" * 60)
        print(f"{'Topic':<30} {'Count':<10} {'Last Data':<20} {'Size'}")
        print("-" * 60)

        for topic, count in sorted(self.topic_counts.items()):
            last_info = self.last_data.get(topic, {})
            last_data = last_info.get('data', 'N/A')[:18]
            size = last_info.get('size', 0)
            age = time.time() - last_info.get('timestamp', 0)

            print(f"{topic[:28]:<30} {count:<10} {last_data:<20} {size:>4}B")

        print("\nMonitoring... Press Ctrl+C to stop")
        print(f"Total messages: {sum(self.topic_counts.values())}")

    def run(self):
        """Main monitoring loop"""
        print("Starting DDS topics monitor...")

        if not UNITREE_SDK_AVAILABLE and hasattr(self, 'ros_node'):
            # Use ROS2 spinning
            try:
                while self.running:
                    rclpy.spin_once(self.ros_node, timeout_sec=0.1)
                    time.sleep(0.1)

                    # Update display every 2 seconds
                    if int(time.time()) % 2 == 0:
                        self.print_status()

            except KeyboardInterrupt:
                print("\nStopping monitor...")
                self.running = False
            finally:
                if hasattr(self, 'ros_node'):
                    self.ros_node.destroy_node()
                rclpy.shutdown()
        else:
            # Use simple polling for Unitree SDK
            try:
                while self.running:
                    self.print_status()
                    time.sleep(2)

            except KeyboardInterrupt:
                print("\nStopping monitor...")
                self.running = False

def main():
    print("DDS Topics Monitor")
    print("=" * 30)

    monitor = DDSMonitor()

    try:
        monitor.run()
    except KeyboardInterrupt:
        print("\nMonitor stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Exiting...")

if __name__ == '__main__':
    main()