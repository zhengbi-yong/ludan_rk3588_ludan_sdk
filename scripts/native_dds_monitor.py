#!/usr/bin/env python3
"""
Native DDS Topics Monitor
Monitors DDS topics directly using DDS libraries, not ROS2
"""

import sys
import os
import time
import threading
from collections import defaultdict

# Try to use CycloneDDS (most common DDS implementation)
try:
    import cyclonedds
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.topic import Topic
    from cyclonedds.sub import DataReader
    from cyclonedds.core import QoS, Event
    CYCLONEDDS_AVAILABLE = True
except ImportError:
    CYCLONEDDS_AVAILABLE = False
    print("CycloneDDS not available")

# Try FastDDS
try:
    import fastdds
    FASTDDS_AVAILABLE = True
except ImportError:
    FASTDDS_AVAILABLE = False

# Try OpenDDS
try:
    import opendds
    OPENDDS_AVAILABLE = True
except ImportError:
    OPENDDS_AVAILABLE = False

class DDSMonitor:
    def __init__(self):
        self.topic_counts = defaultdict(int)
        self.last_data = {}
        self.running = True
        self.participant = None
        self.topics_data = {}
        self.dds_domain = 0  # Unitree robots use domain ID 0

        # Set DDS environment
        os.environ['CYCLONEDDS_DOMAIN'] = str(self.dds_domain)
        os.environ['ROS_DOMAIN_ID'] = str(self.dds_domain)

        if CYCLONEDDS_AVAILABLE:
            self.setup_cyclonedds()
        elif FASTDDS_AVAILABLE:
            self.setup_fastdds()
        else:
            self.setup_shell_monitoring()

    def setup_cyclonedds(self):
        """Setup CycloneDDS monitoring"""
        try:
            from cyclonedds.domain import DomainParticipant
            # Create participant with domain ID 0
            self.participant = DomainParticipant(domain_id=self.dds_domain)
            print(f"✓ CycloneDDS participant created (domain {self.dds_domain})")

            # Start discovery thread
            self.discovery_thread = threading.Thread(target=self.discover_topics, daemon=True)
            self.discovery_thread.start()

        except Exception as e:
            print(f"✗ CycloneDDS setup failed: {e}")
            self.setup_shell_monitoring()

    def setup_fastdds(self):
        """Setup FastDDS monitoring"""
        try:
            # FastDDS setup would go here
            print("FastDDS setup not implemented")
            self.setup_shell_monitoring()
        except Exception as e:
            print(f"FastDDS setup failed: {e}")
            self.setup_shell_monitoring()

    def setup_shell_monitoring(self):
        """Fallback to shell-based DDS monitoring"""
        print("Using shell-based DDS monitoring")
        self.shell_monitor_once()

    def discover_topics(self):
        """Discover DDS topics using CycloneDDS"""
        try:
            # Known Unitree DDS topics
            unitree_topics = [
                "lowcmd",
                "lowstate",
                "rt/lowcmd",
                "rt/lowstate",
                "bms_cmd",
                "bms_state",
                "sport_mode",
                "imu_state",
                "wireless_controller",
                "xixiLowCmd::LowCmd",
                "unitree_hg::LowCmd",
                "unitree_hg::LowState"
            ]

            print(f"Trying to discover {len(unitree_topics)} known Unitree DDS topics...")

            # Try to subscribe to each known topic
            for topic_name in unitree_topics:
                try:
                    # Create topic with generic bytes type for any data
                    topic = Topic(self.participant, topic_name, "bytes")
                    reader = DataReader(self.participant, topic)

                    # Store topic info
                    self.topics_data[topic_name] = {
                        'active': True,
                        'type': 'unknown',
                        'reader': reader,
                        'last_message': None
                    }
                    print(f"✓ Subscribed to DDS topic: {topic_name}")

                    # Start message listener for this topic
                    listener_thread = threading.Thread(
                        target=self.listen_to_topic,
                        args=(topic_name, reader),
                        daemon=True
                    )
                    listener_thread.start()

                except Exception as e:
                    print(f"✗ Failed to subscribe to {topic_name}: {e}")
                    self.topics_data[topic_name] = {
                        'active': False,
                        'error': str(e)
                    }

            # Also try to discover unknown topics
            try:
                discovered_topics = self.participant.get_topics()
                print(f"Discovered {len(discovered_topics)} additional topics")

                for topic_name in discovered_topics:
                    if topic_name not in self.topics_data:
                        self.topics_data[topic_name] = {
                            'active': False,
                            'discovered': True
                        }
                        print(f"? Discovered DDS topic: {topic_name}")

            except Exception as e:
                print(f"Auto-discovery failed: {e}")

        except Exception as e:
            print(f"Topic discovery failed: {e}")

    def listen_to_topic(self, topic_name, reader):
        """Listen to messages on a specific DDS topic"""
        try:
            while self.running:
                # Try to read data from the topic
                try:
                    for sample in reader.take_iter(timeout=1.0):
                        # Update topic info when data received
                        if topic_name in self.topics_data:
                            self.topics_data[topic_name]['last_message'] = time.time()
                            self.topics_data[topic_name]['message_count'] = \
                                self.topics_data[topic_name].get('message_count', 0) + 1
                            self.topic_counts[topic_name] += 1

                        # Print sample data (first 50 bytes)
                        if hasattr(sample, 'data'):
                            data_preview = str(sample.data)[:50]
                            print(f"[{topic_name}] Received: {data_preview}...")
                        else:
                            print(f"[{topic_name}] Received message (type: {type(sample)})")

                except Exception as take_error:
                    # No data available, continue
                    continue

                time.sleep(0.01)  # Small delay to prevent CPU overload

        except Exception as e:
            print(f"Error listening to {topic_name}: {e}")

    def shell_monitor_once(self):
        """One-time shell-based DDS monitoring"""
        import subprocess

        print("Checking DDS processes and tools...")

        # Method 1: Check for DDS processes
        try:
            result = subprocess.run(["ps", "aux"], capture_output=True, text=True)
            dds_processes = []
            for line in result.stdout.split('\n'):
                if 'dds' in line.lower() or 'unitree' in line.lower():
                    if 'monitor' not in line.lower() and 'grep' not in line.lower():
                        dds_processes.append(line.strip())

            if dds_processes:
                print(f"\nFound {len(dds_processes)} DDS/Unitree processes:")
                for proc in dds_processes:
                    print(f"  {proc}")
                self.topics_data['shell_processes'] = {'active': True, 'count': len(dds_processes)}
            else:
                print("No DDS processes found")

        except Exception as e:
            print(f"Process check failed: {e}")

        # Method 2: Check network ports (DDS uses specific ports)
        try:
            result = subprocess.run(["netstat", "-tuln"], capture_output=True, text=True)
            dds_ports = []
            for line in result.stdout.split('\n'):
                if '7400' <= line.split(':')[-1].split()[0] <= '7499':
                    dds_ports.append(line.strip())

            if dds_ports:
                print(f"\nFound {len(dds_ports)} DDS network ports:")
                for port in dds_ports:
                    print(f"  {port}")
                self.topics_data['shell_ports'] = {'active': True, 'count': len(dds_ports)}

        except Exception as e:
            print(f"Port check failed: {e}")

        # Method 3: Check for DDS tools
        tools_to_try = ["ddsperf", "cycloneddsinfo", "fastddsgen", "rtiddsspy"]
        found_tools = []

        for tool in tools_to_try:
            try:
                result = subprocess.run(["which", tool], capture_output=True, text=True)
                if result.returncode == 0:
                    found_tools.append(tool)
            except:
                continue

        if found_tools:
            print(f"\nFound DDS tools: {', '.join(found_tools)}")
            self.topics_data['shell_tools'] = {'active': True, 'tools': found_tools}

        # Method 4: Check ROS2 DDS topics (if available)
        try:
            result = subprocess.run(["ros2", "topic", "list"], capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                topics = result.stdout.strip().split('\n')
                dds_topics = [t for t in topics if 'lowcmd' in t or 'lowstate' in t or 'unitree' in t]
                if dds_topics:
                    print(f"\nFound ROS2 DDS-related topics: {len(dds_topics)}")
                    for topic in dds_topics:
                        print(f"  {topic}")
                    self.topics_data['ros2_topics'] = {'active': True, 'topics': dds_topics}

        except Exception as e:
            print(f"ROS2 topic check failed: {e}")

    def print_status(self):
        """Print current monitoring status"""
        os.system('clear' if os.name == 'posix' else 'cls')
        print("=" * 70)
        print("                    NATIVE DDS TOPICS MONITOR")
        print("=" * 70)
        print()

        if CYCLONEDDS_AVAILABLE:
            print(f"DDS Library: CycloneDDS")
        elif FASTDDS_AVAILABLE:
            print(f"DDS Library: FastDDS")
        else:
            print(f"DDS Library: Shell-based monitoring")

        print(f"DDS Domain: {os.environ.get('CYCLONEDDS_DOMAIN', os.environ.get('ROS_DOMAIN_ID', '0'))}")
        print()

        print("Discovered DDS Topics:")
        print("-" * 40)

        if self.topics_data:
            for topic, info in self.topics_data.items():
                status = "Active" if info.get('active', False) else "Inactive"
                print(f"{topic:<40} {status}")
        else:
            print("No DDS topics discovered yet...")
            print()
            print("This could mean:")
            print("- No DDS applications are running")
            print("- Different DDS domain ID")
            print("- Network configuration issues")

        print()
        print("Monitoring... Press Ctrl+C to stop")
        print(f"Last update: {time.strftime('%H:%M:%S')}")

    def run(self):
        """One-time DDS topics check"""
        print("Starting one-time DDS topics check...")

        if CYCLONEDDS_AVAILABLE:
            # Give some time for topic discovery
            print("Waiting for topic discovery...")
            time.sleep(2)

        # Print results once
        self.print_final_status()

    def print_final_status(self):
        """Print final DDS topics status"""
        print("\n" + "=" * 70)
        print("                    DDS TOPICS CHECK RESULTS")
        print("=" * 70)
        print()

        if CYCLONEDDS_AVAILABLE:
            print(f"DDS Library: CycloneDDS")
        elif FASTDDS_AVAILABLE:
            print(f"DDS Library: FastDDS")
        else:
            print(f"DDS Library: Shell-based monitoring")

        print(f"DDS Domain ID: {self.dds_domain}")
        print()

        print("DDS Topics Found:")
        print("-" * 40)

        if self.topics_data:
            active_count = 0
            for topic, info in self.topics_data.items():
                if info.get('active', False):
                    status = "✓ Active"
                    active_count += 1
                    count = info.get('message_count', 0)
                    if count > 0:
                        status += f" ({count} messages)"
                else:
                    status = "✗ Inactive"
                    if 'error' in info:
                        status += f" - {info['error']}"

                print(f"{topic:<40} {status}")

            print(f"\nSummary: {active_count}/{len(self.topics_data)} topics active")

        else:
            print("No DDS topics found.")
            print("\nPossible reasons:")
            print("- No DDS applications are currently running")
            print("- Wrong DDS domain ID (current: 0)")
            print("- Network configuration issues")
            print("- DDS library not installed")

        print("\n" + "=" * 70)

def check_dds_environment():
    """Check DDS environment and libraries"""
    print("DDS Environment Check:")
    print("-" * 25)

    # Check environment variables
    env_vars = [
        'ROS_DOMAIN_ID',
        'CYCLONEDDS_DOMAIN',
        'FASTDDS_DOMAIN_ID',
        'NDDS_QOS_PROFILES'
    ]

    for var in env_vars:
        value = os.environ.get(var, 'Not set')
        print(f"{var}: {value}")

    print()

    # Check DDS library availability
    print("DDS Libraries:")
    print("-" * 15)
    print(f"CycloneDDS: {'✓' if CYCLONEDDS_AVAILABLE else '✗'}")
    print(f"FastDDS: {'✓' if FASTDDS_AVAILABLE else '✗'}")
    print(f"OpenDDS: {'✓' if OPENDDS_AVAILABLE else '✗'}")
    print()

def main():
    print("NATIVE DDS TOPICS MONITOR")
    print("=" * 30)
    print()

    check_dds_environment()

    try:
        monitor = DDSMonitor()
        monitor.run()
    except KeyboardInterrupt:
        print("\nMonitor stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Exiting DDS monitor...")

if __name__ == '__main__':
    main()