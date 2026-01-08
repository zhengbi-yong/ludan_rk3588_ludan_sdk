# Multi-Port Motor Feedback ROS2 Package

ROS2 package that receives CAN frames from ZLG device (4 ports) and publishes motor feedback to `/motor_feedback` topic.

## Features

- **4 CAN ports** (8000-8003) receiving from 30 motors
- **Motor distribution**: 10 + 7 + 7 + 6 = 30 motors
- **DM motor data parsing** with position, velocity, torque, temperature
- **ROS2 topic publishing** at 100Hz
- **MultiPortMotorFeedback message** with port_id field

## Motor Distribution

| Port | TCP Port | Motors | CAN IDs |
|------|----------|--------|---------|
| 0    | 8000     | 10     | 1-10    |
| 1    | 8001     | 7      | 11-17   |
| 2    | 8002     | 7      | 18-24   |
| 3    | 8003     | 6      | 25-30   |

## Build

```bash
# Navigate to the ludan_sdk directory
cd /home/linaro/ludan_sdk

# If you have a ROS2 workspace, copy this package to it
# Or build it in place:

# Create a symlink or copy to your ROS2 workspace
cp -r multi_port_motor_feedback_ros2 ~/your_ros2_ws/src/

# Build
cd ~/your_ros2_ws
colcon build --packages-select multi_port_motor_feedback

# Source the workspace
source install/setup.bash
```

## Usage

### Using launch file (recommended)

```bash
ros2 launch multi_port_motor_feedback motor_feedback_publisher.launch.py
```

With custom ZLG IP:

```bash
ros2 launch multi_port_motor_feedback motor_feedback_publisher.launch.py zlg_ip:=192.168.1.10
```

### Running executable directly

```bash
# Source workspace first
source install/setup.bash

# Run with default IP (192.168.1.5)
ros2 run multi_port_motor_feedback can_motor_feedback_publisher

# Run with custom IP
ros2 run multi_port_motor_feedback can_motor_feedback_publisher --zlg-ip 192.168.1.10
```

## Topics

### Published

- `/motor_feedback` (`multi_port_motor_feedback/msg/MultiPortMotorFeedback`)
  - Published at 100Hz for each motor
  - Contains: motor_id, port_id, position, velocity, torque, temperature, error, raw CAN data

## Message Definition

```
# MultiPortMotorFeedback.msg
std_msgs/Header header
int32 port_id           # Port ID (0-3 for ports 8000-8003)
int32 motor_id          # Motor ID (1-30)
int32 can_id            # Raw CAN ID (1-30)
int32 mode              # Motor mode (0=disabled, 1=enabled, 2=error)
int32 error             # Error code (0=OK)
int32 position          # Position raw
float32 position_rad    # Position in radians
int32 velocity          # Velocity raw
float32 velocity_rad_s  # Velocity in rad/s
int32 torque            # Torque raw
float32 torque_nm       # Torque in Nm
int32 temp_mos          # MOS temperature
int32 temp_rotor        # Rotor temperature
uint8[] can_data        # Raw CAN frame data (6 bytes)
```

## Example Subscriber

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from multi_port_motor_feedback.msg import MultiPortMotorFeedback

class MotorSubscriber(Node):
    def __init__(self):
        super().__init__('motor_subscriber')
        self.subscription = self.create_subscription(
            MultiPortMotorFeedback,
            '/motor_feedback',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        self.get_logger().info(
            f'Motor {msg.motor_id} (Port {msg.port_id}): '
            f'Pos={msg.position_rad:.3f}rad, '
            f'Vel={msg.velocity_rad_s:.3f}rad/s'
        )

def main():
    rclpy.init()
    motor_subscriber = MotorSubscriber()
    rclpy.spin(motor_subscriber)
    motor_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Dependencies

- ROS2 (Humble/Foxy/Galactic)
- rclcpp
- std_msgs
- ZLG CANFDNET SDK (included in `../LINUX_CANFDNET_V1.3.0.5_example`)

## Troubleshooting

### "Neither multi_port_motor_feedback nor motor_feedback message package found"

Make sure you've sourced your workspace:
```bash
source ~/your_ros2_ws/install/setup.bash
```

### Cannot connect to ZLG device

Check that:
1. ZLG device is powered on
2. Network connection is correct (ping 192.168.1.5)
3. IP address matches your device configuration

## License

Apache-2.0
