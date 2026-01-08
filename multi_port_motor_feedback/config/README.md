# CAN Send Test - JSON Configuration Guide

## Overview
The `can_send_test` program sends controlled CAN frames to 4 ports (8000-8003) based on JSON configuration files.

## Motor Distribution
- **Port 8000**: Motors 1-10 (10 motors)
- **Port 8001**: Motors 11-17 (7 motors)
- **Port 8002**: Motors 18-24 (7 motors)
- **Port 8003**: Motors 25-30 (6 motors)
- **Total**: 30 motors

---

## Complete Test Flow (Shared Memory Mode)

### Terminal 1: Start Data Sender
```bash
cd /home/linaro/ludan_sdk/multi_port_motor_feedback/build
./can_send_test --config /home/linaro/ludan_sdk/multi_port_motor_feedback/config/motor_test_config_full.json
```

### Terminal 2: Start ROS2 Publisher (Test Mode)
```bash
source /home/linaro/ludan_sdk/multi_port_motor_feedback/multi_port_motor_feedback_ros2/install/setup.bash
ros2 run multi_port_motor_feedback can_motor_feedback_publisher --test-mode
```

### Terminal 3: Start Subscriber (Optional)
```bash
cd /home/linaro/motor_feedback_api/motor_feedback_api
python3 motor_subscriber.py
```

### Terminal 4: Monitor Topic (Optional)
```bash
source /home/linaro/ludan_sdk/multi_port_motor_feedback/multi_port_motor_feedback_ros2/install/setup.bash
ros2 topic echo /motor_feedback
```

---

## Data Flow

```
can_send_test ──→ Shared Memory (/can_motor_test_shm) ──→ can_motor_feedback_publisher --test-mode ──→ ROS2 /motor_feedback ──→ motor_subscriber.py
     (Process 1)                                                              (Process 2)                                   (Process 3)
```

---

## Quick Start

1. **Build** (if not already built):
```bash
cd /home/linaro/ludan_sdk/multi_port_motor_feedback/build
cmake ..
make can_send_test
```

2. **Start the test**:
   - Terminal 1: `./can_send_test --config ../config/motor_test_config_full.json`
   - Terminal 2: `ros2 run multi_port_motor_feedback can_motor_feedback_publisher --test-mode`

3. **Verify data**:
   - Terminal 3: `ros2 topic echo /motor_feedback`
   - Or: `python3 /home/linaro/motor_feedback_api/motor_feedback_api/motor_subscriber.py`

---

## Usage

### Basic Usage (without config file)
```bash
./can_send_test --ip 192.168.1.5 --interval 100
```

### With JSON Configuration (Shared Memory Mode)
```bash
./can_send_test --config config/motor_test_config_full.json
```

### Command Line Options
| Option | Description | Default |
|--------|-------------|---------|
| `--config <file>` | JSON configuration file | none |
| `--interval <n>` | Send interval in ms | 100 (or from config) |
| `--quiet` | Don't print each frame | false |
| `-h, --help` | Show help message | - |

## JSON Configuration Format

### Global Settings
```json
{
  "send_interval_ms": 100,    // Send interval in milliseconds
  "verbose": true,             // Print frame data
  "p_max": 6.283,             // Position max range (2*PI radians)
  "v_max": 45.0,              // Velocity max range (rad/s)
  "t_max": 18.0,              // Torque max range (Nm)
  "motors": [...]              // Array of motor configurations
}
```

### Motor Configuration
Each motor in the `motors` array can have the following properties:

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `id` | integer | required | Motor ID (1-30) |
| `enabled` | boolean | true | Enable/disable this motor |
| `position` | float | 0.0 | Position (radians) |
| `velocity` | float | 0.0 | Velocity (rad/s) |
| `torque` | float | 0.0 | Torque (Nm) |
| `temp_mos` | float | 25.0 | MOS temperature (°C, 0-125) |
| `temp_rotor` | float | 25.0 | Rotor temperature (°C, 0-125) |
| `error_code` | integer | 1 | Error state code |
| `data_mode` | integer | 0 | Data generation mode (0-3) |
| `pos_increment` | float | 0.0 | Position increment per frame (mode 1,3) |
| `vel_amplitude` | float | 0.0 | Velocity amplitude for sine (mode 2,3) |
| `vel_frequency` | float | 0.0 | Velocity frequency in Hz (mode 2,3) |
| `p_max` | float | 6.283 | Position range override |
| `v_max` | float | 45.0 | Velocity range override |
| `t_max` | float | 18.0 | Torque range override |

### Error Codes
| Code | Hex | Description |
|------|-----|-------------|
| 0 | 0x0 | 失能 (Disabled) |
| 1 | 0x1 | 使能 (Enabled) |
| 8 | 0x8 | 超压 (Overvoltage) |
| 9 | 0x9 | 欠压 (Undervoltage) |
| 10 | 0xA | 过电流 (Overcurrent) |
| 11 | 0xB | MOS过温 (MOS over temperature) |
| 12 | 0xC | 线圈过温 (Coil over temperature) |
| 13 | 0xD | 通讯丢失 (Communication lost) |
| 14 | 0xE | 过载 (Overload) |

### Data Modes

#### Mode 0: Fixed Value (default)
Motor values remain constant at configured values.
```json
{
  "id": 1,
  "enabled": true,
  "position": 0.0,
  "velocity": 0.0,
  "torque": 0.0,
  "data_mode": 0
}
```

#### Mode 1: Increment Position
Position increases by `pos_increment` each frame.
```json
{
  "id": 2,
  "position": 0.0,
  "data_mode": 1,
  "pos_increment": 0.001
}
```

#### Mode 2: Sine Wave Velocity
Velocity follows a sine wave, position is the integral.
```json
{
  "id": 3,
  "position": 0.0,
  "data_mode": 2,
  "vel_amplitude": 25.0,
  "vel_frequency": 0.5
}
```

#### Mode 3: Custom (Combination)
Combines increment and sine wave.
```json
{
  "id": 4,
  "position": 0.0,
  "data_mode": 3,
  "pos_increment": 0.0005,
  "vel_amplitude": 1.0,
  "vel_frequency": 0.5
}
```

## CAN Frame Format

Each CAN frame (8 bytes) contains:
| Byte | Content | Description |
|------|---------|-------------|
| D[0] | ID \| ERR<<4 | Motor ID (low 4 bits) + Error (high 4 bits) |
| D[1] | POS[15:8] | Position high byte (16-bit signed) |
| D[2] | POS[7:0] | Position low byte |
| D[3] | VEL[11:4] | Velocity high bits (12-bit signed) |
| D[4] | VEL[3:0]\|T[11:8] | Velocity low + Torque high |
| D[5] | T[7:0] | Torque low byte (12-bit signed) |
| D[6] | T_MOS | MOS temperature (0-125°C) |
| D[7] | T_Rotor | Rotor temperature (0-125°C) |

### Data Mapping
- **Position**: 16-bit signed, maps -p_max to +p_max → -32767 to +32767
- **Velocity**: 12-bit signed, maps -v_max to +v_max → -2047 to +2047
- **Torque**: 12-bit signed, maps -t_max to +t_max → -2047 to +2047

### Example Calculation
For velocity with v_max=45.0 rad/s:
```
V = 25.0 rad/s
VEL = 25.0 / 45.0 * 2047 + 2048 = 3185 = 0xC71
```

## Example Configurations

### Minimal Config (Single Motor)
```json
{
  "send_interval_ms": 100,
  "motors": [
    {
      "id": 1,
      "enabled": true
    }
  ]
}
```

### Test Different Modes
```json
{
  "motors": [
    {"id": 1, "data_mode": 0, "position": 0.0},
    {"id": 2, "data_mode": 1, "pos_increment": 0.001},
    {"id": 3, "data_mode": 2, "vel_amplitude": 25.0, "vel_frequency": 0.5},
    {"id": 4, "enabled": false}
  ]
}
```

### Error State Simulation
```json
{
  "motors": [
    {"id": 1, "error_code": 0, "enabled": false},
    {"id": 2, "error_code": 11, "temp_mos": 85.0},
    {"id": 3, "error_code": 14}
  ]
}
```

## Building
```bash
cd /home/linaro/ludan_sdk/multi_port_motor_feedback
mkdir -p build && cd build
cmake ..
make can_send_test
```

## Configuration Files
- `config/motor_test_config.json` - Basic test configuration (8 motors)
- `config/motor_test_config_full.json` - Full 30 motor configuration
