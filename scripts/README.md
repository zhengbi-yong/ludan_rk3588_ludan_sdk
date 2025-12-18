# Unitree SDK2 Scripts

This directory contains organized scripts for Unitree SDK2 development and deployment. The scripts have been reorganized into logical categories while maintaining full backward compatibility and functionality.

## Directory Structure

```
scripts/
├── README.md                 # This file
├── launcher.sh              # Main launcher for easy access to all scripts
├── lowcmd/                  # Low-level command processing
├── dds_bridge/              # DDS communication and CAN bridge
├── deployment/              # Deployment and configuration
├── testing/                 # Testing and verification utilities
└── ros_integration/         # ROS1 integration and bridge scripts
```

## Quick Start

### Using the Launcher (Recommended)

```bash
# Show help and available options
./launcher.sh help

# List all available scripts
./launcher.sh list

# Start LowCmd publisher (for /lowcmd processing)
./launcher.sh lowcmd start_jetson_lowcmd.sh

# Test LowCmd format
./launcher.sh lowcmd test_lowcmd_format.py

# Show DDS bridge options
./launcher.sh dds_bridge
```

### Direct Script Execution

All scripts can still be executed directly from their subdirectories:

```bash
# LowCmd processing
./lowcmd/start_jetson_lowcmd.sh
./lowcmd/jetson_lowcmd_publisher.py
./lowcmd/test_lowcmd_format.py

# DDS bridge utilities
./dds_bridge/can_monitor.sh
./dds_bridge/can_quick_test.sh

# Testing
./testing/test_500hz_system.sh
./testing/verify_500hz_simple.sh
```

## Categories

### lowcmd/ - Low-level Command Processing

Scripts for processing and publishing `/lowcmd` messages for robot joint control.

**Key Scripts:**
- `start_jetson_lowcmd.sh` - Main launcher for Jetson LowCmd publisher
- `jetson_lowcmd_publisher.py` - Python ROS1 publisher for `/lowcmd` topic
- `test_lowcmd_format.py` - Test LowCmd data format validation
- `python_lowcmd_builder.py` - LowCmd message builder utility
- `lowcmd_data_example.json` - Example LowCmd data format
- `lowcmd_format_examples.md` - Documentation for LowCmd format

**Usage Example:**
```bash
# Start LowCmd publisher with default settings
./launcher.sh lowcmd start_jetson_lowcmd.sh

# Or run directly
./lowcmd/start_jetson_lowcmd.sh --rate 100 --amplitude 0.5
```

### dds_bridge/ - DDS Communication and CAN Bridge

Scripts for DDS middleware and CAN bus communication.

**Key Scripts:**
- `can_monitor.sh` - CAN bus monitoring utility
- `can_quick_test.sh` - Quick CAN functionality test
- `can_frequency_test.sh` - CAN frequency testing
- `sine_curve_publisher.py` - Sine wave trajectory publisher
- `fox.sh` - Foxglove visualization setup
- `foxglove_launcher.sh` - Foxglove visualization launcher
- `quick_foxglove.sh` - Quick Foxglove setup

### deployment/ - Deployment and Configuration

Scripts for system deployment and configuration.

**Key Scripts:**
- `deploy_test.sh` - General deployment testing
- `start_deploy_jetson.sh` - Jetson deployment starter
- `start_deploy_rk3588.sh` - RK3588 deployment starter
- `deploy_test_rk3588_receiver.sh` - RK3588 receiver deployment
- `JETSON_SETUP_GUIDE.md` - Jetson setup documentation

### testing/ - Testing and Verification

Scripts for system testing and verification.

**Key Scripts:**
- `test_500hz_system.sh` - 500Hz system performance testing
- `verify_500hz_simple.sh` - Simple 500Hz verification
- `test_ros2_deploy.sh` - ROS2 deployment testing
- `test_network_connectivity.sh` - Network connectivity testing
- `test_jetson_publisher.py` - Jetson publisher testing

### ros_integration/ - ROS1 Integration

Scripts for ROS1 integration and bridge functionality.

**Key Scripts:**
- `ros1_to_dds_bridge.py` - ROS1 to DDS bridge
- `deploy_ros1_to_dds.sh` - ROS1-DDS deployment
- `ros1_lowcmd_bridge_example.py` - ROS1 LowCmd bridge example
- `test_ros1_dds_communication.py` - ROS1-DDS communication test
- `README_ROS1_DDS_BRIDGE.md` - ROS1-DDS bridge documentation

## LowCmd Processing (/lowcmd topic)

The `/lowcmd` processing functionality is fully preserved and enhanced:

### Core Components

1. **Publisher Scripts** (`lowcmd/`):
   - Publish joint control commands to `/lowcmd` topic
   - Support for sine wave trajectories
   - Network and DDS bridge integration
   - Real-time performance optimization

2. **Message Builder** (`python_lowcmd_builder.py`):
   - Python utilities for building LowCmd messages
   - Support for G1/H1 robots (35 motors)
   - Binary packing and JSON conversion
   - CRC validation

3. **Format Validation** (`test_lowcmd_format.py`):
   - Comprehensive testing of LowCmd data format
   - Validation of motor commands and parameters
   - JSON and binary format verification

### Usage Examples

```bash
# Start LowCmd publisher with sine wave control
./launcher.sh lowcmd start_jetson_lowcmd.sh

# Test the data format
./launcher.sh lowcmd test_lowcmd_format.py

# Run message builder demo
./launcher.sh lowcmd python_lowcmd_builder.py

# Custom LowCmd publishing
./launcher.sh lowcmd jetson_lowcmd_publisher.py \
    --rk3588_ip 192.168.1.20 \
    --rate 50 \
    --amplitude 0.3 \
    --joints [4,5,10,11]
```

## Backward Compatibility

All existing functionality is preserved:
- Scripts can be executed directly from their new locations
- Internal script references are updated automatically
- All command-line arguments and options remain unchanged
- Configuration files and documentation are preserved

## Migration Guide

### For Existing Users

If you were previously running scripts like:
```bash
# Old way (still works from new location)
./scripts/jetson_lowcmd_publisher.py
./scripts/start_jetson_lowcmd.sh
```

You can now use:
```bash
# New recommended way
./scripts/launcher.sh lowcmd jetson_lowcmd_publisher.py
./scripts/launcher.sh lowcmd start_jetson_lowcmd.sh

# Or direct execution (still works)
./scripts/lowcmd/jetson_lowcmd_publisher.py
./scripts/lowcmd/start_jetson_lowcmd.sh
```

### For Development

When adding new scripts:
1. Place them in the appropriate category directory
2. Make them executable (`chmod +x`)
3. Update this README if needed
4. Test with the launcher: `./launcher.sh <category> <script>`

## Troubleshooting

### Script Not Found
```bash
# Check available scripts in category
./launcher.sh lowcmd

# List all scripts
./launcher.sh list
```

### Permission Denied
```bash
# Make scripts executable
chmod +x scripts/launcher.sh
chmod +x scripts/*/*.sh
chmod +x scripts/*/*.py
```

### Dependencies
Ensure you have:
- ROS1 (Noetic) environment sourced
- Python3 with required packages
- Unitree SDK2 built and installed
- Proper network configuration for multi-machine setups

## Support

For issues with specific scripts:
1. Check the script's help/usage information
2. Consult the documentation in the respective category
3. Verify your environment setup
4. Test with known working configurations

The reorganization maintains all original functionality while providing better organization and easier access to the tools you need.