#!/bin/bash
# Compatibility script to create symlinks for backward compatibility
# This creates symlinks in the scripts root pointing to reorganized scripts

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}Creating compatibility symlinks...${NC}"

# Function to create symlink if target exists
create_symlink() {
    local target="$1"
    local link="$2"

    if [ -f "$target" ]; then
        if [ -L "$link" ]; then
            echo -e "${YELLOW}Symlink already exists: $link${NC}"
        elif [ -f "$link" ]; then
            echo -e "${RED}File exists and is not a symlink: $link${NC}"
        else
            ln -s "$target" "$link"
            echo -e "${GREEN}Created: $link -> $target${NC}"
        fi
    else
        echo -e "${RED}Target not found: $target${NC}"
    fi
}

# Create symlinks for commonly used scripts
echo "Creating symlinks for LowCmd scripts..."
create_symlink "lowcmd/jetson_lowcmd_publisher.py" "jetson_lowcmd_publisher.py"
create_symlink "lowcmd/start_jetson_lowcmd.sh" "start_jetson_lowcmd.sh"
create_symlink "lowcmd/test_lowcmd_format.py" "test_lowcmd_format.py"
create_symlink "lowcmd/python_lowcmd_builder.py" "python_lowcmd_builder.py"

echo "Creating symlinks for DDS Bridge scripts..."
create_symlink "dds_bridge/can_monitor.sh" "can_monitor.sh"
create_symlink "dds_bridge/can_quick_test.sh" "can_quick_test.sh"
create_symlink "dds_bridge/can_frequency_test.sh" "can_frequency_test.sh"
create_symlink "dds_bridge/sine_curve_publisher.py" "sine_curve_publisher.py"

echo "Creating symlinks for Deployment scripts..."
create_symlink "deployment/deploy_test.sh" "deploy_test.sh"
create_symlink "deployment/start_deploy_jetson.sh" "start_deploy_jetson.sh"
create_symlink "deployment/start_deploy_rk3588.sh" "start_deploy_rk3588.sh"

echo "Creating symlinks for Testing scripts..."
create_symlink "testing/test_500hz_system.sh" "test_500hz_system.sh"
create_symlink "testing/verify_500hz_simple.sh" "verify_500hz_simple.sh"
create_symlink "testing/test_ros2_deploy.sh" "test_ros2_deploy.sh"

echo "Creating symlinks for Foxglove scripts..."
create_symlink "dds_bridge/fox.sh" "fox.sh"
create_symlink "dds_bridge/foxglove_launcher.sh" "foxglove_launcher.sh"
create_symlink "dds_bridge/quick_foxglove.sh" "quick_foxglove.sh"

echo "Creating symlinks for ROS Integration scripts..."
create_symlink "ros_integration/ros1_to_dds_bridge.py" "ros1_to_dds_bridge.py"
create_symlink "ros_integration/deploy_ros1_to_dds.sh" "deploy_ros1_to_dds.sh"

echo ""
echo -e "${GREEN}Compatibility symlinks created successfully!${NC}"
echo ""
echo "You can now use both old and new ways to run scripts:"
echo ""
echo "  Old way (still works via symlinks):"
echo "    ./scripts/jetson_lowcmd_publisher.py"
echo "    ./scripts/can_monitor.sh"
echo ""
echo "  New organized way:"
echo "    ./scripts/launcher.sh lowcmd jetson_lowcmd_publisher.py"
echo "    ./scripts/dds_bridge/can_monitor.sh"
echo ""
echo "To remove symlinks later, run:"
echo "  find $SCRIPT_DIR -maxdepth 1 -type l -delete"