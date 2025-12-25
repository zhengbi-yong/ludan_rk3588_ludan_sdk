#!/bin/bash
# Motor Feedback Package Build Script

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="/home/linaro/controller_ws"
PKG_NAME="motor_feedback"

echo "========================================"
echo "  Motor Feedback Package Build Script"
echo "========================================"

# Check if ROS2 workspace exists
if [ ! -d "$ROS2_WS" ]; then
    echo "ERROR: ROS2 workspace not found at $ROS2_WS"
    echo "Please create the workspace first or modify ROS2_WS in this script"
    exit 1
fi

# Check if src directory exists
if [ ! -d "$ROS2_WS/src" ]; then
    echo "Creating src directory in ROS2 workspace..."
    mkdir -p "$ROS2_WS/src"
fi

# Create symbolic link if it doesn't exist
LINK_PATH="$ROS2_WS/src/$PKG_NAME"
if [ ! -e "$LINK_PATH" ]; then
    echo "Creating symbolic link: $LINK_PATH -> $SCRIPT_DIR"
    ln -s "$SCRIPT_DIR" "$LINK_PATH"
else
    echo "Symbolic link already exists: $LINK_PATH"
fi

# Build the package
echo ""
echo "Building $PKG_NAME..."
cd "$ROS2_WS"
colcon build --packages-select $PKG_NAME --symlink-install

# Source the workspace
echo ""
echo "Sourcing workspace..."
source install/setup.bash

echo ""
echo "========================================"
echo "  Build Complete!"
echo "========================================"
echo ""
echo "To use the package:"
echo "  source $ROS2_WS/install/setup.bash"
echo ""
echo "Launch commands:"
echo "  ros2 launch $PKG_NAME motor_feedback.launch.py"
echo "  ros2 run $PKG_NAME motor_feedback_publisher"
echo "  ros2 run $PKG_NAME motor_feedback_subscriber"
echo ""
