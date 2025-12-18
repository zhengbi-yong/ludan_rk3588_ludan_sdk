#!/bin/bash

# Quick Foxglove Bridge Launcher
# Simple script to start Foxglove bridge with minimal setup

# Configuration
PORT=${1:-8765}
HOST=${2:-0.0.0.0}

echo "Starting Foxglove Bridge on port $PORT..."

# Source ROS2 if not already sourced
[ -z "$ROS_DISTRO" ] && source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null

# Start Foxglove bridge
ros2 launch foxglove_bridge foxglove_bridge.launch.xml \
    port:=$PORT \
    address:=$HOST &

BRIDGE_PID=$!

echo "Foxglove Bridge started (PID: $BRIDGE_PID)"
echo "Connect Foxglove Studio to: ws://localhost:$PORT"
echo "Press Ctrl+C to stop"

wait $BRIDGE_PID