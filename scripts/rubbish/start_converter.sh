#!/bin/bash

echo "Starting DDS to ROS2 LowCmd converter for Foxglove..."

# Set ROS2 environment
source /opt/ros/humble/setup.bash

# Start the converter
cd /home/linaro/motor_dds_proj
python3 src/dds_to_ros2_converter.py

echo "Converter stopped."