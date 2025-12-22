#!/bin/bash

echo "DDS Topics Monitor"
echo "=================="

# Source ROS2 if needed
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
fi

# List all topics and filter for DDS-related ones
echo "Available Topics:"
echo "----------------"

ros2 topic list | while read topic; do
    if [[ $topic == *"lowcmd"* ]] || \
       [[ $topic == *"lowstate"* ]] || \
       [[ $topic == *"dds"* ]] || \
       [[ $topic == *"unitree"* ]] || \
       [[ $topic == *"xixi"* ]]; then
        echo "$topic"
    fi
done

echo ""
echo "Topic Details:"
echo "---------------"

# Show details for DDS topics
ros2 topic list | while read topic; do
    if [[ $topic == *"lowcmd"* ]] || \
       [[ $topic == *"lowstate"* ]] || \
       [[ $topic == *"dds"* ]] || \
       [[ $topic == *"unitree"* ]] || \
       [[ $topic == *"xixi"* ]]; then
        echo -n "$topic: "
        ros2 topic info "$topic" 2>/dev/null | grep "Type:" | cut -d' ' -f2-
    fi
done