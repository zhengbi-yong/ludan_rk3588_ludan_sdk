#!/bin/bash
# 测试使能指令通信的脚本

echo "=== ROS2 环境检查 ==="
echo "AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo ""

echo "=== 检查 /motor_enable 话题 ==="
ros2 topic info /motor_enable
echo ""

echo "=== 检查活跃的节点 ==="
ros2 node list
echo ""

echo "=== 发送测试消息 ==="
python3 /home/linaro/ludan_sdk/scripts/enable_motor.py 5 1
echo ""

echo "=== 等待2秒后检查话题 ==="
sleep 2
ros2 topic info /motor_enable
