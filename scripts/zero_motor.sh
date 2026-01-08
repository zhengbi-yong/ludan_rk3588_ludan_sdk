#!/bin/bash
# DM 电机标零脚本
# 使用方法: ./zero_motor.sh <motor_id>
# 示例: ./zero_motor.sh 1-30

if [ -z "$1" ]; then
    echo "DM 电机标零脚本"
    echo ""
    echo "使用方法: $0 <motor_id> [<motor_id> ...]"
    echo ""
    echo "示例:"
    echo "  $0 1           # 标零电机 1"
    echo "  $0 1 2 3       # 标零电机 1,2,3"
    echo "  $0 1-30        # 标零电机 1-30"
    exit 1
fi

MOTOR_IDS="$@"

echo "========================================"
echo "DM 电机标零命令"
echo "========================================"
echo "目标电机: $MOTOR_IDS"
echo "标零数据帧: FF FF FF FF FF FF FF FE"
echo "========================================"
echo ""

# 发送标零命令
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --zero $MOTOR_IDS

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ 标零命令发送成功！"
else
    echo ""
    echo "✗ 标零命令发送失败！"
    exit 1
fi
