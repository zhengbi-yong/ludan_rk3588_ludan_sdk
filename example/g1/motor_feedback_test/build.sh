#!/bin/bash

# 编译 motor_feedback_reader

cd "$(dirname "$0")"

echo "========================================="
echo "    编译电机反馈读取器"
echo "========================================="

# 创建build目录
mkdir -p build
cd build

# 运行CMake
echo "[1/2] 运行CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

# 编译
echo "[2/2] 编译..."
make -j4

if [ $? -eq 0 ]; then
    echo ""
    echo "========================================="
    echo "✅ 编译成功!"
    echo "========================================="
    echo "可执行文件: ./build/motor_feedback_reader"
    echo ""
    echo "使用方法:"
    echo "  ./build/motor_feedback_reader [motor_id] [channel]"
    echo ""
    echo "示例:"
    echo "  ./build/motor_feedback_reader 4 2    # 接收电机4, CAN通道2"
    echo "  ./build/motor_feedback_reader 0 2    # 接收所有电机, CAN通道2"
else
    echo ""
    echo "❌ 编译失败!"
    exit 1
fi
