#!/bin/bash

# Foxglove兼容的xixiLowCmd桥接启动脚本
# 使用标准ROS2消息类型，避免临时包问题

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_status() {
    local status=$1
    local message=$2
    case $status in
        "INFO")  echo -e "${GREEN}[INFO]${NC} $message" ;;
        "WARN")  echo -e "${YELLOW}[WARN]${NC} $message" ;;
        "ERROR") echo -e "${RED}[ERROR]${NC} $message" ;;
    esac
}

# 配置
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
FOXGLOVE_PORT=${FOXGLOVE_PORT:-8765}
FOXGLOVE_HOST=${FOXGLOVE_HOST:-"0.0.0.0"}

print_status "INFO" "启动Foxglove兼容的xixiLowCmd桥接..."
echo -e "  ROS域ID: ${BLUE}$ROS_DOMAIN_ID${NC}"
echo -e "  Foxglove端口: ${BLUE}$FOXGLOVE_PORT${NC}"

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    print_status "INFO" "加载ROS2环境..."
    source /opt/ros/humble/setup.bash
fi

# 检查是否有/lowcmd话题
print_status "INFO" "检查DDS话题..."
if ros2 topic list | grep -q "/lowcmd"; then
    print_status "INFO" "✓ 找到/lowcmd话题"
else
    print_status "WARN" "⚠ 未找到/lowcmd话题，桥接器将等待数据..."
fi

# 启动数据桥接器
print_status "INFO" "启动数据桥接器..."
cd /home/linaro/unitree_sdk2/scripts/dds_bridge/simple

export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
python3 foxglove_compatible_bridge.py &
BRIDGE_PID=$!
echo "桥接器PID: $BRIDGE_PID"

# 等待桥接器启动
sleep 3

# 检查话题是否正常发布
print_status "INFO" "检查发布的话题..."
for topic in "/lowcmd_joint_states" "/lowcmd_raw_bytes" "/lowcmd_control_info"; do
    if ros2 topic list | grep -q "$topic"; then
        print_status "INFO" "✓ 话题 $topic 正常发布"
    else
        print_status "WARN" "⚠ 话题 $topic 尚未发布"
    fi
done

# 检查Foxglove bridge
if command -v ros2 &> /dev/null; then
    if ros2 pkg list | grep -q "foxglove_bridge"; then
        print_status "INFO" "启动Foxglove Bridge..."
        ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
            port:=$FOXGLOVE_PORT \
            address:=$FOXGLOVE_HOST \
            &
        FOXGLOVE_PID=$!
        echo "Foxglove Bridge PID: $FOXGLOVE_PID"
    else
        print_status "WARN" "⚠ Foxglove Bridge包未安装，请手动安装："
        echo "sudo apt install ros-\$ROS_DISTRO-foxglove-bridge"
    fi
fi

# 显示使用说明
echo -e "\n${BLUE}🎯 Foxglove兼容桥接已启动!${NC}"
echo -e "\n${GREEN}📡 可用话题:${NC}"
echo -e "  ${YELLOW}/lowcmd_joint_states${NC}     - 关节状态 (Foxglove原生支持)"
echo -e "  ${YELLOW}/lowcmd_raw_bytes${NC}        - 原始数据字节"
echo -e "  ${YELLOW}/lowcmd_control_info${NC}     - 控制信息"
echo -e "\n${GREEN}🌐 Foxglove连接:${NC}"
echo -e "  WebSocket: ${BLUE}ws://$FOXGLOVE_HOST:$FOXGLOVE_PORT${NC}"
echo -e "  或打开: ${BLUE}https://studio.foxglove.dev/${NC} 并连接到上述地址"
echo -e "\n${GREEN}📋 在Foxglove中:${NC}"
echo -e "  1. 使用 '${YELLOW}Joint State${NC}' 面板查看关节状态"
echo -e "  2. 使用 '${YELLOW}Raw Messages${NC}' 面板查看原始数据"
echo -e "\n${GREEN}🔧 调试命令:${NC}"
echo -e "  ros2 topic echo ${YELLOW}/lowcmd_joint_states${NC}"
echo -e "  ros2 topic hz ${YELLOW}/lowcmd_joint_states${NC}"
echo -e "\n按 ${YELLOW}Ctrl+C${NC} 停止所有服务"

# 清理函数
cleanup() {
    print_status "INFO" "停止所有服务..."

    if [ ! -z "$BRIDGE_PID" ]; then
        kill $BRIDGE_PID 2>/dev/null || true
        print_status "INFO" "✓ 数据桥接器已停止"
    fi

    if [ ! -z "$FOXGLOVE_PID" ]; then
        kill $FOXGLOVE_PID 2>/dev/null || true
        print_status "INFO" "✓ Foxglove Bridge已停止"
    fi

    # 杀死可能残留的进程
    pkill -f "foxglove_bridge" 2>/dev/null || true
    pkill -f "foxglove_compatible_bridge" 2>/dev/null || true

    print_status "INFO" "👋 所有服务已停止"
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

# 等待进程
wait