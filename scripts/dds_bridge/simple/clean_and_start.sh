#!/bin/bash

# 清理并重新启动Foxglove兼容桥接器
# 确保没有冲突的进程

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

echo -e "${BLUE}🧹 清理并重新启动Foxglove兼容桥接器${NC}"
echo -e "${BLUE}=======================================${NC}"

# 1. 停止所有相关进程
print_status "INFO" "停止所有相关进程..."

# 停止各种可能的进程
pkill -f "xixi_lowcmd_forwarder" 2>/dev/null || true
pkill -f "foxglove_compatible_bridge" 2>/dev/null || true
pkill -f "foxglove_bridge" 2>/dev/null || true
pkill -f "start_foxglove_bridge" 2>/dev/null || true

# 等待进程完全停止
sleep 2

# 2. 清理临时文件
print_status "INFO" "清理临时文件..."
rm -f /tmp/xixi_lowcmd_forwarder.py 2>/dev/null || true
rm -rf /tmp/xixi_lowcmd_ros2 2>/dev/null || true

# 3. 检查进程是否完全停止
print_status "INFO" "检查进程状态..."
if pgrep -f "xixi_lowcmd\|foxglove" > /dev/null; then
    print_status "WARN" "仍有进程运行，强制停止..."
    pkill -9 -f "xixi_lowcmd\|foxglove" 2>/dev/null || true
    sleep 1
fi

# 4. 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    print_status "INFO" "加载ROS2环境..."
    source /opt/ros/humble/setup.bash
fi

# 5. 设置域ID
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID

print_status "INFO" "环境设置完成:"
echo -e "  ROS域ID: ${BLUE}$ROS_DOMAIN_ID${NC}"
echo -e "  ROS发行版: ${BLUE}$ROS_DISTRO${NC}"

# 6. 启动新的桥接器
print_status "INFO" "启动Foxglove兼容桥接器..."
cd /home/linaro/unitree_sdk2/scripts/dds_bridge/simple

python3 foxglove_compatible_bridge.py &
BRIDGE_PID=$!

print_status "INFO" "桥接器PID: $BRIDGE_PID"

# 等待桥接器启动
sleep 3

# 7. 启动Foxglove Bridge
print_status "INFO" "启动Foxglove WebSocket Bridge..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    port:=8765 \
    address:=0.0.0.0 \
    log_level:=warn \
    &
FOXGLOVE_PID=$!

print_status "INFO" "Foxglove Bridge PID: $FOXGLOVE_PID"

# 等待Foxglove Bridge启动
sleep 3

# 8. 验证话题
print_status "INFO" "验证发布的话题..."
echo
for topic in "/lowcmd_joint_states" "/lowcmd_raw_bytes" "/lowcmd_control_info"; do
    if ros2 topic list | grep -q "$topic"; then
        echo -e "  ${GREEN}✓${NC} $topic"

        # 显示话题类型
        topic_type=$(ros2 topic info $topic 2>/dev/null | grep "Type:" | cut -d' ' -f2 || echo "Unknown")
        echo -e "    类型: ${BLUE}$topic_type${NC}"
    else
        echo -e "  ${YELLOW}⚠${NC} $topic (等待发布中)"
    fi
done

echo
print_status "INFO" "检查是否有冲突的xixi话题..."
conflicting_topics=$(ros2 topic list 2>/dev/null | grep -E "(lowcmd$|joint_commands$)" || echo "")
if [ -n "$conflicting_topics" ]; then
    print_status "WARN" "发现可能冲突的话题:"
    echo "$conflicting_topics" | while read topic; do
        topic_type=$(ros2 topic info $topic 2>/dev/null | grep "Type:" | cut -d' ' -f2 || echo "Unknown")
        if [[ "$topic_type" == *"xixi_lowcmd_ros2"* ]]; then
            echo -e "  ${RED}✗${NC} $topic ($topic_type) - 需要停止"
        else
            echo -e "  ${GREEN}✓${NC} $topic ($topic_type) - 正常"
        fi
    done
else
    print_status "INFO" "✓ 没有发现冲突的话题"
fi

echo
echo -e "${GREEN}🎯 清理和启动完成!${NC}"
echo
echo -e "${GREEN}📡 可用话题:${NC}"
echo -e "  ${YELLOW}/lowcmd_joint_states${NC}    - 关节状态 (Foxglove原生支持)"
echo -e "  ${YELLOW}/lowcmd_raw_bytes${NC}        - 原始数据"
echo -e "  ${YELLOW}/lowcmd_control_info${NC}     - 控制信息"
echo
echo -e "${GREEN}🌐 Foxglove连接:${NC}"
echo -e "  WebSocket: ${BLUE}ws://localhost:8765${NC}"
echo -e "  或打开: ${BLUE}https://studio.foxglove.dev/${NC}"
echo
echo -e "${GREEN}🔧 调试命令:${NC}"
echo -e "  ros2 topic echo ${YELLOW}/lowcmd_joint_states${NC}"
echo -e "  ros2 topic hz ${YELLOW}/lowcmd_joint_states${NC}"
echo -e "\n按 ${YELLOW}Ctrl+C${NC} 停止所有服务"

# 清理函数
cleanup() {
    print_status "INFO" "停止所有服务..."

    # 停止桥接器
    if [ ! -z "$BRIDGE_PID" ]; then
        kill $BRIDGE_PID 2>/dev/null || true
        print_status "INFO" "✓ 桥接器已停止"
    fi

    # 停止Foxglove Bridge
    if [ ! -z "$FOXGLOVE_PID" ]; then
        kill $FOXGLOVE_PID 2>/dev/null || true
        print_status "INFO" "✓ Foxglove Bridge已停止"
    fi

    # 强制清理任何残留进程
    pkill -f "foxglove_compatible_bridge" 2>/dev/null || true
    pkill -f "foxglove_bridge" 2>/dev/null || true

    print_status "INFO" "👋 所有服务已停止"
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

# 等待进程
wait