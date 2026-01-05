#!/bin/bash
# 快速启动脚本 - 完整电机控制流程

set -e

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  电机控制 - 快速启动${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ROS2 环境未加载${NC}"
    echo "请先运行: source /opt/ros/humble/setup.bash"
    exit 1
fi

# Source 工作空间
for ws in "/home/linaro/controller_ws" "/home/linaro/yunfei_ws" "/home/linaro/motor_feedback"; do
    if [ -d "$ws/install" ]; then
        source "$ws/install/setup.bash" 2>/dev/null || true
    fi
done

echo -e "${GREEN}✓ ROS2 环境: $ROS_DISTRO${NC}"
echo ""

# 停止旧进程
echo -e "${YELLOW}停止旧进程...${NC}"
pkill -f motor_controller 2>/dev/null || true
pkill -f listener_8888 2>/dev/null || true
sleep 1
echo -e "${GREEN}✓ 旧进程已停止${NC}"
echo ""

# 使能电机
echo -e "${YELLOW}[1/3] 使能电机...${NC}"
for i in 1 2 3 4 5 6; do
    /home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable $i 2>/dev/null || true
done
echo -e "${GREEN}✓ 电机已使能 (ID: 1-6)${NC}"
echo ""

# 启动 Controller
echo -e "${YELLOW}[2/3] 启动 Motor Controller...${NC}"
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable \
    --enable-motor-cmd --auto-start \
    > /tmp/motor_controller.log 2>&1 &
CONTROLLER_PID=$!
sleep 2

if ps -p $CONTROLLER_PID > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Controller PID: $CONTROLLER_PID${NC}"
else
    echo -e "${RED}❌ Controller 启动失败${NC}"
    tail -20 /tmp/motor_controller.log
    exit 1
fi

# 启动 Listener
echo -e "${YELLOW}[3/3] 启动 UDP Listener...${NC}"
cd /home/linaro/ludan_sdk/scripts
python3 listener_8888_to_ros2.py 8888 > /tmp/listener_8888.log 2>&1 &
LISTENER_PID=$!
sleep 1

if ps -p $LISTENER_PID > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Listener PID: $LISTENER_PID${NC}"
else
    echo -e "${RED}❌ Listener 启动失败${NC}"
    tail -20 /tmp/listener_8888.log
    kill $CONTROLLER_PID 2>/dev/null || true
    exit 1
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  启动完成！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "进程信息:"
echo "  Controller: PID $CONTROLLER_PID (日志: /tmp/motor_controller.log)"
echo "  Listener:   PID $LISTENER_PID (日志: /tmp/listener_8888.log)"
echo ""
echo -e "${YELLOW}现在可以发送控制命令：${NC}"
echo ""
echo "  # 发送位置=1 的命令"
echo "  cd /home/linaro/ludan_sdk/example/g1/motor_test10"
echo "  python3 send_position_1.py --continuous 10"
echo ""
echo "  # 或运行完整测试"
echo "  ./test_all_ports.sh"
echo ""
echo -e "${YELLOW}停止所有进程：${NC}"
echo "  kill $CONTROLLER_PID $LISTENER_PID"
echo ""
