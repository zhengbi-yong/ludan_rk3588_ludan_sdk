#!/bin/bash
# 完整数据流测试脚本
# 测试 UDP → listener → ROS2 → motor_controller → CAN

echo "========================================"
echo "  Complete Data Flow Test"
echo "========================================"
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ROS2 环境未加载${NC}"
    echo "请先运行: source /opt/ros/humble/setup.bash"
    exit 1
fi

# Source 工作空间
for ws in "/home/linaro/yunfei_ws" "/home/linaro/motor_feedback" "/home/linaro/controller_ws"; do
    if [ -d "$ws/install" ]; then
        source "$ws/install/setup.bash" 2>/dev/null
    fi
done

echo -e "${GREEN}✓ ROS2 环境: $ROS_DISTRO${NC}"
echo ""

# ============================================================
# 步骤 1: 启动 motor_controller (后台)
# ============================================================
echo -e "${BLUE}[1/4] 启动 Motor Controller...${NC}"
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable \
    --enable-motor-cmd \
    --auto-start \
    > /tmp/motor_controller.log 2>&1 &

CONTROLLER_PID=$!
sleep 2

if ps -p $CONTROLLER_PID > /dev/null; then
    echo -e "${GREEN}✓ Controller PID: $CONTROLLER_PID${NC}"
    tail -5 /tmp/motor_controller.log
else
    echo -e "${RED}❌ Controller 启动失败${NC}"
    cat /tmp/motor_controller.log
    exit 1
fi
echo ""

# ============================================================
# 步骤 2: 启动 listener (后台)
# ============================================================
echo -e "${BLUE}[2/4] 启动 UDP Listener...${NC}"
cd /home/linaro/ludan_sdk/scripts
python3 listener_8888_to_ros2.py 8888 \
    > /tmp/listener.log 2>&1 &

LISTENER_PID=$!
sleep 1

if ps -p $LISTENER_PID > /dev/null; then
    echo -e "${GREEN}✓ Listener PID: $LISTENER_PID${NC}"
    tail -3 /tmp/listener.log
else
    echo -e "${RED}❌ Listener 启动失败${NC}"
    cat /tmp/listener.log
    kill $CONTROLLER_PID 2>/dev/null
    exit 1
fi
echo ""

# ============================================================
# 步骤 3: 发送 UDP 测试数据
# ============================================================
echo -e "${BLUE}[3/4] 发送 UDP 测试数据...${NC}"
cd /home/linaro/ludan_sdk/example/g1/motor_test2
python3 udp_test_sender.py --single

echo ""
echo -e "${GREEN}✓ UDP 数据包已发送${NC}"
echo ""

# ============================================================
# 步骤 4: 检查数据流状态
# ============================================================
echo -e "${BLUE}[4/4] 检查数据流状态...${NC}"
echo ""

sleep 1

echo "--- Controller 日志 (最后 10 行) ---"
tail -10 /tmp/motor_controller.log | grep -E "ROS2_msg|Send|Success|Error"
echo ""

echo "--- Listener 日志 (最后 10 行) ---"
tail -10 /tmp/listener.log | grep -E "JSON|motor_cmd|发布|Motor"
echo ""

# ============================================================
# 清理
# ============================================================
echo -e "${YELLOW}========================================"
echo "  测试完成！"
echo "========================================${NC}"
echo ""
echo "进程状态:"
ps -p $CONTROLLER_PID > /dev/null && echo -e "  Controller: ${GREEN}运行中${NC} (PID: $CONTROLLER_PID)" || echo -e "  Controller: ${RED}已停止${NC}"
ps -p $LISTENER_PID > /dev/null && echo -e "  Listener:   ${GREEN}运行中${NC} (PID: $LISTENER_PID)" || echo -e "  Listener:   ${RED}已停止${NC}"
echo ""
echo "查看完整日志:"
echo "  Controller: tail -f /tmp/motor_controller.log"
echo "  Listener:   tail -f /tmp/listener.log"
echo ""
echo "停止所有进程:"
echo "  kill $CONTROLLER_PID $LISTENER_PID"
echo ""
