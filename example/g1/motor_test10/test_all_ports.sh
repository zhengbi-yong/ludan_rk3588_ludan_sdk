#!/bin/bash
# 四端口完整测试脚本
# 自动化测试端口 8000, 8001, 8002, 8003 的数据下发

set -e

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  四端口电机测试 - 完整自动化流程${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ROS2 环境未加载${NC}"
    echo "请先运行: source /opt/ros/humble/setup.bash"
    exit 1
fi

# Source 工作空间
for ws in "/home/linaro/yunfei_ws" "/home/linaro/motor_feedback" "/home/linaro/controller_ws"; do
    if [ -d "$ws/install" ]; then
        source "$ws/install/setup.bash" 2>/dev/null || true
    fi
done

echo -e "${GREEN}✓ ROS2 环境: $ROS_DISTRO${NC}"
echo ""

# ============================================================
# 步骤 1: 使能所有电机
# ============================================================
echo -e "${YELLOW}[1/5] 使能电机...${NC}"
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable 1 2>/dev/null || true
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable 2 2>/dev/null || true
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable 3 2>/dev/null || true
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable 4 2>/dev/null || true
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable 5 2>/dev/null || true
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable 6 2>/dev/null || true
echo -e "${GREEN}✓ 电机已使能 (ID: 1, 2, 3, 4, 5, 6)${NC}"
echo ""

# ============================================================
# 步骤 2: 启动控制器 (后台)
# ============================================================
echo -e "${YELLOW}[2/5] 启动 Motor Controller...${NC}"

# 停止旧进程
pkill -f motor_controller 2>/dev/null || true
pkill -f listener_8888 2>/dev/null || true
sleep 1

# 启动控制器
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable \
    --enable-motor-cmd --auto-start \
    > /tmp/motor_test10_controller.log 2>&1 &

CONTROLLER_PID=$!
sleep 3

if ps -p $CONTROLLER_PID > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Controller PID: $CONTROLLER_PID${NC}"
else
    echo -e "${RED}❌ Controller 启动失败${NC}"
    cat /tmp/motor_test10_controller.log
    exit 1
fi

# 显示初始化信息
grep -E "Port.*Initialized|All.*Ports Initialized" /tmp/motor_test10_controller.log || true
echo ""

# ============================================================
# 步骤 3: 启动 listener (后台)
# ============================================================
echo -e "${YELLOW}[3/5] 启动 UDP Listener...${NC}"
cd /home/linaro/ludan_sdk/scripts
python3 listener_8888_to_ros2.py 8888 \
    > /tmp/motor_test10_listener.log 2>&1 &

LISTENER_PID=$!
sleep 2

if ps -p $LISTENER_PID > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Listener PID: $LISTENER_PID${NC}"
else
    echo -e "${RED}❌ Listener 启动失败${NC}"
    cat /tmp/motor_test10_listener.log
    kill $CONTROLLER_PID 2>/dev/null || true
    exit 1
fi
echo ""

# ============================================================
# 步骤 4: 发送测试数据
# ============================================================
echo -e "${YELLOW}[4/5] 发送测试数据...${NC}"
cd /home/linaro/ludan_sdk/example/g1/motor_test10

echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}  测试 1: 单次数据包 (所有端口)${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
python3 four_port_udp_sender.py --all

echo ""
sleep 2
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}  测试 2: 持续发送 (10秒, 50Hz)${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
python3 four_port_udp_sender.py --continuous 10

echo ""
sleep 2
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}  测试 3: 正弦波测试 (5秒)${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
python3 four_port_udp_sender.py --sine 5

echo ""

# ============================================================
# 等待超时自动停止
# ============================================================
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}  等待超时自动停止 (500ms 超时)${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo "UDP 发送已停止，等待 500ms 超时..."
sleep 2

# ============================================================
# 步骤 5: 显示测试结果
# ============================================================
echo -e "${YELLOW}[5/5] 测试结果${NC}"
echo ""

# 显示控制器状态
echo -e "${BLUE}=== Controller 状态 (最后 5 行) ===${NC}"
tail -5 /tmp/motor_test10_controller.log | grep -E "DEBUG|STAT|端口分配" || \
tail -5 /tmp/motor_test10_controller.log

echo ""

# 显示端口发送统计
echo -e "${BLUE}=== 端口发送统计 ===${NC}"
if grep -q "DEBUG.*端口分配" /tmp/motor_test10_controller.log; then
    grep "DEBUG.*端口分配" /tmp/motor_test10_controller.log | tail -1
else
    # 从 STAT 行中提取信息
    tail -1 /tmp/motor_test10_controller.log | grep "STAT"
fi

echo ""
echo -e "${BLUE}=== 预期结果验证 ===${NC}"
echo "  端口 8000: 电机 1, 2, 3 (3 个电机)"
echo "  端口 8001: 电机 4 (1 个电机)"
echo "  端口 8002: 电机 5 (1 个电机)"
echo "  端口 8003: 电机 6 (1 个电机)"
echo ""

# ============================================================
# 清理
# ============================================================
echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}  测试完成！${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""
echo "进程状态:"
ps -p $CONTROLLER_PID > /dev/null 2>&1 && echo -e "  Controller: ${GREEN}运行中${NC} (PID: $CONTROLLER_PID)" || echo -e "  Controller: ${RED}已停止${NC}"
ps -p $LISTENER_PID > /dev/null 2>&1 && echo -e "  Listener:   ${GREEN}运行中${NC} (PID: $LISTENER_PID)" || echo -e "  Listener:   ${RED}已停止${NC}"
echo ""
echo "查看完整日志:"
echo "  Controller: tail -f /tmp/motor_test10_controller.log"
echo "  Listener:   tail -f /tmp/motor_test10_listener.log"
echo ""
echo "停止所有进程:"
echo "  kill $CONTROLLER_PID $LISTENER_PID"
echo ""
