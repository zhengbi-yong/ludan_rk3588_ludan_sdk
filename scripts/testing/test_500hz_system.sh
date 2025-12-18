#!/bin/bash

# 500Hz DDS-to-CAN系统完整测试脚本
# 测试deploy_test发送50Hz正弦波，motor_controller输出500Hz插值

echo "🚀 500Hz DDS-to-CAN系统测试"
echo "=========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m'

# 测试参数
TEST_DURATION=10  # 测试10秒
NETWORK_INTERFACE="veth0"
CAN_INTERFACE="can0"

echo -e "${BLUE}📋 测试配置:${NC}"
echo "  - DDS网络接口: $NETWORK_INTERFACE"
echo "  - CAN接口: $CAN_INTERFACE"
echo "  - 测试时长: ${TEST_DURATION}秒"
echo "  - deploy_test频率: 50Hz"
echo "  - motor_controller频率: 500Hz"
echo ""

# 检查必要的可执行文件
echo -e "${BLUE}🔍 检查可执行文件...${NC}"
if [ ! -f "/home/linaro/unitree_sdk2/build/bin/motor_controller" ]; then
    echo -e "${RED}❌ motor_controller 未找到${NC}"
    echo "请先运行: make motor_controller"
    exit 1
fi

if [ ! -f "/home/linaro/unitree_sdk2/build/bin/deploy_test" ]; then
    echo -e "${RED}❌ deploy_test 未找到${NC}"
    echo "请先运行: make deploy_test"
    exit 1
fi

if [ ! -f "/home/linaro/unitree_sdk2/can_quick_test.sh" ]; then
    echo -e "${RED}❌ can_quick_test.sh 未找到${NC}"
    exit 1
fi

echo -e "${GREEN}✅ 所有必要文件已就位${NC}"

# 检查网络接口
echo ""
echo -e "${BLUE}🌐 检查网络接口...${NC}"
if ! ip link show $NETWORK_INTERFACE >/dev/null 2>&1; then
    echo -e "${YELLOW}⚠️  网络接口 $NETWORK_INTERFACE 不存在${NC}"
    echo "正在创建虚拟网络接口..."

    # 尝试创建veth接口
    if ip link show veth0 >/dev/null 2>&1; then
        echo "veth0 已存在"
        NETWORK_INTERFACE="veth0"
    elif ip link show veth1 >/dev/null 2>&1; then
        echo "veth1 已存在"
        NETWORK_INTERFACE="veth1"
    else
        echo -e "${RED}❌ 无法创建虚拟网络接口${NC}"
        echo "请手动创建或使用物理接口"
        exit 1
    fi
fi

# 检查CAN接口
if ! ip link show $CAN_INTERFACE >/dev/null 2>&1; then
    echo -e "${YELLOW}⚠️  CAN接口 $CAN_INTERFACE 不存在${NC}"
    echo "正在检查可用CAN接口..."

    for can_if in can0 can1 can2; do
        if ip link show $can_if >/dev/null 2>&1; then
            echo -e "${GREEN}✅ 找到CAN接口: $can_if${NC}"
            CAN_INTERFACE="$can_if"
            break
        fi
    done
fi

echo -e "${GREEN}✅ 网络配置完成${NC}"
echo "  DDS接口: $NETWORK_INTERFACE"
echo "  CAN接口: $CAN_INTERFACE"
echo ""

# 停止可能运行的进程
echo -e "${BLUE}🛑 停止可能运行的进程...${NC}"
pkill -f motor_controller 2>/dev/null || true
pkill -f deploy_test 2>/dev/null || true
pkill -f g1_ankle_swing 2>/dev/null || true
sleep 1

echo -e "${BLUE}🔧 启动motor_controller (500Hz输出)...${NC}"
/home/linaro/unitree_sdk2/build/bin/motor_controller $NETWORK_INTERFACE $CAN_INTERFACE &
MOTOR_PID=$!
echo "motor_controller PID: $MOTOR_PID"

# 等待motor_controller启动
sleep 3

# 检查motor_controller是否正常运行
if ! ps -p $MOTOR_PID > /dev/null; then
    echo -e "${RED}❌ motor_controller 启动失败${NC}"
    exit 1
fi

echo -e "${GREEN}✅ motor_controller 已启动${NC}"

echo ""
echo -e "${BLUE}🌊 启动deploy_test (50Hz正弦波)...${NC}"
echo "测试参数: 50Hz发布, ${TEST_DURATION}秒持续时间"

# 启动deploy_test
timeout ${TEST_DURATION}s /home/linaro/unitree_sdk2/build/bin/deploy_test $NETWORK_INTERFACE --duration $TEST_DURATION &
DEPLOY_PID=$!

echo "deploy_test PID: $DEPLOY_PID"

# 启动CAN频率监控
echo ""
echo -e "${BLUE}📊 启动CAN频率监控...${NC}"
echo "监控${TEST_DURATION}秒CAN通信..."

# 运行快速CAN测试
if [ "$TEST_DURATION" -gt 3 ]; then
    sleep 3  # 等待数据稳定
    /home/linaro/unitree_sdk2/can_quick_test.sh &
    MONITOR_PID=$!
fi

# 等待测试完成
echo ""
echo -e "${BLUE}⏱️  测试进行中... (请等待${TEST_DURATION}秒)${NC}"

# 等待deploy_test完成
wait $DEPLOY_PID
DEPLOY_EXIT_CODE=$?

# 停止监控
if [ ! -z "$MONITOR_PID" ]; then
    sleep 2  # 等待最后的数据
    pkill -f can_quick_test.sh 2>/dev/null || true
    kill $MONITOR_PID 2>/dev/null || true
fi

# 停止motor_controller
echo ""
echo -e "${BLUE}🛑 停止motor_controller...${NC}"
kill $MOTOR_PID 2>/dev/null || true
wait $MOTOR_PID 2>/dev/null || true

# 显示测试结果
echo ""
echo -e "${BOLD}📊 测试完成结果${NC}"
echo "=========================================="

if [ $DEPLOY_EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}✅ deploy_test 成功完成${NC}"
    echo "  - DDS命令发送: 正常"
    echo "  - 测试时长: ${TEST_DURATION}秒"
else
    echo -e "${RED}❌ deploy_test 提前结束 (退出码: $DEPLOY_EXIT_CODE)${NC}"
    echo "  可能原因:"
    echo "    - DDS连接问题"
    echo "    - 网络接口问题"
    echo "    - 用户中断 (Ctrl+C)"
fi

echo ""
echo -e "${BLUE}🎯 建议下一步操作:${NC}"
echo "1. 检查CAN频率是否接近500Hz"
echo "2. 验证motor_controller的插值效果"
echo "3. 运行详细的500Hz验证工具:"
echo "   python3 verify_500hz.py"
echo "   ./verify_500hz_simple.sh"

echo ""
echo -e "${BOLD}🔍 快速验证命令:${NC}"
echo "# 验证500Hz输出 (需要motor_controller运行)"
echo "candump $CAN_INTERFACE -tA -T 5 | wc -l"
echo ""
echo "# 预期结果: 2500条CAN帧 (500Hz × 5秒)"
echo ""

echo -e "${GREEN}🎉 测试脚本执行完成!${NC}"