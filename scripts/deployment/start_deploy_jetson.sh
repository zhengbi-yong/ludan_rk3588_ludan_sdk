#!/bin/bash

# Jetson客户端启动脚本
# 用法: ./start_deploy_jetson.sh <rk3588_ip> [port] [options]

set -e

# 参数检查
if [ $# -lt 1 ]; then
    echo "用法: ./start_deploy_jetson.sh <rk3588_ip> [port] [options]"
    echo ""
    echo "参数说明:"
    echo "  rk3588_ip    RK3588服务器IP地址"
    echo "  port         服务器端口 (默认: 8888)"
    echo ""
    echo "可选参数:"
    echo "  --amplitude <rad> 正弦波幅度 (默认: 0.3 rad)"
    echo "  --frequency <hz>  正弦波频率 (默认: 0.5 Hz)"
    echo "  --rate <hz>       发布频率 (默认: 50 Hz)"
    echo "  --duration <s>    测试持续时间 (默认: 30 秒)"
    echo ""
    echo "示例:"
    echo "  ./start_deploy_jetson.sh 192.168.1.100 8888"
    echo "  ./start_deploy_jetson.sh 192.168.1.100 8888 --amplitude 0.5"
    echo "  ./start_deploy_jetson.sh 192.168.1.100 8888 --frequency 1.0 --duration 60"
    exit 1
fi

# 参数解析
RK3588_IP="$1"
SERVER_PORT="${2:-8888}"
shift 2

# 默认运动参数
AMPLITUDE="0.3"
FREQUENCY="0.5"
RATE="50"
DURATION="30"

# 解析可选参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --amplitude)
            AMPLITUDE="$2"
            shift 2
            ;;
        --frequency)
            FREQUENCY="$2"
            shift 2
            ;;
        --rate)
            RATE="$2"
            shift 2
            ;;
        --duration)
            DURATION="$2"
            shift 2
            ;;
        *)
            echo "未知参数: $1"
            exit 1
            ;;
    esac
done

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    Jetson Deploy Test 客户端启动${NC}"
echo -e "${BLUE}========================================${NC}"

# 检查可执行文件
BINARY_PATH="$HOME/unitree_sdk2/build/bin/deploy_test_jetson"
if [ ! -f "$BINARY_PATH" ]; then
    echo -e "${RED}❌ 找不到可执行文件: $BINARY_PATH${NC}"
    echo -e "${YELLOW}请确保deploy_test_jetson在当前目录${NC}"
    echo -e "${YELLOW}或提供完整路径${NC}"
    exit 1
fi

echo -e "${GREEN}✅ 找到可执行文件: $BINARY_PATH${NC}"

# 验证IP地址格式
if ! echo "$RK3588_IP" | grep -qE '^([0-9]{1,3}\.){3}[0-9]{1,3}$'; then
    echo -e "${RED}❌ 无效的IP地址格式: $RK3588_IP${NC}"
    exit 1
fi

# 验证端口范围
if ! [[ "$SERVER_PORT" =~ ^[0-9]+$ ]] || [ "$SERVER_PORT" -lt 1024 ] || [ "$SERVER_PORT" -gt 65535 ]; then
    echo -e "${RED}❌ 端口号必须在1024-65535之间: $SERVER_PORT${NC}"
    exit 1
fi

echo -e "${GREEN}📋 配置参数:${NC}"
echo -e "   - RK3588服务器: ${YELLOW}$RK3588_IP:$SERVER_PORT${NC}"
echo -e "   - 正弦波幅度: ${YELLOW}$AMPLITUDE rad ($(echo "$AMPLITUDE * 180 / 3.14159" | bc -l)°)${NC}"
echo -e "   - 正弦波频率: ${YELLOW}$FREQUENCY Hz${NC}"
echo -e "   - 发布频率: ${YELLOW}$RATE Hz${NC}"
echo -e "   - 测试持续时间: ${YELLOW}$DURATION 秒${NC}"
echo

# 网络连通性测试
echo -e "${BLUE}🔍 检查网络连通性...${NC}"
if ping -c 2 "$RK3588_IP" >/dev/null 2>&1; then
    echo -e "${GREEN}✅ 可以ping通RK3588服务器${NC}"

    # 测量延迟
    avg_delay=$(ping -c 3 "$RK3588_IP" 2>/dev/null | tail -1 | awk -F'/' '{print $5}' | awk '{print $1}')
    if [ ! -z "$avg_delay" ]; then
        echo -e "${GREEN}   平均延迟: ${YELLOW}$avg_delay ms${NC}"
        if (( $(echo "$avg_delay < 50" | bc -l 2>/dev/null || echo "1") )); then
            echo -e "${GREEN}   网络延迟良好，适合实时控制${NC}"
        elif (( $(echo "$avg_delay < 100" | bc -l 2>/dev/null || echo "0") )); then
            echo -e "${YELLOW}   网络延迟一般，可能影响实时性能${NC}"
        else
            echo -e "${RED}   网络延迟较高，建议检查网络质量${NC}"
        fi
    fi
else
    echo -e "${RED}❌ 无法ping通RK3588服务器${NC}"
    echo -e "${YELLOW}请检查:${NC}"
    echo "   - 两台设备是否在同一WiFi网络"
    echo "   - RK3588是否已启动服务端"
    echo "   - 防火墙设置"
    echo
    echo -e "${YELLOW}继续尝试连接？(y/N): ${NC}"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 端口连通性测试
echo -e "${BLUE}🔍 检查端口连通性...${NC}"
if command -v nc >/dev/null 2>&1; then
    echo "测试TCP连接到 $RK3588_IP:$SERVER_PORT..."
    if timeout 5 nc -zv "$RK3588_IP" "$SERVER_PORT" 2>/dev/null; then
        echo -e "${GREEN}✅ 端口 $RK3588_IP:$SERVER_PORT 可达${NC}"
        echo -e "${GREEN}   RK3588服务端已启动${NC}"
    else
        echo -e "${RED}❌ 端口 $RK3588_IP:$SERVER_PORT 不可达${NC}"
        echo -e "${YELLOW}请确保:${NC}"
        echo "   - RK3588已运行: ./start_deploy_rk3588.sh wlan0 $SERVER_PORT"
        echo "   - 防火墙允许端口 $SERVER_PORT"
        echo
        echo -e "${YELLOW}继续尝试连接？(y/N): ${NC}"
        read -r response
        if [[ ! "$response" =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
else
    echo -e "${YELLOW}⚠️  nc命令不可用，跳过端口测试${NC}"
fi

echo
echo -e "${BLUE}🚀 启动Jetson客户端...${NC}"
echo -e "${YELLOW}注意: 确保RK3588已启动对应的服务端程序${NC}"
echo -e "${YELLOW}测试过程中可按Ctrl+C停止${NC}"
echo

# 构建命令参数
CLIENT_ARGS="$RK3588_IP $SERVER_PORT"
CLIENT_ARGS="$CLIENT_ARGS --amplitude $AMPLITUDE"
CLIENT_ARGS="$CLIENT_ARGS --frequency $FREQUENCY"
CLIENT_ARGS="$CLIENT_ARGS --rate $RATE"
CLIENT_ARGS="$CLIENT_ARGS --duration $DURATION"

echo -e "${GREEN}执行命令: $BINARY_PATH $CLIENT_ARGS${NC}"
echo

# 启动客户端
exec "$BINARY_PATH" $CLIENT_ARGS
