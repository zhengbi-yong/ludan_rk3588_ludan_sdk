#!/bin/bash

# RK3588服务端启动脚本
# 用法: ./start_deploy_rk3588.sh [network_interface] [port]

set -e

# 默认参数
NETWORK_INTERFACE=${1:-"wlan0"}
SERVER_PORT=${2:-"8888"}

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    RK3588 Deploy Test 服务端启动${NC}"
echo -e "${BLUE}========================================${NC}"

# 检查编译结果
BINARY_PATH="/home/linaro/unitree_sdk2/build/bin/deploy_test_rk3588"
if [ ! -f "$BINARY_PATH" ]; then
    echo -e "${RED}❌ 找不到可执行文件: $BINARY_PATH${NC}"
    echo -e "${YELLOW}请先运行编译命令:${NC}"
    echo "cd /home/linaro/unitree_sdk2/build"
    echo "make deploy_test_rk3588"
    exit 1
fi

echo -e "${GREEN}✅ 找到可执行文件: $BINARY_PATH${NC}"
echo -e "${GREEN}📋 配置参数:${NC}"
echo -e "   - 网络接口: ${YELLOW}$NETWORK_INTERFACE${NC}"
echo -e "   - 监听端口: ${YELLOW}$SERVER_PORT${NC}"
echo

# 检查网络接口是否存在
if ! ip link show "$NETWORK_INTERFACE" >/dev/null 2>&1; then
    echo -e "${RED}❌ 网络接口 '$NETWORK_INTERFACE' 不存在${NC}"
    echo -e "${YELLOW}可用的网络接口:${NC}"
    ip link show | grep -E '^[0-9]+:' | awk -F': ' '{print "   " $2}' | grep -v lo
    exit 1
fi

echo -e "${GREEN}✅ 网络接口 '$NETWORK_INTERFACE' 存在${NC}"

# 检查端口是否被占用
if command -v netstat >/dev/null 2>&1; then
    if netstat -tuln 2>/dev/null | grep -q ":$SERVER_PORT "; then
        echo -e "${YELLOW}⚠️  端口 $SERVER_PORT 已被占用${NC}"
        echo -e "${YELLOW}请使用其他端口或停止占用该端口的服务${NC}"
    else
        echo -e "${GREEN}✅ 端口 $SERVER_PORT 可用${NC}"
    fi
fi

echo
echo -e "${BLUE}🚀 启动RK3588服务端...${NC}"
echo -e "${YELLOW}注意: 确保防火墙允许端口 $SERVER_PORT 的TCP连接${NC}"
echo -e "${YELLOW}启动后，在Jetson上运行: ./deploy_test_jetson <RK3588_IP> $SERVER_PORT${NC}"
echo

cd /home/linaro/unitree_sdk2/build
./bin/deploy_test_rk3588 "$NETWORK_INTERFACE" "$SERVER_PORT"