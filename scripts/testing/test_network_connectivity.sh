#!/bin/bash

# 网络连接测试脚本
# 用于测试RK3588与Jetson之间的网络连接

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    网络连接测试工具${NC}"
echo -e "${BLUE}========================================${NC}"

# 测试参数
TARGET_IP=${1:-"192.168.1.100"}  # 默认Jetson IP
TARGET_PORT=${2:-"8888"}          # 默认端口

echo -e "${GREEN}📋 测试配置:${NC}"
echo -e "   - 目标IP: ${YELLOW}$TARGET_IP${NC}"
echo -e "   - 目标端口: ${YELLOW}$TARGET_PORT${NC}"
echo

# 1. 检查网络接口
echo -e "${BLUE}🔍 1. 检查网络接口...${NC}"
echo "可用网络接口:"
ip addr show | grep -E '^[0-9]+:' | awk -F': ' '{print "   " $2}' | sed 's/@.*//'
echo

# 2. 检查IP地址
echo -e "${BLUE}🔍 2. 检查本机IP地址...${NC}"
for interface in wlan0 eth0; do
    if ip addr show "$interface" >/dev/null 2>&1; then
        ip_addr=$(ip addr show "$interface" | grep 'inet ' | awk '{print $2}' | cut -d'/' -f1)
        if [ ! -z "$ip_addr" ]; then
            echo -e "   ${GREEN}$interface${NC}: $ip_addr"
        fi
    fi
done
echo

# 3. Ping测试
echo -e "${BLUE}🔍 3. Ping连通性测试...${NC}"
if ping -c 4 "$TARGET_IP" >/dev/null 2>&1; then
    echo -e "${GREEN}✅ Ping $TARGET_IP 成功${NC}"
else
    echo -e "${RED}❌ Ping $TARGET_IP 失败${NC}"
    echo -e "${YELLOW}请检查:${NC}"
    echo "   - 两台设备是否在同一网络"
    echo "   - 目标IP是否正确"
    echo "   - 防火墙设置"
fi
echo

# 4. 端口连通性测试
echo -e "${BLUE}🔍 4. 端口连通性测试...${NC}"
if command -v nc >/dev/null 2>&1; then
    echo "测试TCP连接到 $TARGET_IP:$TARGET_PORT..."
    if timeout 5 nc -zv "$TARGET_IP" "$TARGET_PORT" 2>/dev/null; then
        echo -e "${GREEN}✅ 端口 $TARGET_IP:$TARGET_PORT 可达${NC}"
    else
        echo -e "${RED}❌ 端口 $TARGET_IP:$TARGET_PORT 不可达${NC}"
        echo -e "${YELLOW}请检查:${NC}"
        echo "   - 目标服务器是否运行"
        echo "   - 防火墙是否阻止端口 $TARGET_PORT"
    fi
else
    echo -e "${YELLOW}⚠️  nc命令不可用，跳过端口测试${NC}"
fi
echo

# 5. 网络延迟测试
echo -e "${BLUE}🔍 5. 网络延迟测试...${NC}"
if ping -c 10 "$TARGET_IP" >/dev/null 2>&1; then
    avg_delay=$(ping -c 10 "$TARGET_IP" | tail -1 | awk -F'/' '{print $5}' | awk '{print $1}')
    if [ ! -z "$avg_delay" ]; then
        echo -e "${GREEN}✅ 平均延迟: ${YELLOW}$avg_delay ms${NC}"
        if (( $(echo "$avg_delay < 10" | bc -l) )); then
            echo -e "${GREEN}   延迟极低，适合实时控制${NC}"
        elif (( $(echo "$avg_delay < 50" | bc -l) )); then
            echo -e "${YELLOW}   延迟较低，适合一般控制${NC}"
        else
            echo -e "${RED}   延迟较高，可能影响实时性能${NC}"
        fi
    fi
else
    echo -e "${RED}❌ 无法测量延迟${NC}"
fi
echo

echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}网络测试完成${NC}"
echo
echo -e "${YELLOW}测试通过后的启动步骤:${NC}"
echo "1. 在RK3588上: ./start_deploy_rk3588.sh wlan0 $TARGET_PORT"
echo "2. 在Jetson上: ./deploy_test_jetson $(hostname -I | awk '{print $1}') $TARGET_PORT"
