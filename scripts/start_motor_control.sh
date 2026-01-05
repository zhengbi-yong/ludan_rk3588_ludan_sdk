#!/bin/bash

# 启动完整的电机控制数据流
# 1. UDP监听器 (listener.sh)
# 2. motor_controller_with_enable
# 3. 可选：使能指定电机

set -e

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}====================================${NC}"
echo -e "${BLUE}    启动完整电机控制数据流        ${NC}"
echo -e "${BLUE}====================================${NC}"
echo

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ROS2环境未找到${NC}"
    echo -e "${YELLOW}请先source ROS2环境:${NC}"
    echo "source /opt/ros/humble/setup.bash"
    exit 1
fi

echo -e "${GREEN}✓ ROS2环境: $ROS_DISTRO${NC}"

# Source工作空间
for ws in "/home/linaro/controller_ws" "/home/linaro/yunfei_ws" "/home/linaro/motor_feedback"; do
    if [ -d "$ws/install" ]; then
        echo -e "${GREEN}✓ Source工作空间: $ws${NC}"
        source "$ws/install/setup.bash"
    fi
done

echo
echo -e "${BLUE}选择启动模式:${NC}"
echo "1) 仅启动UDP监听器 (listener.sh)"
echo "2) 仅启动motor_controller_with_enable"
echo "3) 同时启动1和2 (完整数据流)"
echo "4) 完整数据流 + 自动使能电机12,29,30"
read -p "请选择 [1-4]: " choice

case $choice in
    1)
        echo -e "${GREEN}🚀 启动UDP监听器...${NC}"
        cd /home/linaro/ludan_sdk/scripts
        exec ./listener.sh
        ;;
    2)
        echo -e "${GREEN}🚀 启动motor_controller_with_enable...${NC}"
        exec /home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable-motor-cmd
        ;;
    3)
        echo -e "${GREEN}🚀 启动完整数据流...${NC}"
        echo -e "${YELLOW}在两个终端分别运行:${NC}"
        echo "终端1: cd /home/linaro/ludan_sdk/scripts && ./listener.sh"
        echo "终端2: /home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable-motor-cmd"
        ;;
    4)
        echo -e "${GREEN}🚀 启动完整数据流并使能电机...${NC}"
        echo -e "${YELLOW}在三个终端分别运行:${NC}"
        echo "终端1: cd /home/linaro/ludan_sdk/scripts && ./listener.sh"
        echo "终端2: /home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable-motor-cmd"
        echo "终端3: 使能电机"
        echo ""
        echo "按Enter继续使能电机..."
        read
        echo -e "${GREEN}使能电机 12, 29, 30...${NC}"
        /home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable 12
        /home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable 29
        /home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable 30
        echo -e "${GREEN}✅ 电机已使能${NC}"
        ;;
    *)
        echo -e "${RED}无效选择${NC}"
        exit 1
        ;;
esac
