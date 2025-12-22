#!/bin/bash

# 启动 UDP 8888 到 ROS2 转发器

set -e

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 显示信息
echo -e "${BLUE}====================================${NC}"
echo -e "${BLUE}    UDP 8888 to ROS2 转发器       ${NC}"
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

# 检查必要的工作空间
WORKSPACES=(
    "/home/linaro/yunfei_ws"
    "/home/linaro/motor_feedback"
    "/home/linaro/controller_ws"
)

for ws in "${WORKSPACES[@]}"; do
    if [ -d "$ws/install" ]; then
        echo -e "${GREEN}✓ 找到工作空间: $ws${NC}"
        source "$ws/install/setup.bash"
    fi
done

# 检查端口参数
PORT=${1:-8888}
echo -e "${GREEN}✓ 监听端口: $PORT${NC}"

# 检查端口是否被占用
if command -v lsof &> /dev/null; then
    if lsof -Pi :$PORT -sTCP:LISTEN -t >/dev/null 2>&1; then
        echo -e "${YELLOW}⚠️  端口 $PORT 已被占用${NC}"
        echo -e "${BLUE}将尝试使用备用端口...${NC}"

        # 尝试备用端口
        for alt_port in {8889..8899}; do
            if ! lsof -Pi :$alt_port -sTCP:LISTEN -t >/dev/null 2>&1; then
                PORT=$alt_port
                echo -e "${GREEN}✓ 使用备用端口: $PORT${NC}"
                break
            fi
        done
    fi
fi

# 显示将要发布的topics
echo
echo -e "${BLUE}📡 将发布以下ROS2 topics:${NC}"
echo "  - /lowcmd (xixilowcmd/LowCmd格式)"
echo "  - /lowcmd_positions"
echo "  - /lowcmd_velocities"
echo "  - /lowcmd_efforts"
echo "  - /lowcmd_motor_modes"
echo "  - /lowcmd_joint_states"
echo "  - /lowcmd_mode_pr"
echo "  - /lowcmd_mode_machine"
echo

echo -e "${GREEN}🚀 启动转发器...${NC}"
echo -e "${YELLOW}等待Jetson发送数据到 ${PORT} 端口${NC}"
echo -e "${BLUE}按 Ctrl+C 停止${NC}"
echo

# 启动转发器
exec "$PWD/listener_8888_to_ros2.py" $PORT