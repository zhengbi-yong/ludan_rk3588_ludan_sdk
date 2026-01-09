#!/bin/bash
#
# start_motor_feedback_logger.sh
#
# 电机反馈数据记录启动脚本
#
# 功能：
#   1. 自动source ROS2环境
#   2. 检查motor_feedback topic是否存在
#   3. 启动数据记录脚本
#
# 使用方法：
#   ./start_motor_feedback_logger.sh
#
# 注意：请确保先启动 can_motor_feedback_publisher 节点

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Motor Feedback Logger Startup Script${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 1. Source ROS2环境
echo -e "${YELLOW}[1/3] Sourcing ROS2 environment...${NC}"
ROS2_SETUP="/home/linaro/ludan_sdk/multi_port_motor_feedback/multi_port_motor_feedback_ros2/install/setup.bash"

if [ -f "$ROS2_SETUP" ]; then
    source "$ROS2_SETUP"
    echo -e "   ${GREEN}OK${NC} - ROS2 environment sourced"
else
    echo -e "   ${RED}ERROR${NC} - Cannot find $ROS2_SETUP"
    exit 1
fi

# 2. 检查motor_feedback topic
echo -e "${YELLOW}[2/3] Checking /motor_feedback topic...${NC}"
sleep 2  # 等待topic列表更新

if ros2 topic list 2>/dev/null | grep -q "/motor_feedback"; then
    echo -e "   ${GREEN}OK${NC} - Topic /motor_feedback found"

    # 显示topic信息
    echo -e "   ${YELLOW}Topic info:${NC}"
    ros2 topic info /motor_feedback 2>/dev/null | sed 's/^/     /'
else
    echo -e "   ${YELLOW}WARNING${NC} - Topic /motor_feedback not found!"
    echo -e "   ${YELLOW}Please start can_motor_feedback_publisher first:${NC}"
    echo -e "      ros2 run multi_port_motor_feedback can_motor_feedback_publisher"
    echo ""
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 3. 启动数据记录脚本
echo -e "${YELLOW}[3/3] Starting motor feedback logger...${NC}"
echo ""

# 检查Python脚本是否存在
PYTHON_SCRIPT="$SCRIPT_DIR/motor_feedback_logger.py"
if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo -e "   ${RED}ERROR${NC} - Cannot find $PYTHON_SCRIPT"
    exit 1
fi

# 运行Python脚本
python3 "$PYTHON_SCRIPT" --log-dir "$SCRIPT_DIR"

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Logger stopped${NC}"
echo -e "${GREEN}========================================${NC}"
