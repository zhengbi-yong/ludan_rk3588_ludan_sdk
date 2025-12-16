#!/bin/bash

# CAN帧实时监控工具
# 显示各电机ID的命令发送情况

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# 初始化计数器
declare -A motor_count
declare -A motor_last_pos
declare -A motor_freq

motor_count[201]=0
motor_count[202]=0
motor_count[203]=0
motor_count[204]=0

start_time=$(date +%s)
total_count=0

# 清屏函数
clear_screen() {
    echo -e "\033[2J\033[H"
}

# 显示标题
show_header() {
    local current_time=$(date +%s)
    local elapsed=$((current_time - start_time))
    local total_hz=0

    if [ $elapsed -gt 0 ]; then
        total_hz=$((total_count / elapsed))
    fi

    echo -e "${CYAN}${BOLD}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║                    CAN电机命令监控器                        ║"
    printf "║ 总帧数: %-8d 总频率: %-6d Hz 运行时间: %-5ds            ║\n" $total_count $total_hz $elapsed
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

# 显示电机状态
show_motor_status() {
    echo -e "${CYAN}┌─────────────────────┬─────────┬──────────┬─────────────┐${NC}"
    echo -e "${CYAN}│      电机ID        │  帧数   │  频率(Hz) │   位置(rad)  │${NC}"
    echo -e "${CYAN}├─────────────────────┼─────────┼──────────┼─────────────┤${NC}"

    for id in 201 202 203 204; do
        local count=${motor_count[$id]}
        local freq=0

        if [ $elapsed -gt 0 ]; then
            freq=$((count / elapsed))
        fi

        local pos=${motor_last_pos[$id]:-"---"}
        local motor_name="Motor $((id-0x200))"

        # 根据是否有数据选择颜色
        local color="${GREEN}"
        if [ $count -eq 0 ]; then
            color="${RED}"
        fi

        printf "${color}│ %-17s │ %7d │ %8d │ %11s │${NC}\n" \
            "$motor_name" "$count" "$freq" "$pos"
    done

    echo -e "${CYAN}└─────────────────────┴─────────┴──────────┴─────────────┘${NC}"
}

# 解析CAN帧数据
parse_can_frame() {
    local can_id=$1
    local data=$2

    # 只处理我们关心的电机ID
    if [[ $can_id =~ ^(201|202|203|204)$ ]]; then
        # 更新计数
        motor_count[$can_id]=$((motor_count[$can_id] + 1))
        total_count=$((total_count + 1))

        # 解析位置数据 (前2字节, little-endian)
        local pos_hex=$(echo $data | cut -d' ' -f1-2 | tr ' ' '\n' | tac | tr -d '\n')
        if [ ${#pos_hex} -eq 4 ]; then
            local pos_int=$((0x$pos_hex))
            # 转换为rad (简化版)
            local pos_float=$(echo "scale=3; $pos_int * 12.5 / 32767" | bc -l 2>/dev/null)
            motor_last_pos[$can_id]=$pos_float
        fi
    fi
}

# 主监控循环
echo -e "${GREEN}🚀 启动CAN电机监控器...${NC}"
echo -e "${BLUE}监控接口: can0${NC}"
echo -e "${YELLOW}按 Ctrl+C 退出${NC}"
echo ""

# 启动candump监控
candump can0 -tA 2>/dev/null | while read -r line; do
    # 解析candump输出
    # 格式: (timestamp) can0  ID [DLC]  data
    if [[ $line =~ ^\(.*\)\ +can0\ +([0-9A-F]+)\ +\[[0-9]+\]\ +(.+)$ ]]; then
        can_id="16#${BASH_REMATCH[1]}"
        can_id=$((can_id))
        data="${BASH_REMATCH[2]}"

        # 解析并更新数据
        parse_can_frame $can_id "$data"

        # 每50帧更新一次显示
        if [ $((total_count % 50)) -eq 0 ]; then
            clear_screen
            show_header
            show_motor_status

            # 显示最近接收的帧
            echo -e "\n${YELLOW}📡 最新接收:${NC}"
            echo "ID: $can_id (${BASH_REMATCH[1]}) | Data: $data"
        fi
    fi
done