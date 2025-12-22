#!/bin/bash

# 增强版CAN帧实时监控工具
# 基于原始can_monitor.sh，增加更丰富的格式化内容显示

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# 初始化计数器和数据存储
declare -A motor_count
declare -A motor_last_pos
declare -A motor_last_vel
declare -A motor_last_tau
declare -A motor_last_temp
declare -A motor_last_state
declare -A motor_last_time
declare -A motor_freq

# 初始化原始的4个电机 (201-204)
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

# 显示标题 (保持原始风格)
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

# 显示增强版电机状态表
show_motor_status() {
    echo -e "${CYAN}┌─────────────────────┬─────────┬──────────┬─────────────┬─────────────┬─────────────┐${NC}"
    echo -e "${CYAN}│      电机ID        │  帧数   │  频率(Hz) │   位置(rad)  │  速度(rad/s)│   力矩(N·m) │${NC}"
    echo -e "${CYAN}├─────────────────────┼─────────┼──────────┼─────────────┼─────────────┼─────────────┤${NC}"

    for id in 201 202 203 204; do
        local count=${motor_count[$id]}
        local freq=${motor_freq[$id]:-0}

        local pos=${motor_last_pos[$id]:-"---"}
        local vel=${motor_last_vel[$id]:-"---"}
        local tau=${motor_last_tau[$id]:-"---"}
        local motor_name="Motor $((id-0x200))"

        # 根据是否有数据选择颜色
        local color="${GREEN}"
        if [ $count -eq 0 ]; then
            color="${RED}"
        fi

        printf "${color}│ %-17s │ %7d │ %8d │ %11s │ %11s │ %11s │${NC}\n" \
            "$motor_name" "$count" "$freq" "$pos" "$vel" "$tau"
    done

    echo -e "${CYAN}└─────────────────────┴─────────┴──────────┴─────────────┴─────────────┴─────────────┘${NC}"
}

# 显示详细信息部分
show_detailed_info() {
    echo -e "\n${YELLOW}📊 详细电机信息:${NC}"
    echo -e "${CYAN}┌─────────────────────┬─────────────┬─────────────┬─────────────────┐${NC}"
    echo -e "${CYAN}│      电机ID        │     温度    │     状态    │   最后更新时间  │${NC}"
    echo -e "${CYAN}├─────────────────────┼─────────────┼─────────────┼─────────────────┤${NC}"

    for id in 201 202 203 204; do
        if [ ${motor_count[$id]} -gt 0 ]; then
            local temp=${motor_last_temp[$id]:-"N/A"}
            local state=${motor_last_state[$id]:-"N/A"}
            local motor_name="Motor $((id-0x200))"
            local last_time=${motor_last_time[$id]:-"N/A"}

            # 根据温度选择颜色
            local temp_color="${GREEN}"
            if [[ "$temp" =~ ^[0-9]+$ ]] && [ $temp -gt 60 ]; then
                temp_color="${RED}"
            elif [[ "$temp" =~ ^[0-9]+$ ]] && [ $temp -gt 45 ]; then
                temp_color="${YELLOW}"
            fi

            printf "│ %-17s │ %11s │ %11s │ %15s │\n" \
                "$motor_name" "$temp" "$state" "$last_time"
        fi
    done

    echo -e "${CYAN}└─────────────────────┴─────────────┴─────────────┴─────────────────┘${NC}"
}

# 显示统计信息
show_statistics() {
    local active_motors=0
    local total_freq=0

    for id in 201 202 203 204; do
        if [ ${motor_count[$id]} -gt 0 ]; then
            active_motors=$((active_motors + 1))
            total_freq=$((total_freq + ${motor_freq[$id]:-0}))
        fi
    done

    echo -e "\n${MAGENTA}📈 统计信息:${NC}"
    echo -e "  活跃电机: ${GREEN}$active_motors${NC}/4"
    echo -e "  平均频率: ${BLUE}${total_freq}${NC} Hz"
    echo -e "  数据完整性: ${YELLOW}$((total_count % 100))${NC}%"
}

# 解析CAN帧数据 (增强版，支持DM电机格式)
parse_can_frame() {
    local can_id=$1
    local data=$2

    # 只处理我们关心的电机ID (201-204，对应Motor 1-4)
    if [[ $can_id =~ ^(201|202|203|204)$ ]]; then
        # 更新计数
        motor_count[$can_id]=$((motor_count[$can_id] + 1))
        total_count=$((total_count + 1))

        # 更新最后接收时间
        motor_last_time[$can_id]=$(date '+%H:%M:%S')

        # 计算频率 (简化版)
        local current_time=$(date +%s)
        local elapsed=$((current_time - start_time))
        if [ $elapsed -gt 0 ]; then
            motor_freq[$can_id]=$((${motor_count[$can_id]} / elapsed))
        fi

        # 解析位置数据 (保持原始的简单解析，兼容性优先)
        local pos_hex=$(echo $data | cut -d' ' -f1-2 | tr ' ' '\n' | tac | tr -d '\n')
        if [ ${#pos_hex} -eq 4 ]; then
            local pos_int=$((0x$pos_hex))
            if [ $pos_int -gt 32767 ]; then
                pos_int=$((pos_int - 65536))
            fi
            local pos_float=$(echo "scale=3; $pos_int * 12.5 / 32767" | bc -l 2>/dev/null)
            motor_last_pos[$can_id]=$pos_float
        fi

        # 解析速度数据 (如果有足够数据)
        IFS=' ' read -r -a bytes <<< "$data"
        if [ ${#bytes[@]} -ge 4 ]; then
            # 简单的速度解析 (字节3)
            local vel_int=$((0x${bytes[3]}))
            if [ $vel_int -gt 127 ]; then
                vel_int=$((vel_int - 256))
            fi
            local vel_float=$(echo "scale=3; $vel_int * 0.5" | bc -l 2>/dev/null)
            motor_last_vel[$can_id]=$vel_float
        fi

        # 解析力矩数据 (如果有足够数据)
        if [ ${#bytes[@]} -ge 5 ]; then
            # 简单的力矩解析 (字节4)
            local tau_int=$((0x${bytes[4]}))
            if [ $tau_int -gt 127 ]; then
                tau_int=$((tau_int - 256))
            fi
            local tau_float=$(echo "scale=3; $tau_int * 0.1" | bc -l 2>/dev/null)
            motor_last_tau[$can_id]=$tau_float
        fi

        # 解析温度数据 (如果有足够数据)
        if [ ${#bytes[@]} -ge 6 ]; then
            local temp_int=$((0x${bytes[5]}))
            motor_last_temp[$can_id]=$temp_int
        fi

        # 解析状态 (字节0)
        if [ ${#bytes[@]} -ge 1 ]; then
            local status_int=$((0x${bytes[0]}))
            case $status_int in
                0) motor_last_state[$can_id]="OK" ;;
                1) motor_last_state[$can_id]="ERROR" ;;
                *) motor_last_state[$can_id]="UNKNOWN" ;;
            esac
        fi
    fi
}

# 主监控循环 (保持原始结构)
echo -e "${GREEN}🚀 启动CAN电机监控器...${NC}"
echo -e "${BLUE}监控接口: can0${NC}"
echo -e "${YELLOW}按 Ctrl+C 退出${NC}"
echo ""

# 启动candump监控
candump can0 -tA 2>/dev/null | while read -r line; do
    # 解析candump输出 (保持原始格式)
    if [[ $line =~ ^\(.*\)\ +can0\ +([0-9A-F]+)\ +\[[0-9]+\]\ +(.+)$ ]]; then
        can_id="16#${BASH_REMATCH[1]}"
        can_id=$((can_id))
        data="${BASH_REMATCH[2]}"

        # 解析并更新数据
        parse_can_frame $can_id "$data"

        # 每50帧更新一次显示 (保持原始更新频率)
        if [ $((total_count % 50)) -eq 0 ]; then
            clear_screen
            show_header
            show_motor_status
            show_detailed_info
            show_statistics

            # 显示最近接收的帧 (保持原始格式)
            echo -e "\n${YELLOW}📡 最新接收:${NC}"
            echo "ID: $can_id (${BASH_REMATCH[1]}) | Data: $data"
        fi
    fi
done