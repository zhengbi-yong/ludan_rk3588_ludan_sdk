#!/bin/bash

# CAN日志分析器 - 分析上传的CAN日志文件
# 计算各电机ID的信号频率并显示详细数据

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# 默认参数
LOG_FILE="$1"
MAX_MOTORS=${2:-20}
DISPLAY_TOP=${3:-10}

# 检查参数
if [ -z "$LOG_FILE" ]; then
    echo -e "${RED}错误: 请指定CAN日志文件${NC}"
    echo -e "用法: $0 <日志文件> [最大电机数] [显示数量]"
    echo -e "示例: $0 candump-2025-12-18_182133.log"
    exit 1
fi

if [ ! -f "$LOG_FILE" ]; then
    echo -e "${RED}错误: 文件 $LOG_FILE 不存在${NC}"
    exit 1
fi

# 初始化数据结构
declare -A motor_count
declare -A motor_first_time
declare -A motor_last_time
declare -A motor_positions
declare -A motor_velocities
declare -A motor_states
declare -A motor_freq
declare -A motor_avg_pos
declare -A motor_avg_vel
declare -A motor_active_status

# 初始化
for ((i=1; i<=MAX_MOTORS; i++)); do
    motor_count[$i]=0
    motor_freq[$i]=0
    motor_active_status[$i]="inactive"
done

total_frames=0
first_timestamp=""
last_timestamp=""

echo -e "${CYAN}${BOLD}"
echo "╔════════════════════════════════════════════════════════════════════════╗"
echo "║                      CAN日志分析器                                 ║"
printf "║ 日志文件: %-55s ║\n" "$LOG_FILE"
echo "╚════════════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

echo -e "${BLUE}正在分析日志文件...${NC}"

# 解析日志文件
while IFS= read -r line; do
    # 跳过空行和注释
    [[ -z "$line" || "$line" =~ ^# ]] && continue

    # 解析日志行格式: (timestamp) interface ID [DLC] data
    if [[ $line =~ ^\(([^)]+)\)\ [^ ]+\ ([0-9A-Fa-f]+)\ \[[0-9]+\]\ (.+)$ ]]; then
        local timestamp="${BASH_REMATCH[1]}"
        local can_id="16#${BASH_REMATCH[2]}"
        can_id=$((can_id))
        local data="${BASH_REMATCH[3]}"

        # 记录时间范围
        if [ -z "$first_timestamp" ]; then
            first_timestamp="$timestamp"
        fi
        last_timestamp="$timestamp"

        # 只处理电机ID范围内的帧
        if [ $can_id -le $MAX_MOTORS ]; then
            total_frames=$((total_frames + 1))
            motor_count[$can_id]=$((motor_count[$can_id] + 1))

            # 记录首末时间
            if [ -z "${motor_first_time[$can_id]}" ]; then
                motor_first_time[$can_id]="$timestamp"
            fi
            motor_last_time[$can_id]="$timestamp"

            # 解析CAN数据
            IFS=' ' read -r -a bytes <<< "$data"
            if [ ${#bytes[@]} -ge 4 ]; then
                # 解析位置 (字节1-2, little-endian)
                local pos_low=$((0x${bytes[1]}))
                local pos_high=$((0x${bytes[2]}))
                local pos_int=$((pos_low + pos_high * 256))
                if [ $pos_int -gt 32767 ]; then
                    pos_int=$((pos_int - 65536))
                fi
                local pos_float=$(echo "scale=4; $pos_int * 3.14159 / 32767" | bc -l 2>/dev/null)

                # 累积位置用于计算平均值
                local current_sum="${motor_positions[$can_id]:-0}"
                motor_positions[$can_id]=$(echo "$current_sum + $pos_float" | bc -l)

                # 解析速度 (如果有的话)
                if [ ${#bytes[@]} -ge 5 ]; then
                    local vel_low=$((0x${bytes[3]}))
                    local vel_high=$((0x${bytes[4]}))
                    local vel_int=$((vel_low + vel_high * 256))
                    if [ $vel_int -gt 32767 ]; then
                        vel_int=$((vel_int - 65536))
                    fi
                    local vel_float=$(echo "scale=4; $vel_int * 10.0 / 32767" | bc -l 2>/dev/null)

                    local current_vel_sum="${motor_velocities[$can_id]:-0}"
                    motor_velocities[$can_id]=$(echo "$current_vel_sum + $vel_float" | bc -l)
                fi

                # 解析状态 (字节0)
                if [ ${#bytes[@]} -ge 1 ]; then
                    local state_int=$((0x${bytes[0]}))
                    motor_states[$can_id]="$state_int"
                fi

                # 标记为活跃
                motor_active_status[$can_id]="active"
            fi
        fi
    fi
done < "$LOG_FILE"

echo -e "${GREEN}✓ 日志解析完成${NC}"
echo ""

# 计算频率和统计数据
for ((i=1; i<=MAX_MOTORS; i++)); do
    if [ ${motor_count[$i]} -gt 0 ]; then
        # 计算持续时间 (秒)
        local duration=$(echo "${motor_last_time[$i]} - ${motor_first_time[$i]}" | bc -l 2>/dev/null)

        # 计算频率
        if (( $(echo "$duration > 0" | bc -l) )); then
            motor_freq[$i]=$(echo "scale=2; ${motor_count[$i]} / $duration" | bc -l)
        fi

        # 计算平均值
        if [ ${motor_count[$i]} -gt 0 ]; then
            motor_avg_pos[$i]=$(echo "scale=4; ${motor_positions[$i]} / ${motor_count[$i]}" | bc -l)
            if [ -n "${motor_velocities[$i]}" ]; then
                motor_avg_vel[$i]=$(echo "scale=4; ${motor_velocities[$i]} / ${motor_count[$i]}" | bc -l)
            fi
        fi
    fi
done

# 显示统计摘要
show_summary() {
    echo -e "${CYAN}${BOLD}📊 分析摘要${NC}"
    echo -e "${CYAN}──────────────────────────────────────────────────────${NC}"

    # 时间范围
    echo -e "📅 时间范围: ${GREEN}$first_timestamp${NC} 到 ${GREEN}$last_timestamp${NC}"

    # 活跃电机
    local active_count=0
    for ((i=1; i<=MAX_MOTORS; i++)); do
        if [ "${motor_active_status[$i]}" = "active" ]; then
            active_count=$((active_count + 1))
        fi
    done
    echo -e "🤖 活跃电机: ${YELLOW}$active_count${NC} / $MAX_MOTORS"

    # 总帧数
    echo -e "📦 总帧数: ${BLUE}$total_frames${NC}"

    # 时间跨度
    if [ -n "$first_timestamp" ] && [ -n "$last_timestamp" ]; then
        local total_duration=$(echo "$last_timestamp - $first_timestamp" | bc -l 2>/dev/null)
        echo -e "⏱️  时间跨度: ${MAGENTA}${total_duration}${NC} 秒"

        if (( $(echo "$total_duration > 0" | bc -l) )); then
            local overall_freq=$(echo "scale=2; $total_frames / $total_duration" | bc -l)
            echo -e "🔄 平均频率: ${CYAN}${overall_freq}${NC} Hz"
        fi
    fi
}

# 显示电机详情表格
show_motor_details() {
    echo ""
    echo -e "${CYAN}${BOLD}🔧 电机详细信息 (按频率排序)${NC}"
    echo -e "${CYAN}┌─────────────┬─────────┬──────────┬─────────────┬─────────────┬─────────────┬─────────────┐${NC}"
    echo -e "${CYAN}│   电机ID    │  帧数   │ 频率(Hz) │   状态      │   平均位置  │  平均速度  │  活跃状态  │${NC}"
    echo -e "${CYAN}├─────────────┼─────────┼──────────┼─────────────┼─────────────┼─────────────┼─────────────┤${NC}"

    # 创建排序数组
    local sorted_motors=()
    for ((i=1; i<=MAX_MOTORS; i++)); do
        sorted_motors+=("$i:${motor_freq[$i]}")
    done

    # 按频率排序 (降序)
    IFS=$'\n' sorted_motors=($(sort -t: -k2 -nr <<<"${sorted_motors[*]}"))

    # 显示前DISPLAY_TOP个电机
    for ((i=0; i<DISPLAY_TOP && i<${#sorted_motors[@]}; i++)); do
        local motor_id="${sorted_motors[$i]%%:*}"
        local freq="${sorted_motors[$i]##*:}"
        local count=${motor_count[$motor_id]}
        local state=${motor_states[$motor_id]:-"---"}
        local avg_pos=${motor_avg_pos[$motor_id]:-"---"}
        local avg_vel=${motor_avg_vel[$motor_id]:-"---"}
        local status=${motor_active_status[$motor_id]}

        # 状态显示
        case $state in
            "0") state_color="${RED}"; state_text="DISABLED${NC}" ;;
            "1") state_color="${GREEN}"; state_text="ENABLED${NC}" ;;
            "2") state_color="${YELLOW}"; state_text="STANDBY${NC}" ;;
            *) state_color="${MAGENTA}"; state_text="$state" ;;
        esac

        # 活跃状态颜色
        local row_color="${RED}"
        if [ "$status" = "active" ]; then
            row_color="${GREEN}"
        fi

        printf "${row_color}│ %11d │ %7d │ %8.2f │ %-11s${NC} │ ${state_color}%11s │ %11s │ %11s │${NC}\n" \
            "$motor_id" "$count" "$freq" "$state_text" "$avg_pos" "$avg_vel" "$status"
    done

    echo -e "${CYAN}└─────────────┴─────────┴──────────┴─────────────┴─────────────┴─────────────┴─────────────┘${NC}"
}

# 显示频率分布
show_frequency_distribution() {
    echo ""
    echo -e "${YELLOW}${BOLD}📈 频率分布${NC}"
    echo -e "${YELLOW}────────────────────${NC}"

    local high_freq=0
    local medium_freq=0
    local low_freq=0
    local no_freq=0

    for ((i=1; i<=MAX_MOTORS; i++)); do
        local freq=${motor_freq[$i]}
        if (( $(echo "$freq > 100" | bc -l) )); then
            high_freq=$((high_freq + 1))
        elif (( $(echo "$freq > 10" | bc -l) )); then
            medium_freq=$((medium_freq + 1))
        elif (( $(echo "$freq > 0" | bc -l) )); then
            low_freq=$((low_freq + 1))
        else
            no_freq=$((no_freq + 1))
        fi
    done

    echo -e "${GREEN}>100Hz: ${high_freq} 个电机${NC}"
    echo -e "${YELLOW}10-100Hz: ${medium_freq} 个电机${NC}"
    echo -e "${BLUE}0-10Hz: ${low_freq} 个电机${NC}"
    echo -e "${RED}0Hz: ${no_freq} 个电机${NC}"
}

# 运行分析
show_summary
show_motor_details
show_frequency_distribution

echo ""
echo -e "${GREEN}${BOLD}✓ 分析完成${NC}"
echo -e "${BLUE}提示: 可以调整参数来显示更多电机或更改分析范围${NC}"