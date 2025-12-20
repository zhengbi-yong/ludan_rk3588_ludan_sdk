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
declare -A motor_data_type  # 存储数据类型 (CMD/FEEDBACK/DATA)

# 初始化原始的4个电机 (1-4，对应实际的CAN ID)
motor_count[1]=0
motor_count[2]=0
motor_count[3]=0
motor_count[4]=0

start_time=$(date +%s)
total_count=0
last_display_time=$start_time  # 记录上次显示时间

# 清屏函数
clear_screen() {
    echo -e "\033[2J\033[H"
}

# 显示标题 (保持原始风格)
show_header() {
    local current_time=$(date +%s)
    local elapsed=$((current_time - start_time))
    local total_hz=0

    # More accurate frequency calculation
    if [ $elapsed -gt 0 ]; then
        total_hz=$(echo "scale=1; $total_count / $elapsed" | bc -l 2>/dev/null)
        # If bc fails, fall back to integer arithmetic
        if [ -z "$total_hz" ]; then
            total_hz=$((total_count / elapsed))
        fi
    fi

    echo -e "${CYAN}${BOLD}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║                    CAN电机反馈监控器                        ║"
    printf "║ 总帧数: %-8d 总频率: %-6s Hz 运行时间: %-5ds            ║\n" $total_count "$total_hz" $elapsed
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

# 显示增强版电机状态表
show_motor_status() {
    echo -e "${CYAN}┌─────────────────────┬─────────┬──────────┬─────────────┬─────────────┬─────────────┬─────────────┐${NC}"
    echo -e "${CYAN}│      电机ID        │  帧数   │  频率(Hz) │   位置(rad)  │  速度(rad/s)│   力矩(N·m) │    类型     │${NC}"
    echo -e "${CYAN}├─────────────────────┼─────────┼──────────┼─────────────┼─────────────┼─────────────┼─────────────┤${NC}"

    for id in 1 2 3 4; do
        local count=${motor_count[$id]}
        # Calculate frequency based on elapsed time
        local current_time=$(date +%s)
        local elapsed=$((current_time - start_time))
        local freq=0
        if [ $elapsed -gt 0 ] && [ $count -gt 0 ]; then
            freq=$(echo "scale=1; $count / $elapsed" | bc -l 2>/dev/null)
            if [ -z "$freq" ]; then
                freq=$((count / elapsed))
            fi
        fi

        local pos=${motor_last_pos[$id]:-"---"}
        local vel=${motor_last_vel[$id]:-"---"}
        local tau=${motor_last_tau[$id]:-"---"}
        local data_type=${motor_data_type[$id]:-"N/A"}
        local motor_name="Motor $id"

        # 根据状态选择颜色
        local state=${motor_last_state[$id]:-"N/A"}
        local color="${GREEN}"

        if [ $count -eq 0 ]; then
            color="${RED}"
        elif [ "$state" = "ERROR" ]; then
            color="${RED}"
        elif [ "$state" = "ENABLED" ]; then
            color="${GREEN}"
        elif [ "$state" = "DISABLED" ]; then
            color="${YELLOW}"
        elif [ "$data_type" = "FEEDBACK" ]; then
            color="${BLUE}"
        elif [ "$state" = "ID_MISMATCH" ]; then
            color="${MAGENTA}"
        elif [ "$state" = "UNKNOWN" ]; then
            color="${CYAN}"
        else
            color="${GREEN}"
        fi

        printf "${color}│ %-17s │ %7d │ %8s │ %11s │ %11s │ %11s │ %11s │${NC}\n" \
            "$motor_name" "$count" "$freq" "$pos" "$vel" "$tau" "$data_type"
    done

    echo -e "${CYAN}└─────────────────────┴─────────┴──────────┴─────────────┴─────────────┴─────────────┴─────────────┘${NC}"
}

# 显示详细信息部分
show_detailed_info() {
    echo -e "\n${YELLOW}📊 详细电机信息:${NC}"
    echo -e "${CYAN}┌─────────────────────┬─────────────┬─────────────┬─────────────────┐${NC}"
    echo -e "${CYAN}│      电机ID        │    错误码   │     状态    │   最后更新时间  │${NC}"
    echo -e "${CYAN}├─────────────────────┼─────────────┼─────────────┼─────────────────┤${NC}"

    for id in 1 2 3 4; do
        if [ ${motor_count[$id]} -gt 0 ]; then
            local error_desc=${motor_last_temp[$id]:-"N/A"}
            local state=${motor_last_state[$id]:-"N/A"}
            local motor_name="Motor $id"
            local last_time=${motor_last_time[$id]:-"N/A"}

            # 根据状态选择颜色
            local state_color="${GREEN}"
            if [ "$state" = "ERROR" ]; then
                state_color="${RED}"
            elif [ "$state" = "DISABLED" ]; then
                state_color="${YELLOW}"
            elif [ "$state" = "ENABLED" ]; then
                state_color="${GREEN}"
            elif [ "$state" = "ID_MISMATCH" ]; then
                state_color="${MAGENTA}"
            elif [ "$state" = "UNKNOWN" ]; then
                state_color="${CYAN}"
            else
                state_color="${GREEN}"
            fi

            printf "│ %-17s │ ${state_color}%11s${NC} │ ${state_color}%11s${NC} │ %15s │\n" \
                "$motor_name" "$error_desc" "$state" "$last_time"
        fi
    done

    echo -e "${CYAN}└─────────────────────┴─────────────┴─────────────┴─────────────────┘${NC}"
}

# 显示统计信息
show_statistics() {
    local active_motors=0
    local total_freq=0
    local current_time=$(date +%s)
    local elapsed_time=$((current_time - start_time))

    for id in 1 2 3 4; do
        if [ ${motor_count[$id]} -gt 0 ]; then
            active_motors=$((active_motors + 1))
            # Calculate individual motor frequency more accurately
            local motor_freq_current=0
            if [ $elapsed_time -gt 0 ]; then
                motor_freq_current=$(echo "scale=1; ${motor_count[$id]} / $elapsed_time" | bc -l 2>/dev/null)
                if [ -z "$motor_freq_current" ]; then
                    motor_freq_current=$((${motor_count[$id]} / elapsed_time))
                fi
            fi
            total_freq=$(echo "scale=1; $total_freq + $motor_freq_current" | bc -l 2>/dev/null)
        fi
    done

    # Calculate overall average frequency
    local avg_freq=0
    if [ $active_motors -gt 0 ]; then
        avg_freq=$(echo "scale=1; $total_freq / $active_motors" | bc -l 2>/dev/null)
        if [ -z "$avg_freq" ]; then
            avg_freq=$((total_freq / active_motors))
        fi
    fi

    echo -e "\n${MAGENTA}📈 统计信息:${NC}"
    echo -e "  活跃电机: ${GREEN}$active_motors${NC}/4"
    echo -e "  平均频率: ${BLUE}${avg_freq}${NC} Hz"
}

# 解析CAN帧数据 (专门处理电机反馈数据)
parse_can_frame() {
    local can_id=$1
    local data=$2

    # 只处理我们关心的电机ID (1-4，对应Motor 1-4)
    if [[ $can_id =~ ^([1-4])$ ]]; then
        # 更新计数
        motor_count[$can_id]=$((motor_count[$can_id] + 1))
        total_count=$((total_count + 1))

        # 更新最后接收时间
        motor_last_time[$can_id]=$(date '+%H:%M:%S')

        # 解析数据 - 专门处理电机反馈数据
        IFS=' ' read -r -a bytes <<< "$data"

        # 检查数据长度 - 电机反馈通常是8字节
        if [ ${#bytes[@]} -eq 8 ]; then
            # 8字节数据 - 电机反馈数据
            parse_feedback_data $can_id "${bytes[@]}"
        else
            # 其他长度的数据，尝试兼容性解析
            parse_simple_feedback_data $can_id "${bytes[@]}"
        fi
    fi
}

# 解析反馈数据 (8字节) - 根据C++代码的标准格式
parse_feedback_data() {
    local can_id=$1
    shift
    local bytes=("$@")

    # 存储数据类型
    motor_data_type[$can_id]="FEEDBACK"

    # 解析反馈帧第一字节：高4位是ID，低4位是错误码
    local motor_id_from_frame=$((16#${bytes[0]} >> 4))
    local error_code=$((16#${bytes[0]} & 0x0F))

    # 验证电机ID是否匹配 (CAN ID vs Frame ID)
    if [ $motor_id_from_frame -ne $can_id ]; then
        # ID不匹配，仍然显示数据但标记ID不匹配
        motor_last_state[$can_id]="ID_MISMATCH"
        motor_last_temp[$can_id]="F:$motor_id_from_frame"
    else
        # ID匹配，继续解析数据

        # 解析位置 (16-bit signed, little-endian, 字节1-2)
        local pos_int=$(((16#${bytes[2]} << 8) | 16#${bytes[1]}))
        if [ $pos_int -gt 32767 ]; then
            pos_int=$((pos_int - 65536))
        fi
        local pos_float=$(echo "scale=3; $pos_int * 12.5 / 32767" | bc -l 2>/dev/null)
        motor_last_pos[$can_id]=$pos_float

        # 解析速度 (12-bit signed, 字节2-3，其中字节3的高4位和字节2组合)
        local vel_int=$((((16#${bytes[3]} & 0x0F) << 8) | 16#${bytes[2]}))
        if [ $vel_int -gt 2047 ]; then
            vel_int=$((vel_int - 4096))
        fi
        local vel_float=$(echo "scale=3; $vel_int * 30.0 / 2047" | bc -l 2>/dev/null)
        motor_last_vel[$can_id]=$vel_float

        # 解析转矩 (12-bit signed, 字节4-5，其中字节4的高4位和字节5组合)
        local tau_int=$((((16#${bytes[4]} & 0x0F) << 8) | 16#${bytes[5]}))
        if [ $tau_int -gt 2047 ]; then
            tau_int=$((tau_int - 4096))
        fi
        local tau_float=$(echo "scale=3; $tau_int * 10.0 / 2047" | bc -l 2>/dev/null)
        motor_last_tau[$can_id]=$tau_float

        # 更新状态和错误信息
        if [ $error_code -eq 0 ]; then
            motor_last_state[$can_id]="DISABLED"
            motor_last_temp[$can_id]="0x0-失能"
        elif [ $error_code -eq 1 ]; then
            motor_last_state[$can_id]="ENABLED"
            motor_last_temp[$can_id]="0x1-使能"
        elif [ $error_code -eq 8 ]; then
            motor_last_state[$can_id]="ERROR"
            motor_last_temp[$can_id]="0x8-超压"
        elif [ $error_code -eq 9 ]; then
            motor_last_state[$can_id]="ERROR"
            motor_last_temp[$can_id]="0x9-欠压"
        elif [ $error_code -eq 10 ]; then
            motor_last_state[$can_id]="ERROR"
            motor_last_temp[$can_id]="0xA-过流"
        elif [ $error_code -eq 11 ]; then
            motor_last_state[$can_id]="ERROR"
            motor_last_temp[$can_id]="0xB-MOS过温"
        elif [ $error_code -eq 12 ]; then
            motor_last_state[$can_id]="ERROR"
            motor_last_temp[$can_id]="0xC-线圈过温"
        elif [ $error_code -eq 13 ]; then
            motor_last_state[$can_id]="ERROR"
            motor_last_temp[$can_id]="0xD-通讯丢失"
        elif [ $error_code -eq 14 ]; then
            motor_last_state[$can_id]="ERROR"
            motor_last_temp[$can_id]="0xE-过载"
        else
            motor_last_state[$can_id]="UNKNOWN"
            motor_last_temp[$can_id]="0x$error_code"
        fi
    fi
}

# 解析控制指令数据
parse_command_data() {
    local can_id=$1
    shift
    local bytes=("$@")

    # 存储数据类型
    motor_data_type[$can_id]="CMD"

    # 从你的输出 Data: 39 09 87 65 43 21 09 87 来分析
    # 这是一个8字节的数据，看起来像是控制指令

    # 尝试按照常见的控制指令格式解析：
    # 字节0-1: 目标位置
    # 字节2-3: 目标速度
    # 字节4-5: Kp增益
    # 字节6-7: Kd增益

    # 解析目标位置 (小端序，字节0-1)
    local pos_int=$(((16#${bytes[1]} << 8) | 16#${bytes[0]}))
    if [ $pos_int -gt 32767 ]; then
        pos_int=$((pos_int - 65536))
    fi
    local pos_float=$(echo "scale=3; $pos_int * 12.5 / 32767" | bc -l 2>/dev/null)
    motor_last_pos[$can_id]=$pos_float

    # 解析目标速度 (小端序，字节2-3)
    local vel_int=$(((16#${bytes[3]} << 8) | 16#${bytes[2]}))
    if [ $vel_int -gt 32767 ]; then
        vel_int=$((vel_int - 65536))
    fi
    local vel_float=$(echo "scale=3; $vel_int * 30.0 / 32767" | bc -l 2>/dev/null)
    motor_last_vel[$can_id]=$vel_float

    # 解析Kp增益 (小端序，字节4-5)
    local kp_int=$(((16#${bytes[5]} << 8) | 16#${bytes[4]}))
    local kp_float=$(echo "scale=2; $kp_int * 100.0 / 65535" | bc -l 2>/dev/null)

    # 解析Kd增益 (小端序，字节6-7)
    local kd_int=$(((16#${bytes[7]} << 8) | 16#${bytes[6]}))
    local kd_float=$(echo "scale=2; $kd_int * 10.0 / 65535" | bc -l 2>/dev/null)

    motor_last_tau[$can_id]="Kp:$kp_float Kd:$kd_float"

    # 设置状态为控制指令
    motor_last_state[$can_id]="CMD"
    motor_last_temp[$can_id]="---"
}

# 简化数据解析 (兼容性)
parse_simple_data() {
    local can_id=$1
    local data=$2

    # 存储数据类型
    motor_data_type[$can_id]="DATA"

    # 解析位置数据 (保持原始的简单解析)
    local pos_hex=$(echo $data | cut -d' ' -f1-2 | tr ' ' '\n' | tac | tr -d '\n')
    if [ ${#pos_hex} -eq 4 ]; then
        local pos_int=$((0x$pos_hex))
        if [ $pos_int -gt 32767 ]; then
            pos_int=$((pos_int - 65536))
        fi
        local pos_float=$(echo "scale=3; $pos_int * 12.5 / 32767" | bc -l 2>/dev/null)
        motor_last_pos[$can_id]=$pos_float
    fi

    IFS=' ' read -r -a bytes <<< "$data"

    # 解析速度数据
    if [ ${#bytes[@]} -ge 4 ]; then
        local vel_int=$((0x${bytes[3]}))
        if [ $vel_int -gt 127 ]; then
            vel_int=$((vel_int - 256))
        fi
        local vel_float=$(echo "scale=3; $vel_int * 0.5" | bc -l 2>/dev/null)
        motor_last_vel[$can_id]=$vel_float
    fi

    # 解析力矩数据
    if [ ${#bytes[@]} -ge 5 ]; then
        local tau_int=$((0x${bytes[4]}))
        if [ $tau_int -gt 127 ]; then
            tau_int=$((tau_int - 256))
        fi
        local tau_float=$(echo "scale=3; $tau_int * 0.1" | bc -l 2>/dev/null)
        motor_last_tau[$can_id]=$tau_float
    fi

    motor_last_state[$can_id]="DATA"
    motor_last_temp[$can_id]="---"
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

        # 每10帧更新一次显示 (提高更新频率)
        if [ $((total_count % 10)) -eq 0 ]; then
            clear_screen
            show_header
            show_motor_status
            show_detailed_info
            show_statistics

            # 显示最近接收的帧 (保持原始格式)
            echo -e "\n${YELLOW}📡 最新接收:${NC}"
            echo "ID: $can_id (0x$(printf '%02x' $can_id)) | Data: $data"
        fi

        # 每5帧显示实时CAN帧信息
        if [ $((total_count % 5)) -eq 0 ]; then
            # 移动光标到底部显示实时信息
            data_type=${motor_data_type[$can_id]:-"UNKNOWN"}
            state_info=""
            state_color="${CYAN}"

            if [ "$data_type" = "FEEDBACK" ]; then
                state_color="${BLUE}"
                state_info=" [反馈]"
                if [ ${motor_last_state[$can_id]} = "ERROR" ]; then
                    state_info="${state_info} [${RED}错误${state_color}]"
                elif [ ${motor_last_state[$can_id]} = "ENABLED" ]; then
                    state_info="${state_info} [${GREEN}正常${state_color}]"
                elif [ ${motor_last_state[$can_id]} = "DISABLED" ]; then
                    state_info="${state_info} [${YELLOW}失能${state_color}]"
                fi
            fi

            printf "\r${state_color}📡 实时: ID:$can_id 状态:${motor_last_state[$can_id]:-} $state_info ${NC}"
        fi
    fi
done