#!/bin/bash

# 简化的CAN日志分析器
# 可以处理空文件或实时监控CAN数据

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

LOG_FILE="$1"
MAX_MOTORS=${2:-20}

show_help() {
    echo -e "${CYAN}CAN日志分析器${NC}"
    echo -e "用法: $0 [日志文件] [最大电机数]"
    echo -e ""
    echo -e "功能:"
    echo -e "  - 分析现有CAN日志文件"
    echo -e "  - 显示各电机ID的消息频率"
    echo -e "  - 按电机ID分列显示状态和位置信息"
    echo -e ""
    echo -e "示例:"
    echo -e "  $0                    # 实时监控模式"
    echo -e "  $0 can.log           # 分析日志文件"
    echo -e "  $0 can.log 30        # 分析最多30个电机"
}

check_can_interface() {
    echo -e "${BLUE}检查CAN接口...${NC}"

    # 检查can-utils是否安装
    if ! command -v candump &> /dev/null; then
        echo -e "${RED}错误: candump未安装${NC}"
        echo -e "${YELLOW}安装方法: sudo apt-get install can-utils${NC}"
        exit 1
    fi

    # 查找可用的CAN接口
    local can_interfaces=$(ip link show | grep -o "can[0-9]*" | head -5)

    if [ -n "$can_interfaces" ]; then
        echo -e "${GREEN}发现CAN接口: $can_interfaces${NC}"
        for interface in $can_interfaces; do
            local status=$(ip link show "$interface" | grep -o "state [A-Z]*")
            echo -e "  $interface: ${status#state }"
        done
    else
        echo -e "${YELLOW}未发现CAN接口${NC}"
        echo -e "${BLUE}提示: 使用 'sudo ip link set can0 up type can bitrate 500000' 启动CAN接口${NC}"
    fi
}

analyze_log_file() {
    if [ ! -f "$LOG_FILE" ]; then
        echo -e "${RED}错误: 文件 $LOG_FILE 不存在${NC}"
        return 1
    fi

    local file_size=$(stat -f%z "$LOG_FILE" 2>/dev/null || stat -c%s "$LOG_FILE" 2>/dev/null)

    if [ "$file_size" -eq 0 ]; then
        echo -e "${YELLOW}⚠️  日志文件为空${NC}"
        echo -e "${BLUE}ℹ️  可能原因:${NC}"
        echo -e "  - CAN接口未启动"
        echo -e "  - 没有CAN数据传输"
        echo -e "  - 权限不足"

        echo -e "\n${CYAN}解决方案:${NC}"
        check_can_interface
        return 0
    fi

    echo -e "${GREEN}📊 分析文件: $LOG_FILE (${file_size} 字节)${NC}"
    echo -e "${CYAN}─────────────────────────────────────${NC}"

    # 简单的文本分析
    local total_lines=$(wc -l < "$LOG_FILE")
    local motor_lines=$(grep -E "^ *\([0-9]" "$LOG_FILE" | wc -l)

    echo -e "📋 总行数: ${BLUE}$total_lines${NC}"
    echo -e "🤖 CAN数据行: ${GREEN}$motor_lines${NC}"

    # 分析电机ID出现频率
    echo -e "\n${YELLOW}🔧 电机消息频率 (前10个):${NC}"
    echo -e "${CYAN}ID     消息数    频率估计${NC}"
    echo -e "${CYAN}──────────────────────${NC}"

    # 提取电机ID并计数
    awk '/[0-9]+\)\ +can[0-9]+\ +([0-9A-Fa-f]+)/ {print $3}' "$LOG_FILE" | \
    sed 's/^/#/' | sort | uniq -c | sort -nr | head -10 | while read count hex_id; do
        local motor_id=$((hex_id))
        printf "%3d   %6d    ~%3d Hz\n" "$motor_id" "$count" "$((count / 5))"
    done
}

start_real_time_monitor() {
    echo -e "${GREEN}🚀 启动实时CAN监控...${NC}"
    echo -e "${YELLOW}按 Ctrl+C 停止监控${NC}"
    echo ""

    # 找到可用的CAN接口
    local interface=$(ip link show | grep -o "can[0-9]*" | head -1)

    if [ -z "$interface" ]; then
        echo -e "${RED}错误: 未找到可用的CAN接口${NC}"
        exit 1
    fi

    echo -e "${BLUE}监控接口: $interface${NC}"

    # 简单的实时监控
    candump "$interface" -tA | while read -r line; do
        if [[ $line =~ ^\([0-9].*\)\ +$interface\ +([0-9A-F]+) ]]; then
            local timestamp="${line#(*}"
            timestamp="${timestamp%)*}"
            local can_id="16#${BASH_REMATCH[2]}"
            local motor_id=$((can_id))

            printf "${GREEN}[%s]${NC} ${CYAN}电机 %3d${NC} ${YELLOW}%s${NC}\n" \
                "$(date '+%H:%M:%S')" "$motor_id" "$line"
        fi
    done
}

main() {
    echo -e "${CYAN}CAN信号分析器${NC}"
    echo -e "${CYAN}================${NC}"
    echo ""

    # 检查帮助参数
    if [[ "$1" == "-h" || "$1" == "--help" ]]; then
        show_help
        exit 0
    fi

    if [ -n "$LOG_FILE" ]; then
        # 分析日志文件模式
        echo -e "${BLUE}日志分析模式${NC}"
        analyze_log_file
    else
        # 实时监控模式
        echo -e "${BLUE}实时监控模式${NC}"
        check_can_interface
        echo ""
        read -p "是否开始实时监控? (y/N): " -n 1 -r
        echo ""
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            start_real_time_monitor
        else
            echo -e "${YELLOW}退出${NC}"
        fi
    fi
}

main "$@"