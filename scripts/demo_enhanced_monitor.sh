#!/bin/bash

# 演示增强版CAN监控器的功能
# 模拟CAN数据流来展示格式化效果

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
NC='\033[0m'

# 模拟CAN数据
simulate_can_data() {
    echo -e "${GREEN}🚀 启动CAN电机监控器...${NC}"
    echo -e "${BLUE}监控接口: can0${NC}"
    echo -e "${YELLOW}模拟数据模式 - 按 Ctrl+C 退出${NC}"
    echo ""

    local frame_count=0

    while true; do
        frame_count=$((frame_count + 1))

        # 模拟不同电机的数据
        local motor_id=$(( (frame_count % 4) + 201 ))
        local timestamp=$(date '+%H:%M:%S.%3N')

        # 生成模拟数据
        case $((frame_count % 4)) in
            0) data="00 80 00 88 40 00 2A 1C" ;;  # 电机1正常
            1) data="00 FF FF CC 80 00 35 20" ;;  # 电机2位置正
            2) data="01 00 00 40 3F FF 28 1F" ;;  # 电机3速度负
            3) data="04 12 34 56 78 90 45 30" ;;  # 电机4错误
        esac

        echo "($timestamp) can0  $(printf "%03X" $motor_id) [8] $data"

        # 每10帧显示一次格式化输出
        if [ $((frame_count % 10)) -eq 0 ]; then
            echo ""
            echo -e "${CYAN}${BOLD}"
            echo "╔════════════════════════════════════════════════════════════════╗"
            echo "║                    CAN电机命令监控器                        ║"
            printf "║ 总帧数: %-8d 总频率: %-6d Hz 运行时间: %-5ds            ║\n" $frame_count $((frame_count / 5)) 5
            echo "╚════════════════════════════════════════════════════════════════╝"
            echo -e "${NC}"

            echo -e "${CYAN}┌─────────────────────┬─────────┬──────────┬─────────────┬─────────────┬─────────────┐${NC}"
            echo -e "${CYAN}│      电机ID        │  帧数   │  频率(Hz) │   位置(rad)  │  速度(rad/s)│   力矩(N·m) │${NC}"
            echo -e "${CYAN}├─────────────────────┼─────────┼──────────┼─────────────┼─────────────┼─────────────┤${NC}"

            # 模拟电机1数据
            local count1=$(((frame_count + 3) / 4))
            local pos1=$(echo "scale=3; -12.5 + ($frame_count % 10) * 2.5" | bc -l)
            local vel1=$(echo "scale=3; -30 + ($frame_count % 20) * 3" | bc -l)
            local tau1=$(echo "scale=3; -10 + ($frame_count % 40) * 0.5" | bc -l)
            printf "${GREEN}│ %-17s │ %7d │ %8d │ %11s │ %11s │ %11s │${NC}\n" \
                "Motor 1" $count1 $((count1 / 5)) "$pos1" "$vel1" "$tau1"

            # 模拟电机2数据
            local count2=$(((frame_count + 2) / 4))
            local pos2=$(echo "scale=3; 0 + ($frame_count % 15) * 1.5" | bc -l)
            local vel2=$(echo "scale=3; 20 + ($frame_count % 25) * 2" | bc -l)
            local tau2=$(echo "scale=3; 5 + ($frame_count % 30) * 0.3" | bc -l)
            printf "${GREEN}│ %-17s │ %7d │ %8d │ %11s │ %11s │ %11s │${NC}\n" \
                "Motor 2" $count2 $((count2 / 5)) "$pos2" "$vel2" "$tau2"

            # 模拟电机3数据
            local count3=$(((frame_count + 1) / 4))
            local pos3=$(echo "scale=3; 10 - ($frame_count % 20) * 1" | bc -l)
            local vel3=$(echo "scale=3; -15 + ($frame_count % 35) * 1.5" | bc -l)
            local tau3=$(echo "scale=3; -5 + ($frame_count % 25) * 0.8" | bc -l)
            printf "${GREEN}│ %-17s │ %7d │ %8d │ %11s │ %11s │ %11s │${NC}\n" \
                "Motor 3" $count3 $((count3 / 5)) "$pos3" "$vel3" "$tau3"

            # 模拟电机4数据 (有错误)
            local count4=$((frame_count / 4))
            local pos4=$(echo "scale=3; -8 + ($frame_count % 12) * 2" | bc -l)
            printf "${RED}│ %-17s │ %7d │ %8d │ %11s │ %11s │ %11s │${NC}\n" \
                "Motor 4" $count4 $((count4 / 5)) "$pos4" "---" "---"

            echo -e "${CYAN}└─────────────────────┴─────────┴──────────┴─────────────┴─────────────┴─────────────┘${NC}"

            echo -e "\n${YELLOW}📊 详细电机信息:${NC}"
            echo -e "${CYAN}┌─────────────────────┬─────────────┬─────────────┬─────────────────┐${NC}"
            echo -e "${CYAN}│      电机ID        │     温度    │     状态    │   最后更新时间  │${NC}"
            echo -e "${CYAN}├─────────────────────┼─────────────┼─────────────┼─────────────────┤${NC}"
            printf "│ %-17s │ %11s │ %11s │ %15s │\n" "Motor 1" "45°C" "OK" "$(date '+%H:%M:%S')"
            printf "│ %-17s │ %11s │ %11s │ %15s │\n" "Motor 2" "52°C" "OK" "$(date '+%H:%M:%S')"
            printf "│ %-17s │ %11s │ %11s │ %15s │\n" "Motor 3" "38°C" "OK" "$(date '+%H:%M:%S')"
            printf "│ %-17s │ %11s │ %11s │ %15s │\n" "Motor 4" "75°C" "ERROR" "$(date '+%H:%M:%S')"
            echo -e "${CYAN}└─────────────────────┴─────────────┴─────────────┴─────────────────┘${NC}"

            echo -e "\n${MAGENTA}📈 统计信息:${NC}"
            echo -e "  活跃电机: ${GREEN}3${NC}/4"
            echo -e "  平均频率: ${BLUE}$((frame_count / 5))${NC} Hz"
            echo -e "  数据完整性: ${YELLOW}$((frame_count % 100))${NC}%"

            echo -e "\n${YELLOW}📡 最新接收:${NC}"
            echo "ID: $motor_id ($(printf "%03X" $motor_id)) | Data: $data"

            echo -e "\n${BLUE}💡 提示: 这是在演示增强版CAN监控器的格式化功能${NC}"
            echo -e "${BLUE}   实际使用时会显示真实的CAN总线数据${NC}"
        fi

        sleep 0.2
    done
}

echo -e "${CYAN}${BOLD}"
echo "╔══════════════════════════════════════════════════════════════════════╗"
echo "║                增强版CAN监控器演示                            ║"
echo "║           基于原始can_monitor.sh，添加更多格式化内容              ║"
echo "╚══════════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

echo -e "${GREEN}✨ 增强功能:${NC}"
echo -e "  • 保持原始can_monitor.sh的结构和风格"
echo -e "  • 增加速度(rad/s)和力矩(N·m)显示"
echo -e "  • 添加温度和状态监控"
echo -e "  • 增强统计信息显示"
echo -e "  • 支持DM电机数据格式解析"
echo -e ""

simulate_can_data