#!/bin/bash

# 测试DM电机CAN数据解析
# 模拟不同类型的CAN帧数据来测试解析功能

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m'

# 解析DM电机CAN帧数据 (基于ludan_control_board的解析)
parse_dm_frame() {
    local can_id=$1
    local data=$2

    echo -e "${CYAN}解析电机 $can_id 的CAN帧数据:${NC}"
    echo -e "${BLUE}原始数据: $data${NC}"

    # 解析数据 (8字节)
    IFS=' ' read -r -a bytes <<< "$data"
    echo -e "${BLUE}字节数量: ${#bytes[@]}${NC}"

    if [ ${#bytes[@]} -ge 6 ]; then
        # DM电机反馈数据格式 (基于dm4310_fbdata_test):
        # Byte 0: Error Code (4 bits) + Motor ID (4 bits)
        # Byte 1: Position high byte
        # Byte 2: Position low byte
        # Byte 3: Velocity high byte
        # Byte 4: Velocity low 4 bits + Torque high 4 bits
        # Byte 5: Torque low byte
        # Byte 6: MOS temperature
        # Byte 7: Coil temperature

        echo -e "\n${YELLOW}详细解析:${NC}"

        # 解析错误码和电机ID (字节0)
        local error_code=$((0x${bytes[0]} >> 4))
        local motor_id_fb=$((0x${bytes[0]} & 0x0F))
        echo -e "  字节0: 错误码=$error_code, 电机ID=$motor_id_fb"

        # 位置 (字节1-2, 16位有符号整数)
        local pos_high=$((0x${bytes[1]}))
        local pos_low=$((0x${bytes[2]}))
        local pos_int=$(((pos_high << 8) | pos_low))
        echo -e "  位置原始值: $pos_int (0x${bytes[1]}${bytes[2]})"

        # 处理有符号数
        if [ $pos_int -gt 32767 ]; then
            pos_int=$((pos_int - 65536))
        fi

        # 转换为rad (DM4310: P_MIN1 = -12.5, P_MAX1 = 12.5)
        local pos_float=$(echo "scale=4; $pos_int * 25.0 / 65536 - 12.5" | bc -l 2>/dev/null)
        echo -e "  ${GREEN}位置: $pos_float rad${NC}"

        # 速度 (字节3和字节4的高4位, 12位有符号整数)
        local vel_high=$((0x${bytes[3]}))
        local vel_mid=$(((0x${bytes[4]} & 0xF0) >> 4))
        local vel_int=$(((vel_high << 4) | vel_mid))
        echo -e "  速度原始值: $vel_int (字节3=0x${bytes[3]}, 字节4高4位=0x${bytes[4]})"

        # 处理有符号数 (12位)
        if [ $vel_int -gt 2047 ]; then
            vel_int=$((vel_int - 4096))
        fi

        # 转换为rad/s (DM4310: V_MIN1 = -30.0, V_MAX1 = 30.0)
        local vel_float=$(echo "scale=4; $vel_int * 60.0 / 4096 - 30.0" | bc -l 2>/dev/null)
        echo -e "  ${GREEN}速度: $vel_float rad/s${NC}"

        # 力矩 (字节4的低4位和字节5, 12位有符号整数)
        local torque_high=$(((0x${bytes[4]} & 0x0F)))
        local torque_low=$((0x${bytes[5]}))
        local torque_int=$(((torque_high << 8) | torque_low))
        echo -e "  力矩原始值: $torque_int (字节4低4位=0x${torque_high}, 字节5=0x${bytes[5]})"

        # 处理有符号数 (12位)
        if [ $torque_int -gt 2047 ]; then
            torque_int=$((torque_int - 4096))
        fi

        # 转换为N·m (DM4310: T_MIN1 = -10.0, T_MAX1 = 10.0)
        local torque_float=$(echo "scale=4; $torque_int * 20.0 / 4096 - 10.0" | bc -l 2>/dev/null)
        echo -e "  ${GREEN}力矩: $torque_float N·m${NC}"

        # 温度
        if [ ${#bytes[@]} -ge 7 ]; then
            local mos_temp=$((0x${bytes[6]}))
            echo -e "  ${MAGENTA}MOS温度: $mos_temp°C${NC}"
        fi

        # 错误状态
        case $error_code in
            "0") echo -e "  ${GREEN}状态: 正常${NC}" ;;
            "1") echo -e "  ${YELLOW}状态: 过压${NC}" ;;
            "2") echo -e "  ${YELLOW}状态: 欠压${NC}" ;;
            "3") echo -e "  ${RED}状态: 过流${NC}" ;;
            "4") echo -e "  ${RED}状态: 过温${NC}" ;;
            "5") echo -e "  ${MAGENTA}状态: 磁编码器错误${NC}" ;;
            "7") echo -e "  ${RED}状态: 位置错误${NC}" ;;
            *) echo -e "  ${YELLOW}状态: 未知错误 $error_code${NC}" ;;
        esac

    else
        echo -e "${RED}错误: 数据长度不足 (${#bytes[@]} 字节)${NC}"
    fi
    echo ""
}

echo -e "${CYAN}${BOLD}"
echo "╔══════════════════════════════════════════════════════════════════════╗"
echo "║                    DM电机CAN数据解析测试器                         ║"
echo "╚══════════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

echo -e "${YELLOW}测试案例 1: 正常运行的电机${NC}"
parse_dm_frame "1" "00 80 00 88 40 00 2A 1C"

echo -e "${YELLOW}测试案例 2: 位置为正的电机${NC}"
parse_dm_frame "2" "00 FF FF CC 80 00 35 20"

echo -e "${YELLOW}测试案例 3: 速度为负的电机${NC}"
parse_dm_frame "3" "01 00 00 40 3F FF 28 1F"

echo -e "${YELLOW}测试案例 4: 过温错误${NC}"
parse_dm_frame "4" "04 12 34 56 78 90 45 30"

echo -e "${YELLOW}测试案例 5: 过流错误${NC}"
parse_dm_frame "5" "03 AB CD EF 12 34 50 25"

echo -e "${YELLOW}测试案例 6: 你提供的实际数据 (全0x11)${NC}"
parse_dm_frame "6" "11 11 11 11 11 11 11 11"

echo -e "${CYAN}${BOLD}解析说明:${NC}"
echo -e "• 基于ludan_control_board中的dm4310_fbdata_test函数实现"
echo -e "• 支持DM4310/DM4340等多种DM电机型号"
echo -e "• 实时显示位置(rad)、速度(rad/s)、力矩(N·m)"
echo -e "• 自动检测电机错误状态并彩色显示"
echo -e "• 适用于Ludan控制板的CAN总线数据监控"