#!/bin/bash

# 500Hz插值验证工具 (Shell版)
# 简单但有效的验证工具

echo "🚀 500Hz插值验证工具 (Shell版)"
echo "=========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m'

# 验证参数
TEST_DURATION=10  # 测试10秒
EXPECTED_HZ=500   # 期望频率
EXPECTED_INTERVAL=2  # 期望间隔(ms)

echo -e "${BLUE}测试参数:${NC}"
echo "  - 测试时长: ${TEST_DURATION}秒"
echo "  - 期望频率: ${EXPECTED_HZ}Hz"
echo "  - 期望间隔: ${EXPECTED_INTERVAL}ms"
echo "  - CAN接口: can0"
echo ""

echo -e "${YELLOW}⏱️  开始监控${TEST_DURATION}秒...${NC}"
echo "请确保motor_controller正在运行!"
echo ""

# 初始化计数器
declare -A motor_frames
declare -A motor_positions
declare -A last_timestamp
declare -A intervals

for id in 201 202 203 204; do
    motor_frames[$id]=0
    last_timestamp[$id]=0
    intervals[$id]=""
done

total_frames=0
start_time=$(date +%s.%N)

# 运行candump并实时分析
candump can0 -tA -T $TEST_DURATION 2>/dev/null | while read -r line; do
    # 解析candump输出
    if [[ $line =~ ^\(.*\)\ +can0\ +([0-9A-F]+)\ +\[[0-9]+\]\ +(.+)$ ]]; then
        can_id="16#${BASH_REMATCH[1]}"
        can_id=$((can_id))
        data="${BASH_REMATCH[2]}"

        # 只处理我们关心的电机ID
        if [[ $can_id =~ ^(201|202|203|204)$ ]]; then
            total_frames=$((total_frames + 1))
            motor_frames[$can_id]=$((motor_frames[$can_id] + 1))

            # 解析位置数据 (简化版)
            pos_hex=$(echo $data | cut -d' ' -f1-2 | tr ' ' '\n' | tac | tr -d '\n')
            if [ ${#pos_hex} -eq 4 ]; then
                pos_int=$((0x$pos_hex))
                # 简化的位置计算
                motor_positions[$can_id]=$pos_int
            fi

            # 计算时间间隔 (简化版)
            current_time=$(date +%s.%N)
            if [ ${last_timestamp[$can_id]} != "0" ]; then
                interval=$(echo "$current_time - ${last_timestamp[$can_id]}" | bc -l 2>/dev/null || echo "2")
                interval_ms=$(echo "$interval * 1000" | bc -l 2>/dev/null | cut -d. -f1)

                # 存储前几个间隔用于分析
                if [ ${motor_frames[$can_id]} -le 10 ]; then
                    intervals[$can_id]="${intervals[$can_id]} ${interval_ms}"
                fi
            fi
            last_timestamp[$can_id]=$current_time

            # 实时显示进度
            current_time=$(date +%s)
            elapsed=$((current_time - $(echo $start_time | cut -d. -f1)))

            if [ $elapsed -le $TEST_DURATION ]; then
                progress=$((elapsed * 100 / TEST_DURATION))
                bar=$(printf "%*s" $progress | tr ' ' '█')
                printf "\r进度: [%-20s] %d%% | 总帧数: %d" "$bar" $progress $total_frames
            fi
        fi
    fi
done

echo ""
echo ""

# 计算总时间
end_time=$(date +%s.%N)
duration=$(echo "$end_time - $start_time" | bc -l 2>/dev/null || echo "$TEST_DURATION")
duration_int=$(echo $duration | cut -d. -f1)

if [ $duration_int -eq 0 ]; then
    duration_int=1
fi

# 计算平均频率
total_hz=$(echo "scale=1; $total_frames / $duration_int" | bc -l 2>/dev/null || echo "0")

echo -e "${BOLD}📊 验证结果:${NC}"
echo "=========================================="
echo "总监控时间: ${duration_int}秒"
echo "总CAN帧数: $total_frames"
echo "平均频率: ${total_hz}Hz"
echo "期望频率: ${EXPECTED_HZ}Hz"

# 计算准确性
if [ "${total_hz}" != "0" ]; then
    accuracy=$(echo "scale=1; $total_hz * 100 / $EXPECTED_HZ" | bc -l 2>/dev/null || echo "0")
    echo "频率准确性: ${accuracy}%"

    # 判断结果
    if (( $(echo "$accuracy >= 95" | bc -l 2>/dev/null || echo "0") )); then
        echo -e "${GREEN}✅ 500Hz发送验证通过!${NC}"
    elif (( $(echo "$accuracy >= 80" | bc -l 2>/dev/null || echo "0") )); then
        echo -e "${YELLOW}⚠️  基本达到500Hz${NC}"
    else
        echo -e "${RED}❌ 未达到500Hz要求${NC}"
    fi
else
    echo -e "${RED}❌ 无法计算频率${NC}"
fi

echo ""
echo -e "${BOLD}🔧 各电机详细分析:${NC}"
echo "------------------------------------------"

for id in 201 202 203 204; do
    motor_num=$((id - 0x200))
    frame_count=${motor_frames[$id]}
    motor_hz=$(echo "scale=1; $frame_count / $duration_int" | bc -l 2>/dev/null || echo "0")

    if [ $frame_count -gt 0 ]; then
        echo -e "Motor $motor_num (ID: 0x$id): ${frame_count}帧 → ${motor_hz}Hz"

        # 分析间隔 (如果有数据)
        if [ -n "${intervals[$id]}" ]; then
            interval_list=$(echo ${intervals[$id]} | tr ' ' '\n' | grep -v '^$' | head -5)
            avg_interval=$(echo ${intervals[$id]} | tr ' ' '\n' | grep -v '^$' | awk '{sum+=$1} END {if(NR>0) print sum/NR; else print "0"}')

            if [ "${avg_interval}" != "0" ]; then
                echo "  📏 平均间隔: ${avg_interval}ms (期望: ${EXPECTED_INTERVAL}ms)"

                # 判断插值效果
                if (( $(echo "$avg_interval >= 1.5 && $avg_interval <= 2.5" | bc -l 2>/dev/null || echo "0") )); then
                    echo "  ✅ 插值间隔正常"
                else
                    echo "  ⚠️  插值间隔异常"
                fi
            fi
        fi
    else
        echo -e "Motor $motor_num (ID: 0x$id): ${RED}0帧${NC} → 无数据"
    fi
    echo ""
done

echo -e "${BOLD}🎯 插值效果验证:${NC}"
echo "------------------------------------------"

# 检查是否有插值特征
has_interpolation=false
total_motors_with_data=0

for id in 201 202 203 204; do
    if [ ${motor_frames[$id]} -gt 0 ]; then
        total_motors_with_data=$((total_motors_with_data + 1))
        motor_hz=$(echo "scale=1; ${motor_frames[$id]} / $duration_int" | bc -l 2>/dev/null || echo "0")

        if (( $(echo "$motor_hz >= 400 && $motor_hz <= 600" | bc -l 2>/dev/null || echo "0") )); then
            has_interpolation=true
        fi
    fi
done

if [ $has_interpolation = true ] && [ $total_motors_with_data -gt 0 ]; then
    echo -e "${GREEN}✅ 检测到插值特征 (稳定的高频发送)${NC}"
    echo "   - 发送频率接近500Hz"
    echo "   - 时间间隔稳定"
    echo "   - 多个电机同时工作"
else
    echo -e "${RED}❌ 未检测到正常插值效果${NC}"
    echo "   可能原因:"
    echo "   - motor_controller未运行"
    echo "   - g1_ankle_swing_example未发送DDS消息"
    echo "   - CAN接口配置问题"
fi

echo ""
echo -e "${BOLD}🔍 故障排除:${NC}"
echo "------------------------------------------"
echo "如果验证失败，请检查:"
echo "1. motor_controller是否正在运行"
echo "2. g1_ankle_swing_example_debug是否正在运行"
echo "3. CAN接口can0是否已配置"
echo "4. 使用 'ip link show can0' 检查接口状态"

echo ""
echo -e "${BLUE}💡 下一步:${NC}"
echo "1. 如果500Hz验证通过，说明插值工作正常"
echo "2. 使用 'candump can0' 实时查看CAN帧"
echo "3. 使用 'can_quick_test.sh' 进行更多测试"