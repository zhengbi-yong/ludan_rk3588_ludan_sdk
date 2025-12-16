#!/bin/bash

# 快速CAN频率测试 - 简化版
# 10秒测试，显示各电机ID的发送频率

echo "🔍 快速CAN频率测试 (10秒)"
echo "================================"
echo ""

# 初始化计数器
for id in 201 202 203 204; do
    count[$id]=0
done

total_count=0

# 运行candump 10秒
echo "⏱️  正在监控10秒..."
candump can0 -tA -T 10000 2>/dev/null > /tmp/can_test.log

# 统计结果
echo ""
echo "📊 测试结果:"
echo "================================"

while read -r line; do
    if [[ $line =~ ^\([^)]+\)\ +can0\ +([0-9A-F]+) ]]; then
        can_id="16#${BASH_REMATCH[1]}"
        can_id=$((can_id))

        if [[ $can_id =~ ^(201|202|203|204)$ ]]; then
            count[$can_id]=$((count[$can_id] + 1))
            total_count=$((total_count + 1))
        fi
    fi
done < /tmp/can_test.log

# 显示结果
for id in 201 202 203 204; do
    motor_num=$((id - 0x200))
    frequency=$((count[$id] / 10))

    if [ ${count[$id]} -gt 0 ]; then
        echo -e "✅ Motor $motor_num (ID: 0x$id): ${count[$id]} 帧 → ${frequency} Hz"
    else
        echo -e "❌ Motor $motor_num (ID: 0x$id): ${count[$id]} 帧 → ${frequency} Hz"
    fi
done

echo ""
echo "📈 总计: $total_count 帧 → $((total_count / 10)) Hz"

# 清理
rm -f /tmp/can_test.log

echo ""
echo "💡 提示:"
echo "  - 期望频率: ~500 Hz (每个电机)"
echo "  - 如果频率为0，检查程序是否在运行"
echo "  - 使用 ./can_monitor.sh 进行实时监控"