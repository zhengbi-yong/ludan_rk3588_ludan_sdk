#!/bin/bash

echo "🔍 CAN帧频率测试"
echo "========================"
echo "接口: can0"
echo "测试时间: 5秒"
echo "正在分析..."

# 运行candump 5秒并分析结果
candump can0 -tA -T 5000 2>/dev/null > /tmp/can_frames.txt

# 统计总帧数
TOTAL_FRAMES=$(wc -l < /tmp/can_frames.txt)
echo ""
echo "📊 结果分析:"
echo "========================"

if [ $TOTAL_FRAMES -gt 0 ]; then
    # 计算总体频率
    TOTAL_HZ=$((TOTAL_FRAMES / 5))
    echo "总帧数: $TOTAL_FRAMES"
    echo "平均频率: $TOTAL_HZ Hz"

    # 按ID分析频率
    echo ""
    echo "📈 各CAN ID频率:"
    echo "------------------------"

    # 提取CAN ID并统计
    awk '{print $3}' /tmp/can_frames.txt | sort | uniq -c | sort -nr | while read count id; do
        id_hz=$((count / 5))
        echo "ID $id: $count frames → $id_hz Hz"
    done

    # 分析时间间隔
    echo ""
    echo "⏱️  时间间隔分析:"
    echo "------------------------"

    # 提取时间戳并计算间隔
    grep -o '([0-9.]* )' /tmp/can_frames.txt | sed 's/[() ]//g' > /tmp/timestamps.txt

    if [ -s /tmp/timestamps.txt ]; then
        # 计算相邻帧的时间差
        awk 'NR>1{delta=$1-prev; if(delta>0) print delta*1000} {prev=$1}' /tmp/timestamps.txt | \
        awk '{sum+=$1; count++; if($1<min || NR==1) min=$1; if($1>max) max=$1} END {
            if(count>0) {
                print "平均间隔: " sum/count " ms"
                print "最小间隔: " min " ms"
                print "最大间隔: " max " ms"
                print "对应频率: " 1000/(sum/count) " Hz"
            }
        }'
    fi

else
    echo "❌ 未检测到CAN帧"
    echo "可能原因:"
    echo "  - CAN接口未配置"
    echo "  - 没有程序在发送CAN帧"
    echo "  - 物理连接问题"
fi

echo ""
echo "🧹 清理临时文件"
rm -f /tmp/can_frames.txt /tmp/timestamps.txt