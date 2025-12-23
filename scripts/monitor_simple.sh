#!/bin/bash
# 简单监控发送到192.168.1.5:8002的TCP数据包
# 使用tcpdump并以hex格式显示

echo "========================================="
echo "监控发送到 192.168.1.5:8002 的TCP数据"
echo "========================================="
echo ""

# 检查是否为root或有sudo权限
if [ "$EUID" -ne 0 ]; then
    echo "需要root权限，使用sudo运行..."
    sudo "$0" "$@"
    exit $?
fi

# 获取网络接口
INTERFACE=$(ip route get 192.168.1.5 2>/dev/null | awk '{print $3; exit}')
if [ -z "$INTERFACE" ]; then
    INTERFACE="eth0"
fi

echo "✓ 使用网络接口: $INTERFACE"
echo "✓ 过滤: 发送到 192.168.1.5:8002"
echo ""
echo "开始监控..."
echo "按 Ctrl+C 停止"
echo ""
echo "=============================================================="

# 使用tcpdump监控，以hex和ASCII格式显示
tcpdump -i $INTERFACE -n -X -l 'tcp dst port 8002 and dst host 192.168.1.5' 2>/dev/null | while read line; do
    # 只显示包含数据包内容的行
    if echo "$line" | grep -qE "^[0-9]{2}:[0-9]{2}:[0-9]{2}|0x[0-9a-f]{4}:"; then
        echo "$line"
    fi
done
