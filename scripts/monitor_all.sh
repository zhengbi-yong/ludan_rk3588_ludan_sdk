#!/bin/bash
# 监控所有TCP流量，帮助调试

echo "========================================="
echo "监控所有TCP流量（调试模式）"
echo "========================================="
echo ""

# 检查是否为root
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
echo "✓ 监控所有TCP流量到 192.168.1.5"
echo ""
echo "开始监控..."
echo "按 Ctrl+C 停止"
echo ""
echo "=============================================================="

# 监控所有到192.168.1.5的TCP流量
tcpdump -i $INTERFACE -n -X -l 'tcp and dst host 192.168.1.5' 2>&1
