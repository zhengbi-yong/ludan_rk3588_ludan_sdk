#!/bin/bash
# 监控发送到周立功设备(192.168.1.5:8002)的TCP流量

echo "========================================="
echo "监控发送到 192.168.1.5:8002 的流量"
echo "========================================="
echo ""

# 获取网络接口
INTERFACE=$(ip route get 192.168.1.5 | awk '{print $3; exit}')
echo "使用网络接口: $INTERFACE"
echo ""

# 检查tcpdump权限
if ! sudo -n true 2>/dev/null; then
    echo "需要sudo权限来运行tcpdump"
fi

echo "开始监控..."
echo "按 Ctrl+C 停止"
echo ""

# 使用tcpdump监控发送到192.168.1.5:8002的TCP包
# -X: 以hex和ASCII格式显示
# -n: 不解析主机名
# -S: 显示绝对序列号
# 'tcp dst port 8002 and dst host 192.168.1.5': 过滤条件

sudo tcpdump -i $INTERFACE -n -X -tt 'tcp dst port 8002 and dst host 192.168.1.5'
