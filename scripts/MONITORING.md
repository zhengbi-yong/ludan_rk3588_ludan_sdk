# 周立功设备流量监控工具

## 概述

这些工具用于监控发送到周立功设备(192.168.1.5:8002)的TCP流量，可以查看到底发送了什么CAN frame数据。

## 监控工具

### 1. 简单监控 (推荐)

```bash
sudo ./scripts/monitor_simple.sh
```

以简化的hex格式显示发送到192.168.1.5:8002的TCP数据包。

### 2. 详细Hex监控

```bash
sudo ./scripts/monitor_zhilgong_traffic.sh
```

使用tcpdump以完整的hex和ASCII格式显示所有数据包。

### 3. Python解析监控 (需要scapy)

```bash
# 安装scapy
sudo apt-get install python3-scapy

# 运行监控
python3 ./scripts/monitor_zhilgong.py
```

自动解析CAN frame格式，显示：
- CAN ID
- DLC
- 数据内容
- 命令类型（使能/禁使能/数据帧）

## 使用示例

### 终端1: 启动监控

```bash
cd /home/linaro/ludan_sdk
sudo ./scripts/monitor_simple.sh
```

### 终端2: 运行motor_controller并使能电机

```bash
# 启动motor_controller
./bin/motor_controller_with_enable

# 在另一个终端使能电机
python3 scripts/enable_motor.py 5 1
```

### 预期输出

如果监控正常工作，你应该看到类似这样的输出：

```
============================================================
📦 发送到 192.168.1.5:8002
============================================================

Frame #1:
  CAN ID    : 0x05 (电机 5)
  DLC       : 8
  数据      : FF FF FF FF FF FF FF FC
  类型      : 使能
  完整Hex   : 05 08 FF FF FF FF FF FF FF FC
  字节详情  :
    [0] CAN_ID  : 0x05
    [1] DLC     : 0x08
    [2] DATA[0] : 0xFF
    [3] DATA[1] : 0xFF
    [4] DATA[2] : 0xFF
    [5] DATA[3] : 0xFF
    [6] DATA[4] : 0xFF
    [7] DATA[5] : 0xFF
    [8] DATA[6] : 0xFF
    [9] DATA[7] : 0xFC  <-- 使能命令
```

## CAN Frame 格式

发送到周立功设备的每个CAN frame包格式：

```
[CAN_ID(1 byte) | DLC(1 byte) | DATA(8 bytes)]
总共10字节
```

### 使能命令
```
XX 08 FF FF FF FF FF FF FF FC
```
- XX: CAN ID (例如 0x05 表示电机5)
- 08: DLC = 8
- FF FF FF FF FF FF FF FC: 使能数据

### 禁使能命令
```
XX 08 FF FF FF FF FF FF FF FD
```
- XX: CAN ID
- 08: DLC = 8
- FF FF FF FF FF FF FF FD: 禁使能数据

## 故障排查

### 没有捕获到数据包

1. **检查网络接口**：
   ```bash
   ip route get 192.168.1.5
   ```

2. **检查程序是否运行**：
   ```bash
   ps aux | grep motor_controller
   ```

3. **检查连接状态**：
   ```bash
   netstat -an | grep 8002
   ```

### 权限错误

监控需要root权限或sudo，确保使用sudo运行：
```bash
sudo ./scripts/monitor_simple.sh
```

### 找不到网络接口

手动指定网络接口：
```bash
sudo tcpdump -i eth0 -n -X 'tcp dst port 8002 and dst host 192.168.1.5'
```
