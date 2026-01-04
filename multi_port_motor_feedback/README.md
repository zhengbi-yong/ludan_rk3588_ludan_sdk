# Multi-Port Motor Feedback

四端口同时监听 CAN 电机反馈数据程序。

## 功能

- 同时监听 4 个 CAN 端口 (8000, 8001, 8002, 8003)
- 支持 30 个电机 (8+8+8+6 分布)
- 每个端口使用独立的 CAN 通道 (0, 1, 2, 3)
- 每个端口的 CAN ID 是独立的 (1-8 或 1-6)
- 实时显示接收到的 CAN 帧数据
- 统计每个端口的接收帧数

## 端口配置

| TCP 端口 | CAN 通道 | 电机数 | 电机号范围 | CAN ID 范围 |
|----------|----------|--------|------------|-------------|
| 8000     | 0        | 8      | 1-8        | 1-8         |
| 8001     | 1        | 8      | 9-16       | 1-8         |
| 8002     | 2        | 8      | 17-24      | 1-8         |
| 8003     | 3        | 6      | 25-30      | 1-6         |

## 编译

```bash
cd /home/linaro/ludan_sdk/motorfeedback
mkdir build && cd build
cmake ..
make
```

## 运行

```bash
# 使用默认配置 (ZLG IP: 192.168.1.5)
./multi_port_motor_feedback

# 指定 ZLG 设备 IP
./multi_port_motor_feedback --zlg-ip 192.168.1.100
```

## 输出示例

```
=== Initializing Multi-Port Motor Feedback Manager ===
ZLG IP: 192.168.1.5
Ports: 8000(8 motors, ch0) 8001(8 motors, ch1) 8002(8 motors, ch2) 8003(6 motors, ch3)
Total motors: 30
[Device] Opening ZLG device...
[Device] Device opened, handle: 140180289965568
  [Port 8000] Initializing...
  [Port 8000] CAN channel 0 initialized, handle: 140180289965568
  [Port 8000] Ready (motors 1-8, local CAN ID: 1-8)
  [Port 8001] Initializing...
  [Port 8001] CAN channel 1 initialized, handle: 140180289965568
  [Port 8001] Ready (motors 9-16, local CAN ID: 1-8)
  [Port 8002] Initializing...
  [Port 8002] CAN channel 2 initialized, handle: 140180289965568
  [Port 8002] Ready (motors 17-24, local CAN ID: 1-8)
  [Port 8003] Initializing...
  [Port 8003] CAN channel 3 initialized, handle: 140180289965568
  [Port 8003] Ready (motors 25-30, local CAN ID: 1-6)
>>> All CAN Receive Threads Started <<<

[Port 8000 RAW] CAN ID=0x001 Len=8 Data: 01 00 80 00 00 00 2a 1c
[Port 8000 RAW] CAN ID=0x002 Len=8 Data: 02 00 80 00 00 00 29 1c
...

=== Status ===
  Port 8000: 1234 frames
  Port 8001: 1234 frames
  Port 8002: 1234 frames
  Port 8003: 923 frames
Total: 4625 frames
=============
```

## 说明

1. **每个端口独立 CAN ID**: 每个端口上的 CAN ID 是独立的，范围都是 1-8 (或 1-6)
2. **共享设备句柄**: 所有通道共用一个 `ZCAN_OpenDevice` 返回的设备句柄
3. **电机映射**:
   - 端口 8000, CAN ID 1 → 电机 1 (数组索引 0)
   - 端口 8001, CAN ID 1 → 电机 9 (数组索引 8)
   - 端口 8003, CAN ID 6 → 电机 30 (数组索引 29)
4. **原始数据**: 只存储前 6 字节（不含温度信息）
