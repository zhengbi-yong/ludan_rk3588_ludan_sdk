# G1 Ankle Swing Example - Debug Tools

这个目录包含了在没有真实机器人连接时调试G1踝关节摆动示例的工具。

## 文件说明

### 核心程序
- **`g1_ankle_swing_example_debug.cpp`** - 增强版踝关节摆动示例，包含详细日志
- **`command_monitor.cpp`** - 监听并显示所有发送的命令
- **`state_simulator.cpp`** - 模拟机器人状态反馈

### 辅助工具
- **`test_ankle_swing.sh`** - 一键启动所有调试组件的脚本

## 使用方法

### 方法1：使用测试脚本（推荐）
```bash
# 在build目录中运行
cd /home/linaro/unitree_sdk2/build/bin
../example/g1/low_level/test_ankle_swing.sh lo

# 或者指定网络接口
../example/g1/low_level/test_ankle_swing.sh eth0
```

### 方法2：手动启动
需要3个终端窗口：

**终端1：启动状态模拟器**
```bash
./state_simulator lo
```

**终端2：启动命令监控器**
```bash
./command_monitor lo
```

**终端3：启动调试版示例**
```bash
./g1_ankle_swing_example_debug lo
```

## 工作原理

### 1. 调试版示例 (g1_ankle_swing_example_debug)
- 显示控制阶段信息
- 每10个命令打印一次详细信息
- 重点显示踝关节的目标位置和PD参数
- 显示接收到的机器人状态

### 2. 命令监控器 (command_monitor)
- 监听`rt/lowcmd` DDS话题
- 实时显示所有发送的电机命令
- 突出显示踝关节的命令
- 显示模式（PR或AB）和CRC校验

### 3. 状态模拟器 (state_simulator)
- 发布模拟的机器人状态到`rt/lowstate`话题
- 发布模拟的IMU数据到`rt/secondary_imu`话题
- 避免因缺少反馈导致的程序错误

## 控制阶段说明

程序按以下阶段运行：

1. **阶段1 (0-3秒)**：将机器人移动到零位姿
2. **阶段2 (3-6秒)**：使用PR模式摆动踝关节
   - PR模式：Pitch/Roll串联控制
   - 左右脚踝同相反向摆动
3. **阶段3 (6秒后)**：使用AB模式摆动踝关节
   - AB模式：A/B并联控制
   - 左右脚踝反相同向摆动

## 预期输出

### 调试版示例输出示例：
```
[DEBUG] Stage 1: Moving to zero posture (t=1.50s)
[DEBUG] Command #50 sent:
[DEBUG]   Mode PR: 0
[DEBUG]   Ankle Commands:
[DEBUG]     Left Ankle Pitch:  pos=-0.123, kp=40.0
[DEBUG]     Left Ankle Roll:   pos=0.045, kp=40.0
```

### 命令监控器输出示例：
```
=== COMMAND MONITOR ===
Time: 5.23s
Mode PR: 1 (0=PR, 1=AB)
Left Ankle Pitch (ID 4):  pos=0.524, kp=40.0
Left Ankle Roll (ID 5):   pos=-0.175, kp=40.0
```

## 故障排除

1. **如果程序无法启动**
   - 检查网络接口是否正确
   - 确保已编译最新版本：`make -j4`

2. **如果没有命令输出**
   - 确保state_simulator先运行
   - 检查DDS初始化是否成功

3. **如果CRC错误**
   - 正常现象，因为模拟的state没有完全实现所有字段

## 网络接口说明

- **`lo`**：本地回环接口，适合在同一台机器上测试
- **`eth0`**：以太网接口，用于通过以太网连接到真实机器人

## 停止所有程序

使用Ctrl+C或运行：
```bash
pkill -f "g1_ankle_swing_example"
pkill -f "command_monitor"
pkill -f "state_simulator"
```