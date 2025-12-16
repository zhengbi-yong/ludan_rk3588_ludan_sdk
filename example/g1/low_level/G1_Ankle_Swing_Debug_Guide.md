# G1踝关节摆动示例调试指南

本指南详细说明如何在没有真实机器人的情况下运行和调试G1踝关节摆动示例程序。通过使用虚拟反馈和命令监控工具，您可以在开发环境中完全测试程序的运行情况。

## 目录

1. [概述](#概述)
2. [文件说明](#文件说明)
3. [环境准备](#环境准备)
4. [编译程序](#编译程序)
5. [运行方式](#运行方式)
6. [预期输出](#预期输出)
7. [故障排除](#故障排除)
8. [深入理解](#深入理解)

## 概述

### 问题背景
在开发机器人控制程序时，通常需要真实的机器人硬件来测试和验证控制命令。然而，在实际开发过程中，开发者可能没有随时访问机器人硬件的条件。本调试方案解决了这个问题。

### 解决方案
我们创建了一套虚拟调试工具，包括：
1. **状态模拟器** - 模拟机器人传感器反馈
2. **命令监控器** - 实时监控发送的控制命令
3. **增强版示例** - 带有详细日志的控制程序

这些工具通过DDS（Data Distribution Service）中间件通信，完全模拟了真实的机器人控制环境。

## 文件说明

### 核心程序文件

| 文件名 | 描述 | 作用 |
|--------|------|------|
| `g1_ankle_swing_example_debug.cpp` | 调试版踝关节摆动示例 | 发送控制命令，带详细日志输出 |
| `command_monitor.cpp` | 命令监控器 | 监听并显示所有发送的电机命令 |
| `state_simulator.cpp` | 状态模拟器 | 生成模拟的机器人状态反馈 |
| `demo_commands.cpp` | 简单演示程序 | 发送预设的测试命令序列 |

### 辅助文件

| 文件名 | 描述 |
|--------|------|
| `test_ankle_swing.sh` | 一键启动脚本，自动运行所有组件 |
| `CMakeLists.txt` | 构建配置文件（已修改） |
| `DEBUG_README.md` | 简体中文版说明文档 |

## 环境准备

### 1. 检查系统要求

确保您的系统满足以下要求：
- Ubuntu 20.04 LTS 或兼容系统
- 已安装Unitree SDK2
- CMake 3.5或更高版本
- GCC 9.4.0或更高版本

### 2. 检查依赖项

运行以下命令确保所有依赖已安装：
```bash
# 更新包列表
sudo apt-get update

# 安装必要的依赖
sudo apt-get install -y cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev
```

### 3. 验证网络接口

程序需要指定一个网络接口来进行DDS通信：
- `lo` - 本地回环接口（推荐用于单机测试）
- `eth0` - 以太网接口（用于连接真实机器人）

您可以使用以下命令查看可用的网络接口：
```bash
ip addr show
```

## 编译程序

### 1. 进入项目目录
```bash
cd /home/linaro/unitree_sdk2
```

### 2. 创建构建目录（如果不存在）
```bash
mkdir -p build
cd build
```

### 3. 配置CMake
```bash
cmake ..
```

如果出现以下输出，说明配置成功：
```
-- Current system architecture: aarch64  # 或 x86_64
-- Unitree SDK library found at: /home/linaro/unitree_sdk2/lib/aarch64/libunitree_sdk2.a
-- Configuring done
-- Generating done
```

### 4. 编译所有示例和调试工具
```bash
make -j4
```

您应该看到类似以下的输出：
```
[ 64%] Built target g1_ankle_swing_example
[ 65%] Built target g1_ankle_swing_example_debug
[ 69%] Built target command_monitor
[ 75%] Built target state_simulator
[100%] Built target demo_commands
```

### 5. 验证编译结果
检查生成的可执行文件：
```bash
ls -la bin/ | grep -E "(ankle|monitor|simulator|demo)"
```

应该看到：
```
-rwxr-xr-x 1 linaro linaro 123456 Dec 16 22:00 command_monitor
-rwxr-xr-x 1 linaro linaro 123456 Dec 16 22:00 demo_commands
-rwxr-xr-x 1 linaro linaro 123456 Dec 16 22:00 g1_ankle_swing_example
-rwxr-xr-x 1 linaro linaro 123456 Dec 16 22:00 g1_ankle_swing_example_debug
-rwxr-xr-x 1 linaro linaro 123456 Dec 16 22:00 state_simulator
```

## 运行方式

### 方式1：使用一键测试脚本（推荐）

#### 步骤1：进入bin目录
```bash
cd /home/linaro/unitree_sdk2/build/bin
```

#### 步骤2：运行测试脚本
```bash
../example/g1/low_level/test_ankle_swing.sh lo
```

#### 步骤3：观察输出

脚本会自动启动所有组件，您应该看到：
```
========================================
G1 Ankle Swing Test - Debug Mode
========================================
Network Interface: lo

Cleaning up any existing processes...
1. Starting state simulator...
State Simulator Initialized
Publishing simulated robot state...
   PID: 12345

2. Starting command monitor...
Command Monitor Started...
   PID: 12346

3. Starting G1 ankle swing example (debug version)...
========================================
G1 Ankle Swing Example - DEBUG VERSION
========================================
Network Interface: lo
This version includes enhanced logging
========================================
   PID: 12347

All processes started successfully!

Command Monitor will show all sent commands
Ankle Swing Example will show its internal state
State Simulator provides fake robot feedback

Press Ctrl+C to stop all processes
========================================
```

### 方式2：手动运行（适合调试）

打开3个终端窗口：

#### 终端1 - 启动状态模拟器
```bash
cd /home/linaro/unitree_sdk2/build/bin
./state_simulator lo
```

预期输出：
```
State Simulator Initialized
Publishing simulated robot state...
1234567890.123456 [0] state_simu: selected interface "lo" is not multicast-capable: disabling multicast
```

#### 终端2 - 启动命令监控器
```bash
cd /home/linaro/unitree_sdk2/build/bin
./command_monitor lo
```

预期输出（初始）：
```
Command Monitor Started...
Listening for commands on topic: rt/lowcmd
Press Ctrl+C to stop
```

#### 终端3 - 启动调试版示例
```bash
cd /home/linaro/unitree_sdk2/build/bin
./g1_ankle_swing_example_debug lo
```

预期输出（初始）：
```
========================================
G1 Ankle Swing Example - DEBUG VERSION
========================================
Network Interface: lo
This version includes enhanced logging
For monitoring commands separately, run:
  ./command_monitor lo
For simulating robot state, run:
  ./state_simulator lo
========================================
[DEBUG] Initializing G1 Ankle Swing Example...
[DEBUG] Command publisher initialized on topic: rt/lowcmd
[DEBUG] State subscriber initialized on topic: rt/lowstate
[DEBUG] IMU subscriber initialized on topic: rt/secondary_imu
[DEBUG] G1Example initialization complete!
[DEBUG] Control stages:
[DEBUG]   Stage 1 (0-3s): Move to zero posture
[DEBUG]   Stage 2 (3-6s): Ankle swing in PR mode
[DEBUG]   Stage 3 (6-9s+): Ankle swing in AB mode
```

### 方式3：简单演示测试

如果您只想快速测试命令监控功能：

#### 终端1：
```bash
cd /home/linaro/unitree_sdk2/build/bin
./command_monitor lo
```

#### 终端2：
```bash
cd /home/linaro/unitree_sdk2/build/bin
./demo_commands lo
```

## 预期输出

### 1. 调试版示例的日志输出

程序运行时会按照3个阶段输出日志：

**阶段1（0-3秒）- 归零阶段**：
```
[DEBUG] Stage 1: Moving to zero posture (t=0.50s)
[DEBUG] Stage 1: Moving to zero posture (t=1.00s)
[DEBUG] Stage 1: Moving to zero posture (t=1.50s)
[DEBUG] Stage 1: Moving to zero posture (t=2.00s)
[DEBUG] Stage 1: Moving to zero posture (t=2.50s)
[DEBUG] Stage 1: Moving to zero posture (t=3.00s)
```

**阶段2（3-6秒）- PR模式摆动**：
```
[DEBUG] Stage 2: Ankle swing in PR mode (t=3.50s)
[DEBUG] Command #250 sent:
[DEBUG]   Mode PR: 0
[DEBUG]   Ankle Commands:
[DEBUG]     Left Ankle Pitch:  pos=0.309, kp=40.0
[DEBUG]     Left Ankle Roll:   pos=0.104, kp=40.0
[DEBUG]     Right Ankle Pitch: pos=0.309, kp=40.0
[DEBUG]     Right Ankle Roll:  pos=-0.104, kp=40.0
```

**阶段3（6秒后）- AB模式摆动**：
```
[DEBUG] Stage 3: Ankle swing in AB mode (t=6.50s)
[DEBUG] Command #750 sent:
[DEBUG]   Mode PR: 1
[DEBUG]   Ankle Commands:
[DEBUG]     Left Ankle Pitch:  pos=0.524, kp=40.0
[DEBUG]     Left Ankle Roll:   pos=-0.175, kp=40.0
[DEBUG]     Right Ankle Pitch: pos=-0.524, kp=40.0
[DEBUG]     Right Ankle Roll:  pos=0.175, kp=40.0
```

**IMU和状态信息**（每1秒输出）：
```
[DEBUG] IMU.torso.rpy: 0.00 0.00 0.00
[DEBUG] gamepad_.A.pressed: 0
[DEBUG] gamepad_.B.pressed: 0
[DEBUG] gamepad_.X.pressed: 0
[DEBUG] gamepad_.Y.pressed: 0
[DEBUG] Ankle Positions:
  Left Ankle Pitch (ID4):  pos=0.309, vel=0.000
  Left Ankle Roll (ID5):   pos=0.104, vel=0.000
  Right Ankle Pitch (ID10): pos=0.309, vel=0.000
  Right Ankle Roll (ID11):  pos=-0.104, vel=0.000
```

### 2. 命令监控器的输出

命令监控器会显示详细的命令信息：

```
=== COMMAND MONITOR ===
Time: 3.56s
Message Count: 1780
Mode PR: 0 (0=PR, 1=AB)
Mode Machine: 1
CRC: 0x1234abcd

=== MOTOR COMMANDS ===
           Joint Name    Mode Target Pos Target Vel       Kp       Kd   Tau FF
--------------------------------------------------------------------------------------------
    Left Ankle Pitch        1     0.309     0.000     40.0      1.0   0.000
     Left Ankle Roll        1     0.104     0.000     40.0      1.0   0.000
   Right Ankle Pitch        1     0.309     0.000     40.0      1.0   0.000
    Right Ankle Roll        1    -0.104     0.000     40.0      1.0   0.000

=== ANKLE JOINTS DETAIL ===
Left Ankle Pitch (ID 4):  pos=0.309, vel=0.000, kp=40.0
Left Ankle Roll (ID 5):   pos=0.104, vel=0.000, kp=40.0
Right Ankle Pitch (ID 10): pos=0.309, vel=0.000, kp=40.0
Right Ankle Roll (ID 11):  pos=-0.104, vel=0.000, kp=40.0
```

### 3. 演示程序的输出

运行demo_commands会看到：
```
Demo Command Publisher Started
Sending test ankle commands...
Step 0: L_Pitch=0.000, L_Roll=0.200, R_Pitch=0.000, R_Roll=-0.200
Step 1: L_Pitch=0.050, L_Roll=0.190, R_Pitch=0.050, R_Roll=-0.190
Step 2: L_Pitch=0.100, L_Roll=0.180, R_Pitch=0.100, R_Roll=-0.180
...
Step 99: L_Pitch=0.000, L_Roll=0.200, R_Pitch=0.000, R_Roll=-0.200
Demo completed!
```

## 故障排除

### 问题1：程序无法启动或段错误

**症状**：运行程序时出现段错误或立即退出

**解决方案**：
1. 确保使用正确的网络接口：
   ```bash
   # 对于本地测试使用lo
   ./program lo

   # 如果lo有问题，尝试其他接口
   ip addr show  # 查看可用接口
   ./program eth0  # 或其他接口
   ```

2. 检查DDS初始化日志，查看是否有错误

3. 确保所有程序都已正确编译

### 问题2：命令监控器没有输出

**症状**：监控器启动正常但没有显示任何命令

**解决方案**：
1. 确保状态模拟器先运行：
   ```bash
   # 先运行状态模拟器
   ./state_simulator lo
   # 等待2秒后再运行其他程序
   sleep 2
   ```

2. 检查网络接口参数是否一致

3. 查看是否有错误消息：
   ```
   selected interface "lo" is not multicast-capable: disabling multicast
   ```
   这个警告可以忽略，不影响功能

### 问题3：CRC校验错误

**症状**：出现`[ERROR] CRC Error`消息

**解决方案**：
这是正常现象，因为模拟器生成的状态数据可能不完整。不影响命令发送和监控。

### 问题4：编译错误

**常见编译错误及解决方案**：

1. **找不到头文件**：
   ```bash
   # 确保在正确的目录
   cd /home/linaro/unitree_sdk2
   ```

2. **链接错误**：
   ```bash
   # 清理并重新编译
   cd build
   make clean
   cmake ..
   make -j4
   ```

3. **权限错误**：
   ```bash
   # 确保可执行文件权限
   chmod +x bin/*
   chmod +x example/g1/low_level/*.sh
   ```

### 问题5：端口已被占用

**症状**：DDS初始化失败，提示端口占用

**解决方案**：
```bash
# 停止所有相关进程
pkill -f "ankle_swing"
pkill -f "command_monitor"
pkill -f "state_simulator"
pkill -f "demo_commands"

# 等待几秒后重新启动
sleep 3
```

## 深入理解

### 1. DDS通信机制

程序使用DDS进行数据通信：

- **命令通道** (`rt/lowcmd`): 发送电机控制命令
- **状态通道** (`rt/lowstate`): 接收机器人状态
- **IMU通道** (`rt/secondary_imu`): 接收IMU数据

### 2. 控制模式说明

**PR模式（Series Control）**：
- 串联控制模式
- 直接控制Pitch和Roll关节
- 左右脚踝同相反向运动

**AB模式（Parallel Control）**：
- 并联控制模式
- 控制A和B两个虚拟关节
- 左右脚踝反相同向运动

### 3. 电机ID映射

踝关节的电机ID：
- ID 4: 左踝Pitch
- ID 5: 左踝Roll
- ID 10: 右踝Pitch
- ID 11: 右踝Roll

### 4. PD控制参数

程序使用的默认PD参数：
```cpp
// 踝关节PD参数
Kp[4] = 40.0;   // 左踝Pitch刚度
Kd[4] = 1.0;    // 左踝Pitch阻尼
Kp[5] = 40.0;   // 左踝Roll刚度
Kd[5] = 1.0;    // 左踝Roll阻尼
Kp[10] = 40.0;  // 右踝Pitch刚度
Kd[10] = 1.0;   // 右踝Pitch阻尼
Kp[11] = 40.0;  // 右踝Roll刚度
Kd[11] = 1.0;   // 右踝Roll阻尼
```

### 5. 运动轨迹解析

**PR模式轨迹**：
```cpp
double max_P = M_PI * 30.0 / 180.0;  // 30度峰值
double max_R = M_PI * 10.0 / 180.0;  // 10度峰值
L_P_des = max_P * sin(2π * t);      // 左踝Pitch
R_P_des = max_P * sin(2π * t);      // 右踝Pitch（同相）
L_R_des = max_R * sin(2π * t);      // 左踝Roll
R_R_des = -max_R * sin(2π * t);     // 右踝Roll（反相）
```

**AB模式轨迹**：
```cpp
double max_A = M_PI * 30.0 / 180.0;  // 30度峰值
double max_B = M_PI * 10.0 / 180.0;  // 10度峰值
L_A_des = +max_A * sin(π * t);       // 左踝A
L_B_des = +max_B * sin(π * t + π);   // 左踝B
R_A_des = -max_A * sin(π * t);       // 右踝A（反相）
R_B_des = -max_B * sin(π * t + π);   // 右踝B（反相）
```

## 总结

通过本指南，您应该能够在没有真实机器人的情况下完全测试和调试G1踝关节摆动示例程序。这个调试方案提供了：

1. **完整的反馈链路** - 通过状态模拟器提供虚拟反馈
2. **透明的命令监控** - 实时查看所有控制命令
3. **详细的日志输出** - 帮助理解程序运行状态
4. **易于使用的脚本** - 一键启动所有组件

这种方法可以大大加速开发和调试过程，特别是在机器人硬件不可用时。您可以使用相同的模式为其他机器人控制程序创建类似的调试环境。