# Jetson ROS1 LowCmd Publisher Setup Guide

## 概述

本指南说明如何在Jetson上设置和运行ROS1 LowCmd发布器，用于向RK3588发送正弦波轨迹数据。

## 🔧 问题说明

**之前的问题**: 原来的tar包只包含接收器代码，Jetson上没有实际发布`/lowcmd`话题的节点。

**现在的解决方案**: 新的tar包包含完整的发布器，能够在Jetson上发布`/lowcmd`话题。

## 📦 文件清单

### 核心文件
- `jetson_lowcmd_publisher.py` - 主要的LowCmd发布器
- `start_jetson_lowcmd.sh` - 一键启动脚本
- `test_jetson_publisher.py` - 发布器测试工具

### 辅助文件
- `python_lowcmd_builder.py` - LowCmd构建工具
- `lowcmd_format_examples.md` - 格式说明文档
- `lowcmd_data_example.json` - 数据示例

## 🚀 快速开始

### 1. 在Jetson上解压并准备

```bash
# 传输文件到Jetson
scp jetson_ros1_lowcmd_package.tar.gz your_jetson_username@192.168.0.139:~/

# SSH到Jetson
ssh your_jetson_username@192.168.0.139

# 解压文件
cd ~
tar -xzf jetson_ros1_lowcmd_package.tar.gz

# 设置权限
chmod +x scripts/jetson_lowcmd_publisher.py
chmod +x scripts/start_jetson_lowcmd.sh
chmod +x scripts/test_jetson_publisher.py
```

### 2. 配置网络

```bash
# 配置Jetson有线接口
sudo ip addr add 192.168.1.10/24 dev eth0
sudo ip link set eth0 up

# 验证配置
ip addr show eth0
```

### 3. 启动发布器

**方法1: 使用启动脚本（推荐）**
```bash
cd ~/scripts
./start_jetson_lowcmd.sh
```

**方法2: 手动启动**
```bash
# 启动ROS Core
source /opt/ros/noetic/setup.bash
roscore &

# 启动发布器
source /opt/ros/noetic/setup.bash
python3 jetson_lowcmd_publisher.py \
    --rk3588_ip:=192.168.1.20 \
    --sine_amplitude:=0.3 \
    --sine_frequency:=0.5
```

### 4. 验证发布器运行

**在新终端中**:
```bash
source /opt/ros/noetic/setup.bash

# 查看话题列表
rostopic list

# 监听/lowcmd话题
rostopic echo /lowcmd

# 检查话题频率
rostopic hz /lowcmd

# 查看关节状态
rostopic echo /joint_states
```

**使用测试工具**:
```bash
cd ~/scripts
python3 test_jetson_publisher.py
```

## 📋 配置参数

### 启动脚本参数

```bash
./start_jetson_lowcmd.sh [选项]

选项:
  --rk3588_ip IP       RK3588 IP地址 (默认: 192.168.1.20)
  --rk3588_port PORT   RK3588端口 (默认: 8888)
  --rate Hz           发布频率 (默认: 50)
  --amplitude VALUE   正弦波幅度 (默认: 0.3)
  --frequency VALUE   正弦波频率 (默认: 0.5)
  --joints [1,2,3]    目标关节ID (默认: [4,5,10,11])
  --no-network        禁用网络传输
  --no-bridge         禁用DDS桥接文件
  --help              显示帮助
```

### 示例配置

```bash
# 高频率小幅度运动
./start_jetson_lowcmd.sh --frequency 1.0 --amplitude 0.2 --rate 100

# 低频率大幅度运动
./start_jetson_lowcmd.sh --frequency 0.3 --amplitude 0.5 --rate 30

# 仅本地测试（不发送到网络）
./start_jetson_lowcmd.sh --no-network --no-bridge
```

## 🎯 目标关节映射

发布器默认控制以下脚踝关节：

```
关节ID  关节名称           映射到Pose
------  --------           ----------
4       left_ankle_pitch   position.x
5       left_ankle_roll    position.y
10      right_ankle_pitch  orientation.x
11      right_ankle_roll   orientation.y
```

## 📊 话题说明

### /lowcmd (geometry_msgs/PoseStamped)

主要的话题，包含正弦波轨迹数据：

```
header:
  stamp: 时间戳
  frame_id: "robot_base"
pose:
  position:
    x: 左脚踝俯仰角度
    y: 左脚踝滚转角度
    z: 0.0
  orientation:
    x: 右脚踝俯仰角度
    y: 右脚踝滚转角度
    z: 0.0
    w: 1.0
```

### /joint_states (sensor_msgs/JointState)

关节状态信息，用于调试：

```
header:
  stamp: 时间戳
name: ["left_ankle_pitch", "left_ankle_roll", "right_ankle_pitch", "right_ankle_roll"]
position: [位置1, 位置2, 位置3, 位置4]
velocity: [速度1, 速度2, 速度3, 速度4]
effort: [力矩1, 力矩2, 力矩3, 力矩4]
```

### /debug_info (std_msgs/Header)

调试信息，包含消息计数：

```
stamp: 时间戳
frame_id: "msg_count_123_time_1234.567"
```

## 🌐 网络传输

发布器支持多种数据传输方式：

### 1. UDP网络传输
- 目标: `RK3588_IP:8888`
- 格式: JSON数据包
- 内容: 时间戳、序列号、关节位置

### 2. DDS桥接文件
- 文件: `/tmp/lowcmd_data.json`
- 格式: JSON格式
- 用途: 供RK3588上的接收器读取

### 3. ROS1话题
- 话题: `/lowcmd`, `/joint_states`, `/debug_info`
- 用途: 本地调试和监控

## 🔍 故障排除

### 常见问题

1. **没有/lowcmd话题**
   - 确认发布器已启动
   - 检查ROS1环境是否正确sourced
   - 运行`rostopic list`验证话题存在

2. **网络连接失败**
   - 检查IP地址配置
   - 确认网线连接
   - 使用`ping 192.168.1.20`测试连通性

3. **发布频率过低**
   - 检查Jetson CPU负载
   - 调整发布频率参数
   - 使用`rostopic hz /lowcmd`监控频率

4. **数据不正常**
   - 检查正弦波参数
   - 验证关节映射
   - 使用`rostopic echo`查看数据

### 调试命令

```bash
# 检查ROS1环境
echo $ROS_DISTRO
env | grep ROS

# 检查网络配置
ip addr show eth0
ping 192.168.1.20

# 检查话题和节点
rostopic list
rosnode list
rosnode info /jetson_lowcmd_publisher

# 监控消息频率
rostopic hz /lowcmd
rostopic hz /joint_states

# 查看消息内容
rostopic echo /lowcmd -n 5
rostopic echo /joint_states -n 5
```

## 📈 性能优化

### 系统优化
```bash
# 设置实时优先级
sudo chrt -f 50 python3 jetson_lowcmd_publisher.py

# 禁用CPU频率缩放
sudo cpupower frequency-set --governor performance

# 增加网络缓冲区
sudo sysctl -w net.core.rmem_max=12582912
sudo sysctl -w net.core.wmem_max=12582912
```

### 参数调优
- **高实时性**: `--rate 100 --amplitude 0.1 --frequency 2.0`
- **平滑运动**: `--rate 50 --amplitude 0.3 --frequency 0.5`
- **测试模式**: `--rate 10 --amplitude 0.5 --frequency 0.1`

## 📞 支持

如有问题，请检查：
1. `scripts/JETSON_SETUP_GUIDE.md` - 本文档
2. `scripts/lowcmd_format_examples.md` - 格式说明
3. `README_ROS1_DDS_BRIDGE.md` - 总体说明

---

**注意**: 在生产环境中使用前，请充分测试所有功能，并确认网络稳定性和数据可靠性。