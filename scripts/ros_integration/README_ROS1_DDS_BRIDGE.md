# ROS1 to DDS Bridge for RK3588 Communication

本文档说明如何在Jetson上使用ROS1 Noetic与RK3588进行DDS通信。

## 概述

ROS1 to DDS Bridge是一个桥接节点，可以将ROS1 Noetic的消息转换为DDS协议，实现与RK3588板卡的实时通信。该桥接支持发送正弦波轨迹控制命令到RK3588的电机控制器。

## 系统架构

```
Jetson (ROS1 Noetic)          RK3588 (DDS)
┌─────────────────┐         ┌─────────────────┐
│ ROS1 Topics     │         │ DDS Subscribers │
│ /cmd_vel        │◄────────┤                 │
│ /joint_states   │◄────────┤                 │
│ /target_pose    │         │                 │
│                 │         │                 │
│ Bridge Node     │────────►│ Motor Control   │
│ - Sine Generator│         │ - LowCmd        │
│ - Network Comm  │         │ - LowState      │
└─────────────────┘         └─────────────────┘
```

## 文件结构

```
/home/linaro/
├── ros1_to_dds_bridge.py          # 主要桥接节点
├── test_ros1_dds_communication.py # 通信测试脚本
├── package.xml                    # ROS1包配置
├── launch/
│   └── ros1_to_dds_bridge.launch # ROS1启动文件
├── scripts/
│   └── deploy_ros1_to_dds.sh     # 部署脚本
└── README_ROS1_DDS_BRIDGE.md     # 本文档
```

## 安装要求

### 系统要求
- Ubuntu 20.04 LTS
- ROS1 Noetic
- Python 3.8+
- 网络连接（以太网直连）

### ROS1包依赖
```bash
sudo apt-get update
sudo apt-get install ros-noetic-std-msgs
sudo apt-get install ros-noetic-geometry-msgs
sudo apt-get install ros-noetic-sensor-msgs
sudo apt-get install ros-noetic-rviz
sudo apt-get install ros-noetic-rqt-plot
```

### Python依赖（可选）
```bash
pip3 install cyclonedds  # 如果使用原生DDS
```

## 网络配置

### 1. 配置Jetson网络接口
```bash
# 设置有线接口IP
sudo ip addr add 192.168.1.10/24 dev eth0
sudo ip link set eth0 up

# 验证配置
ip addr show eth0
```

### 2. 配置RK3588网络接口
在RK3588上执行：
```bash
# 设置有线接口IP
sudo ip addr add 192.168.1.20/24 dev eth0
sudo ip link set eth0 up

# 验证配置
ip addr show eth0
```

### 3. 测试连通性
```bash
# 从Jetson测试
ping 192.168.1.20

# 从RK3588测试
ping 192.168.1.10
```

## 使用方法

### 方法1：使用部署脚本（推荐）

1. **基本使用**
```bash
cd /home/linaro
chmod +x scripts/deploy_ros1_to_dds.sh
./scripts/deploy_ros1_to_dds.sh
```

2. **自定义参数**
```bash
./scripts/deploy_ros1_to_dds.sh \
    --rk3588_ip 192.168.1.20 \
    --amplitude 0.5 \
    --frequency 1.0 \
    --joints [0,1,2,3]
```

3. **跳过通信测试**
```bash
./scripts/deploy_ros1_to_dds.sh --no-test
```

### 方法2：手动启动

1. **启动ROS Core**
```bash
source /opt/ros/noetic/setup.bash
roscore &
```

2. **启动桥接节点**
```bash
source /opt/ros/noetic/setup.bash
python3 ros1_to_dds_bridge.py \
    _rk3588_ip:=192.168.1.20 \
    _sine_amplitude:=0.3 \
    _sine_frequency:=0.5
```

### 方法3：使用launch文件
```bash
source /opt/ros/noetic/setup.bash
roslaunch launch/ros1_to_dds_bridge.launch \
    rk3588_ip:=192.168.1.20 \
    sine_amplitude:=0.3
```

## 参数配置

### 网络参数
- `rk3588_ip`: RK3588的IP地址（默认：192.168.1.20）
- `rk3588_port`: RK3588的端口号（默认：8888）
- `use_dds`: 是否使用原生DDS（默认：false）
- `publish_rate`: 发布频率，单位Hz（默认：50）

### 运动控制参数
- `sine_amplitude`: 正弦波幅度，单位弧度（默认：0.3）
- `sine_frequency`: 正弦波频率，单位Hz（默认：0.5）
- `target_joints`: 目标关节ID列表（默认：[0,1,2]）

## ROS1话题

### 发布的话题
- `/joint_states`: 关节状态反馈（sensor_msgs/JointState）
- `/target_pose`: 目标位姿（geometry_msgs/PoseStamped）

### 订阅的话题
- `/cmd_vel`: 速度命令（geometry_msgs/Twist）
- `/target_joint_states`: 目标关节状态（sensor_msgs/JointState）

## 测试和验证

### 1. 通信测试
```bash
python3 test_ros1_dds_communication.py \
    _rk3588_ip:=192.168.1.20 \
    _test_duration:=30
```

### 2. ROS1话题监控
```bash
# 新终端
source /opt/ros/noetic/setup.bash
rostopic list

# 监控特定话题
rostopic echo /joint_states
rostopic hz /target_pose
```

### 3. 可视化工具
```bash
# 启动RViz
rosrun rviz rviz

# 启动数据绘图
rosrun rqt_plot rqt_plot /joint_states/position[0] /joint_states/position[1]
```

## 故障排除

### 常见问题

1. **网络连接失败**
   - 检查网线连接
   - 验证IP地址配置
   - 检查防火墙设置

2. **ROS1环境问题**
   ```bash
   source /opt/ros/noetic/setup.bash
   echo $ROS_DISTRO  # 应该显示 "noetic"
   ```

3. **桥接节点启动失败**
   - 检查Python3是否安装
   - 验证脚本权限
   - 查看错误日志

4. **数据包丢失**
   - 检查网络质量
   - 降低发布频率
   - 检查RK3588负载

### 日志信息
桥接节点会输出详细的日志信息，包括：
- 网络连接状态
- 数据包发送统计
- 错误和警告信息

### 调试模式
启用调试日志：
```bash
python3 ros1_to_dds_bridge.py --log-level DEBUG
```

## 性能优化

### 网络优化
- 使用千兆以太网
- 减少网络跳数
- 禁用不必要的网络服务

### 参数调优
- 调整`publish_rate`以平衡实时性和负载
- 优化`sine_amplitude`和`sine_frequency`
- 选择合适的`target_joints`

### 系统优化
```bash
# 设置实时优先级
sudo chrt -f 50 python3 ros1_to_dds_bridge.py

# 禁用CPU频率缩放
sudo cpupower frequency-set --governor performance
```

## 扩展开发

### 添加新的消息类型
1. 在桥接节点中导入相应的ROS1消息类型
2. 实现消息转换逻辑
3. 更新网络协议

### 支持原生DDS
1. 安装CycloneDDS Python绑定
2. 修改`setup_dds_connection()`函数
3. 实现DDS数据写入器

### 多机器人支持
1. 扩展网络协议以支持机器人ID
2. 添加多目标IP配置
3. 实现负载均衡

## 许可证

MIT License

## 联系方式

如有问题或建议，请联系：Claude Code Assistant

---

**注意**：在生产环境中使用前，请充分测试并验证所有功能。