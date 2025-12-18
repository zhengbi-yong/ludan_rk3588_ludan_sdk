# LowCmd Message Format Examples

本文档提供`/lowcmd`话题的详细格式说明和Python构建示例。

## 概述

`/lowcmd`是Unitree机器人控制的核心话题，用于发送电机控制命令。不同机器人型号使用不同的消息格式：

- **H1/G1机器人**: `unitree_hg::msg::dds_::LowCmd_` (35个电机)
- **GO2/B2机器人**: `unitree_go::msg::dds_::LowCmd_` (20个电机)

## 数据结构

### H1/G1 LowCmd 格式

```json
{
  "mode_pr": 1,           // PR模式标志 (0=PR模式, 1=AB模式)
  "mode_machine": 1,      // 机器模式 (1=G1类型)
  "motor_cmd": [          // 35个电机命令数组
    {
      "mode": 1,          // 电机模式 (0=禁用, 1=启用)
      "q": 0.0,           // 目标位置 (弧度)
      "dq": 0.0,          // 目标速度 (弧度/秒)
      "tau": 0.0,         // 前馈力矩 (Nm)
      "kp": 100.0,        // 位置增益系数
      "kd": 3.0,          // 速度增益系数
      "reserve": [0, 0, 0] // 保留字段
    }
    // ... 总共35个电机
  ],
  "reserve": [0, 0, 0, 0], // 保留字段
  "crc": 12345678         // CRC32校验值
}
```

### 电机命令详情

```python
# MotorCmd数据结构
class MotorCmd:
    mode: uint8      # 电机模式 (0=禁用, 1=启用)
    q: float         # 目标位置 (弧度, -π到π)
    dq: float        # 目标速度 (弧度/秒)
    tau: float       # 前馈力矩 (Nm)
    kp: float        # 位置增益系数
    kd: float        # 速度增益系数
```

## G1机器人关节映射

### 关节ID与名称对应关系

```
0: left_hip_pitch      # 左髋关节俯仰
1: left_hip_roll       # 左髋关节滚转
2: left_hip_yaw        # 左髋关节偏航
3: left_knee           # 左膝关节
4: left_ankle_pitch    # 左踝关节俯仰 ⭐
5: left_ankle_roll     # 左踝关节滚转 ⭐
6: right_hip_pitch     # 右髋关节俯仰
7: right_hip_roll      # 右髋关节滚转
8: right_hip_yaw       # 右髋关节偏航
9: right_knee          # 右膝关节
10: right_ankle_pitch  # 右踝关节俯仰 ⭐
11: right_ankle_roll   # 右踝关节滚转 ⭐
12: torso_joint        # 躯干关节
13: left_shoulder_pitch # 左肩关节俯仰
14: left_shoulder_roll  # 左肩关节滚转
15: left_shoulder_yaw   # 左肩关节偏航
16: left_elbow         # 左肘关节
17: right_shoulder_pitch # 右肩关节俯仰
18: right_shoulder_roll  # 右肩关节滚转
19: right_shoulder_yaw   # 右肩关节偏航
20: right_elbow        # 右肘关节
21-26: 手腕关节
27-29: 头部关节
30-32: 腰部关节
33-34: 预留
```

⭐ 表示常用于脚踝运动控制的关节

## Python构建示例

### 1. 基本LowCmd构建

```python
from python_lowcmd_builder import LowCmdBuilder

# 创建G1机器人的LowCmd
lowcmd = LowCmdBuilder("hg")

# 设置模式
lowcmd.mode_pr = 1          # PR模式
lowcmd.mode_machine = 1     # G1类型

# 设置单个电机命令
lowcmd.set_motor_command(4,
    mode=1,
    q=0.5,      # 0.5弧度位置
    dq=0.0,     # 静止
    tau=0.0,    # 无前馈力矩
    kp=80.0,    # 位置增益
    kd=2.0      # 速度增益
)
```

### 2. 正弦波运动设置

```python
# 为脚踝关节设置正弦波运动
lowcmd.set_sine_wave_motors(
    motor_ids=[4, 5, 10, 11],  # 左右脚踝的pitch和roll
    amplitude=0.3,              # 0.3弧度幅度
    frequency=0.5               # 0.5Hz频率
)

# 在循环中更新位置
while True:
    lowcmd.update_positions()
    # 发送lowcmd数据...
    time.sleep(0.02)  # 50Hz
```

### 3. JSON格式输出

```python
# 转换为JSON格式
lowcmd_dict = lowcmd.to_dict()
import json
json_data = json.dumps(lowcmd_dict, indent=2)

# 示例输出：
{
  "mode_pr": 1,
  "mode_machine": 1,
  "motors": [
    {
      "id": 0,
      "mode": 1,
      "q": 0.0,
      "dq": 0.0,
      "tau": 0.0,
      "kp": 100.0,
      "kd": 3.0
    }
    // ... 总共35个电机
  ]
}
```

## ROS1集成示例

### 1. 发布到/lowcmd话题

```python
import rospy
from geometry_msgs.msg import PoseStamped

# 初始化ROS1
rospy.init_node('lowcmd_publisher')
pub = rospy.Publisher('/lowcmd', PoseStamped, queue_size=10)

# 创建消息
msg = PoseStamped()
msg.header.stamp = rospy.Time.now()
msg.header.frame_id = "robot_base"

# 将电机位置映射到pose
msg.pose.position.x = motors[4].q   # 左脚踝pitch
msg.pose.position.y = motors[5].q   # 左脚踝roll
msg.pose.orientation.x = motors[10].q  # 右脚踝pitch
msg.pose.orientation.y = motors[11].q  # 右脚踝roll

# 发布消息
pub.publish(msg)
```

### 2. 文件桥接方式

```python
import json
import time

# 写入桥接文件
bridge_data = {
    'timestamp': time.time(),
    'sequence': msg_count,
    'joints': {
        4: left_ankle_pitch,
        5: left_ankle_roll,
        10: right_ankle_pitch,
        11: right_ankle_roll
    }
}

with open('/tmp/lowcmd_data.json', 'w') as f:
    json.dump(bridge_data, f)
```

## 实际应用示例

### 1. 脚踝正弦波运动

```python
# 左右脚踝相位的正弦波运动
import numpy as np
import time

def ankle_sine_wave(t):
    """生成脚踝正弦波运动轨迹"""
    # 左脚踝
    left_pitch = 0.3 * np.sin(2 * np.pi * 0.5 * t)      # 0.3弧度, 0.5Hz
    left_roll = 0.2 * np.sin(2 * np.pi * 0.5 * t + np.pi/4)  # 相位偏移90度

    # 右脚踝 (相位相反)
    right_pitch = -left_pitch
    right_roll = -left_roll

    return {
        4: left_pitch,    # left_ankle_pitch
        5: left_roll,     # left_ankle_roll
        10: right_pitch,  # right_ankle_pitch
        11: right_roll    # right_ankle_roll
    }

# 使用示例
start_time = time.time()
while True:
    t = time.time() - start_time
    ankle_positions = ankle_sine_wave(t)

    # 设置电机位置
    for joint_id, position in ankle_positions.items():
        lowcmd.set_motor_command(joint_id, q=position)

    # 发送命令
    send_lowcmd(lowcmd)
    time.sleep(0.02)  # 50Hz
```

### 2. 完整的低级别控制

```python
from python_lowcmd_builder import create_lowcmd_for_g1

# 创建专门的G1 LowCmd
lowcmd = create_lowcmd_for_g1()

# 设置PID参数
joint_params = {
    4: {'kp': 80.0, 'kd': 2.0},   # 左脚踝 - 小齿轮
    5: {'kp': 80.0, 'kd': 2.0},   # 左脚踝 - 小齿轮
    10: {'kp': 80.0, 'kd': 2.0},  # 右脚踝 - 小齿轮
    11: {'kp': 80.0, 'kd': 2.0}   # 右脚踝 - 小齿轮
}

for joint_id, params in joint_params.items():
    lowcmd.set_motor_command(joint_id, **params)

# 开始正弦波运动
lowcmd.set_sine_wave_motors([4, 5, 10, 11], amplitude=0.3, frequency=0.5)

# 控制循环
rate = rospy.Rate(50)  # 50Hz
while not rospy.is_shutdown():
    lowcmd.update_positions()

    # 发送到RK3588
    send_to_rk3588(lowcmd.to_dict())

    rate.sleep()
```

## 网络传输格式

### 二进制打包格式

```python
# MotorCmd二进制格式 (33字节)
# mode(1) + q(4) + dq(4) + tau(4) + kp(4) + kd(4) + reserve(12)
binary_data = struct.pack('<BfffffIII',
    mode, q, dq, tau, kp, kd, reserve[0], reserve[1], reserve[2])

# 完整LowCmd大小约为: 2 + 35*33 + 16 + 4 = 1163字节
```

### JSON传输格式

```json
{
  "timestamp": 1703123456.789,
  "sequence": 12345,
  "mode_pr": 1,
  "mode_machine": 1,
  "motors": [
    {"id": 4, "q": 0.123, "dq": 0.456, "kp": 80.0, "kd": 2.0}
  ]
}
```

## 注意事项

1. **单位**: 所有角度使用弧度，力矩使用Nm
2. **频率**: 推荐控制频率为50-500Hz
3. **增益参数**: 根据电机类型选择合适的kp/kd值
4. **CRC校验**: 确保数据完整性
5. **模式设置**: 正确设置mode_pr和mode_machine
6. **关节限制**: 注意各关节的运动范围限制

## 相关文件

- `python_lowcmd_builder.py` - LowCmd构建工具类
- `ros1_lowcmd_bridge_example.py` - ROS1桥接示例
- `ros1_to_dds_bridge.py` - 完整的ROS1到DDS桥接
- `deploy_ros1_to_dds.sh` - 部署脚本