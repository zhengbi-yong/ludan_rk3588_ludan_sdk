# Motor Feedback ROS2 Package

DM电机反馈ROS2发布/订阅包，用于获取ID为1-30电机的实时反馈数据。

## 功能特性

- **motor_feedback_publisher**: 从ZLG CAN设备读取DM电机反馈并发布到ROS2话题
- **motor_feedback_subscriber**: 订阅电机反馈话题并显示状态
- 支持DM电机协议解码
- 兼容 `motor_controller_with_enable`

## 目录结构

```
motor_feedback/
├── CMakeLists.txt
├── package.xml
├── msg/
│   └── MotorFeedback.msg      # ROS2消息定义
├── src/
│   ├── motor_feedback_publisher.cpp
│   └── motor_feedback_subscriber.cpp
├── launch/
│   └── motor_feedback.launch.py
└── README.md
```

## 构建方法

```bash
# 1. 进入ROS2工作空间
cd /home/linaro/controller_ws

# 2. 创建软链接到motor_feedback包
ln -s /home/linaro/ludan_sdk/example/g1/motor_feedback src/motor_feedback

# 3. 构建
colcon build --packages-select motor_feedback

# 4. Source工作空间
source install/setup.bash
```

## 使用方法

### 方法1: 使用launch文件启动

```bash
# 启动发布器和订阅器
ros2 launch motor_feedback motor_feedback.launch.py

# 指定参数
ros2 launch motor_feedback motor_feedback.launch.py zlg_ip:=192.168.1.5 channel:=2
```

### 方法2: 分别启动节点

```bash
# 终端1: 启动发布器
ros2 run motor_feedback motor_feedback_publisher --ros-args -p zlg_ip:=192.168.1.5 -p channel:=2

# 终端2: 启动订阅器
ros2 run motor_feedback motor_feedback_subscriber

# 终端3: 查看话题数据
ros2 topic echo /motor_feedback
```

### 方法3: 与motor_controller_with_enable配合使用

```bash
# 终端1: 使能电机并监听反馈
cd /home/linaro/ludan_sdk/build
./bin/motor_controller_with_enable 192.168.1.5:8002 -c 2 --receive-and-enable 1-30 --receive-duration 60

# 终端2: 启动ROS2订阅器查看数据
ros2 run motor_feedback motor_feedback_subscriber
```

## ROS2话题

**Topic**: `/motor_feedback`

**Type**: `motor_feedback_msgs/msg/MotorFeedback`

**字段**:
| 字段 | 类型 | 说明 |
|------|------|------|
| stamp | builtin_interfaces/Time | 时间戳 |
| motor_id | int32 | 电机ID (1-30) |
| can_id | int32 | CAN ID |
| error | int32 | 错误码 |
| position | int16 | 位置原始值 |
| velocity | int16 | 速度原始值 |
| torque | int16 | 力矩原始值 |
| temp_mos | int8 | MOS温度 |
| temp_rotor | int8 | 转子温度 |
| can_data | uint8[8] | 原始CAN数据 |

## 参数配置

### Publisher参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| zlg_ip | 192.168.1.5 | ZLG设备IP |
| zlg_port | 8002 | ZLG设备端口 |
| channel | 2 | CAN通道号 |
| arb_baud | 1000000 | 仲裁段波特率 |
| data_baud | 5000000 | 数据段波特率 |
| publish_rate | 100.0 | 发布频率(Hz) |

### Subscriber参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| display_rate | 1.0 | 显示刷新频率(Hz) |
| show_raw | false | 是否显示原始CAN数据 |

## DM电机反馈帧格式

```
D[0] D[1] D[2] D[3] D[4] D[5] D[6] D[7]
ID|ERR<<4, POS[15:8], POS[7:0], VEL[11:4], VEL[3:0]|T[11:8], T[7:0], T_MOS, T_Rotor
```

| Byte | 内容 | 说明 |
|------|------|------|
| D[0] | ID[3:0] \| ERR[3:0]<<4 | 电机ID(低4位), 错误码(高4位) |
| D[1-2] | POS[15:0] | 位置 (16位有符号) |
| D[3-4] | VEL[11:0] | 速度 (12位有符号) |
| D[4-5] | T[11:0] | 力矩 (12位有符号) |
| D[6] | T_MOS | MOS温度 |
| D[7] | T_Rotor | 转子温度 |

### 错误码定义

| 错误码 | 含义 |
|--------|------|
| 0x0 | 正常/失能 |
| 0x1 | 使能 |
| 0x8 | 过压 |
| 0x9 | 欠压 |
| 0xA | 过流 |
| 0xB | MOS过温 |
| 0xC | 线圈过温 |
| 0xD | 通讯丢失 |
| 0xE | 过载 |

## 输出示例

```
========================================
     DM Motor Feedback Status (ID 1-30)
========================================
     ID    Pos(raw)    Vel(raw)   Tau(raw)    T_MOS   T_Rotor     Error
----------------------------------------------------------------------
======     1       -30105          135          0       44        8  Enabled
======     2       -28432          142          0       43        7  Enabled
----------------------------------------------------------------------
Total motors: 2 | Enabled: 2 | Errors: 0 | Last update: 0.02s ago
========================================
```
