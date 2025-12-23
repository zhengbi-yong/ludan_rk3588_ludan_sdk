# Motor Controller with Enable Functionality

## 概述

这个版本的 motor_controller 增加了使能功能，允许通过ROS2消息指定具体的motor ID发送使能指令到周立功设备。

## 编译

```bash
cd /home/linaro/ludan_sdk/build
make motor_controller_with_enable
```

## 运行

```bash
cd /home/linaro/ludan_sdk
source /home/linaro/controller_ws/install/setup.bash
./build/bin/motor_controller_with_enable
```

## 使能指令使用方法

### 简化工具 (推荐)

使用提供的Python工具，它会自动处理30个电机的数组要求：

```bash
cd /home/linaro/ludan_sdk
source /home/linaro/controller_ws/install/setup.bash

# 使能电机4
python3 scripts/enable_motor.py 4 1

# 禁使能电机4
python3 scripts/enable_motor.py 4 0

# 发送自定义指令 (0xFC)
python3 scripts/enable_motor.py 4 252
```

### 消息格式

使能指令使用 `xixilowcmd::msg::LowCmd` 消息格式，其中：
- `motor_cmd[i].id` = 电机ID (0-29)
- `motor_cmd[i].q` = 使能指令类型 (转换为uint8)
  - `1.0` = 使能 (发送 FF FF FF FF FF FF FF FC)
  - `0.0` = 禁使能 (发送 FF FF FF FF FF FF FF FD)
  - 其他值 = 自定义指令 (发送 FF FF FF FF FF FF FF [指令])

### ROS2 命令行使用

由于LowCmd要求包含30个电机，建议使用简化工具。如果必须直接使用ROS2命令，可以使用以下方法：

#### 方法1: 使用完整30电机格式
```bash
# 使能电机4 (包含所有30个电机，其他电机q值为0)
ros2 topic pub --once /motor_enable xixilowcmd/msg/LowCmd "
motor_cmd: [
  {id: 0, q: 0.0}, {id: 1, q: 0.0}, {id: 2, q: 0.0}, {id: 3, q: 0.0},
  {id: 4, q: 1.0},  # 使能电机4
  {id: 5, q: 0.0}, {id: 6, q: 0.0}, {id: 7, q: 0.0},
  {id: 8, q: 0.0}, {id: 9, q: 0.0}, {id: 10, q: 0.0},
  {id: 11, q: 0.0}, {id: 12, q: 0.0}, {id: 13, q: 0.0},
  {id: 14, q: 0.0}, {id: 15, q: 0.0}, {id: 16, q: 0.0},
  {id: 17, q: 0.0}, {id: 18, q: 0.0}, {id: 19, q: 0.0},
  {id: 20, q: 0.0}, {id: 21, q: 0.0}, {id: 22, q: 0.0},
  {id: 23, q: 0.0}, {id: 24, q: 0.0}, {id: 25, q: 0.0},
  {id: 26, q: 0.0}, {id: 27, q: 0.0}, {id: 28, q: 0.0},
  {id: 29, q: 0.0}
]"
```

#### 方法2: 使用Python脚本
```python
# 创建 enable_single_motor.py 文件
import rclpy
from xixilowcmd.msg import LowCmd, MotorCmd

def enable_single_motor(motor_id, command):
    rclpy.init()
    node = rclpy.create_node('enable_node')
    pub = node.create_publisher(LowCmd, '/motor_enable', 10)

    msg = LowCmd()
    msg.motor_cmd = []

    for i in range(30):
        motor_cmd = MotorCmd()
        motor_cmd.id = i
        motor_cmd.q = float(command) if i == motor_id else 0.0
        motor_cmd.dq = 0.0
        motor_cmd.tau = 0.0
        motor_cmd.kp = 0.0
        motor_cmd.kd = 0.0
        motor_cmd.mode = 0
        msg.motor_cmd.append(motor_cmd)

    pub.publish(msg)
    print(f"使能指令已发送: Motor {motor_id} -> {command}")
    rclpy.shutdown()

# 使用
enable_single_motor(4, 1)  # 使能电机4
```

#### 2. 禁使能单个电机
```bash
# 修改上面的脚本，将 enable_cmd 改为 0
enable_pub.send_enable(4, 0)  # 禁使能电机4
```

#### 3. 同时使能多个电机
```python
def send_multiple_enable(enable_pub, motor_ids, enable_cmd):
    msg = LowCmd()
    msg.motor_cmd = []

    for motor_id in motor_ids:
        motor_cmd = MotorCmd()
        motor_cmd.id = motor_id
        motor_cmd.q = float(enable_cmd)
        msg.motor_cmd.append(motor_cmd)

    enable_pub.publisher_.publish(msg)
    print(f"发送使能指令: Motors {motor_ids}, 指令 {enable_cmd}")

# 使能脚踝关节
send_multiple_enable(enable_pub, [4, 5, 10, 11], 1)
```

#### 4. 发送自定义使能指令
```python
# 发送自定义指令 (例如 0xFC)
enable_pub.send_enable(4, 252)  # 252 = 0xFC
```

## 电机ID与CAN ID映射

- **电机 1-15** → **CAN ID 0x01-0x0F**
- **电机 16-30** → **CAN ID 0x10-0x1F**

## CAN Frame 格式

- **使能**: `FF FF FF FF FF FF FF FC`
- **禁使能**: `FF FF FF FF FF FF FF FD`
- **自定义**: `FF FF FF FF FF FF FF [指令]`

## 周立功设备连接

- **目标IP**: 192.168.1.5
- **目标端口**: 8002
- **协议**: TCP
- **数据包格式**: `[CAN_ID(1 byte) | DLC(1 byte) | DATA(8 bytes)]`

## 日志输出示例

```
================================================================================
🔧 收到使能指令，包含 1 个电机的指令
--------------------------------------------------------------------------------
   🎯 Motor  4 (CAN ID: 0x04) -> 使能
🔗 连接周立功设备: 192.168.1.5:8002
✅ 周立功设备连接成功
📤 发送使能CAN frame到周立功设备 (Motor 4, CAN ID: 0x04)...
✅ 使能指令已发送
================================================================================
```

## 注意事项

1. **Motor ID范围**: 0-29
2. **有效的Motor ID**: 1-30 (映射到CAN ID 0x01-0x1F)
3. **使能指令范围**: 0-255 (uint8)
4. **连接状态**: 程序会自动管理周立功设备的连接
5. **错误处理**: 无效的Motor ID会被拒绝并显示错误信息

## 调试模式

使用 `-v` 参数启动以启用详细日志：
```bash
./build/bin/motor_controller_with_enable -v
```

这将显示详细的CAN frame发送信息。