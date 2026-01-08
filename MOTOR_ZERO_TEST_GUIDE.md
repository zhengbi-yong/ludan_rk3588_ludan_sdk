# 电机标零功能测试指南

## Linaro 主机部分（已完成）

### 1. 已实现的功能

1. **motor_controller_with_enable.cpp**
   - 添加了 `mode=255` 检测逻辑
   - 添加了 `SendZeroCommand()` 方法
   - 标零命令数据帧：`FF FF FF FF FF FF FF FE`

2. **motor_zero ROS2 包**
   - 脚本：`/home/linaro/controller_ws/install/motor_zero/lib/motor_zero/zero_motors.py`

### 2. 测试方法（在 linaro 主机）

```bash
# 步骤1: 使能所有电机
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable 1-30

# 步骤2: 启动电机命令接收
source /home/linaro/controller_ws/install/setup.bash
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable-motor-cmd --auto-start -v

# 步骤3: 在另一个终端测试标零命令
source /home/linaro/controller_ws/install/setup.bash

# 标零单个电机
ros2 run motor_zero zero_motors.py 1

# 标零多个电机
ros2 run motor_zero zero_motors.py 1 2 3

# 标零所有电机
ros2 run motor_zero zero_motors.py 1-30
```

### 3. 预期输出

```
[DEBUG] 收到 1 个电机命令: ID1(mode=255)
[ZERO] 已发送标零命令到电机 1
```

## Ludan 主机部分（需要 AI 工具实现）

### 给 AI 工具的完整提示

请将以下内容发送给 ludan 主机的 AI 工具：

---

**任务：在 ludan (ROS1) 主机中实现电机标零功能**

## 任务描述

在 ludan (ROS1) 主机中创建一个 Python 脚本 `send_motor_zero.py`，通过发布到 `/lowcmd` topic 发送电机标零命令。

## 系统流程

```
ludan (ROS1) -> relay -> linaro (rk3588) -> listener_8888_to_ros2.py -> ROS2 /lowcmd -> motor_controller_with_enable -> DM 电机
```

## 标零命令协议

DM 电机标零命令数据帧（8字节）：`FF FF FF FF FF FF FF FE`

**关键：使用 `mode=255` 作为标零命令的标识**

## 实现要求

创建脚本 `/path/to/ludan/send_motor_zero.py`，支持以下用法：

```bash
# 标零单个电机
python3 send_motor_zero.py 1

# 标零多个电机
python3 send_motor_zero.py 1 2 3

# 标零电机范围
python3 send_motor_zero.py 1-30
```

## 消息格式

发布到 ROS1 的 `/lowcmd` topic，使用 `unitree_legged_msgs::LowCmd` 消息类型：

```python
from unitree_legged_msgs.msg import LowCmd, MotorCmd
import rospy

msg = LowCmd()
motor_cmd = MotorCmd()
motor_cmd.id = motor_id      # 电机 ID (1-30)
motor_cmd.mode = 255          # 标零命令标识（关键！）
motor_cmd.q = 0.0
motor_cmd.dq = 0.0
motor_cmd.kp = 0.0
motor_cmd.kd = 0.0
motor_cmd.tau = 0.0
msg.motor_cmd.append(motor_cmd)

pub = rospy.Publisher('/lowcmd', LowCmd, queue_size=10)
pub.publish(msg)
```

## 完整代码模板

```python
#!/usr/bin/env python3
import rospy
from unitree_legged_msgs.msg import LowCmd, MotorCmd
import sys

def send_zero_command(motor_ids):
    """发送标零命令到指定电机"""
    pub = rospy.Publisher('/lowcmd', LowCmd, queue_size=10)
    rospy.sleep(0.1)  # 等待发布器连接

    msg = LowCmd()

    for motor_id in motor_ids:
        if 1 <= motor_id <= 30:
            cmd = MotorCmd()
            cmd.id = motor_id
            cmd.mode = 255        # 标零命令标识
            cmd.q = 0.0
            cmd.dq = 0.0
            cmd.kp = 0.0
            cmd.kd = 0.0
            cmd.tau = 0.0
            msg.motor_cmd.append(cmd)
        else:
            rospy.logwarn(f'无效的电机ID: {motor_id}')

    if msg.motor_cmd:
        pub.publish(msg)
        rospy.loginfo(f'已发送标零命令到电机: {motor_ids}')

def main():
    rospy.init_node('motor_zero_sender', anonymous=True)

    if len(sys.argv) < 2:
        print('用法: python3 send_motor_zero.py <motor_id> [<motor_id> ...]')
        print('示例:')
        print('  python3 send_motor_zero.py 1           # 标零电机1')
        print('  python3 send_motor_zero.py 1 2 3       # 标零电机1,2,3')
        print('  python3 send_motor_zero.py 1-30        # 标零电机1-30')
        sys.exit(1)

    motor_ids = []
    for arg in sys.argv[1:]:
        if '-' in arg:
            start, end = map(int, arg.split('-'))
            motor_ids.extend(range(start, end + 1))
        else:
            motor_ids.append(int(arg))

    send_zero_command(motor_ids)

if __name__ == '__main__':
    main()
```

## 测试流程

1. 确保 relay 节点正在运行
2. 在 ludan 主机执行：`python3 send_motor_zero.py 1-30`
3. 在 linaro 主机观察 motor_controller_with_enable 的输出

## 预期结果

linaro 主机应显示：
```
[DEBUG] 收到 30 个电机命令: ID1(mode=255) ID2(mode=255) ...
[ZERO] 已发送标零命令到电机 1
[ZERO] 已发送标零命令到电机 2
...
```

---

## 关键点总结

| 组件 | 关键值 |
|------|--------|
| 标零命令数据帧 | `FF FF FF FF FF FF FF FE` |
| 标零命令标识 | `mode = 255` |
| ROS1 Topic | `/lowcmd` |
| ROS2 Topic | `/lowcmd` |
| 电机ID范围 | 1-30 |

## 文件位置

| 文件 | 路径 |
|------|------|
| Linaro 完整提示 | `/home/linaro/ludan_sdk/PROMPT_FOR_LUDAN_AI.md` |
| Linaro 测试脚本 | `/home/linaro/controller_ws/install/motor_zero/lib/motor_zero/zero_motors.py` |
| Linaro 可执行文件 | `/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable` |
