# Ludan 主机电机标零功能实现任务

## 任务背景

在 ludan (ROS1) 主机中实现电机标零功能，通过 relay 节点将命令转发到 linaro 的 rk3588，最终控制 DM 电机标零。

## 系统架构

```
ludan (ROS1) -> relay -> linaro (rk3588) -> listener_8888_to_ros2.py -> ROS2 /lowcmd -> motor_controller_with_enable -> DM 电机
```

## 标零命令格式

DM 电机标零命令数据帧（8字节）：
```
FF FF FF FF FF FF FF FE
```

## 实现要求

在 ludan (ROS1) 主机中创建一个 Python 脚本 `send_motor_zero.py`，功能如下：

### 1. 脚本功能

```bash
# 标零单个电机
python3 send_motor_zero.py 1

# 标零多个电机
python3 send_motor_zero.py 1 2 3

# 标零电机范围
python3 send_motor_zero.py 1-30
```

### 2. 消息格式

需要发布到 ROS1 的 `/lowcmd` topic，消息类型为 `unitree_legged_msgs::LowCmd`。

**关键点：使用 `mode=255` 作为标零命令的标识**

消息结构：
```python
msg = LowCmd()
motor_cmd = MotorCmd()
motor_cmd.id = motor_id          # 电机 ID (1-30)
motor_cmd.mode = 255              # 标零命令标识（特殊值）
motor_cmd.q = 0.0                 # 位置（标零时为0）
motor_cmd.dq = 0.0                # 速度（标零时为0）
motor_cmd.kp = 0.0                # Kp（标零时为0）
motor_cmd.kd = 0.0                # Kd（标零时为0）
motor_cmd.tau = 0.0               # 力矩（标零时为0）
msg.motor_cmd.append(motor_cmd)
```

### 3. 实现示例

```python
#!/usr/bin/env python3
import rospy
from unitree_legged_msgs.msg import LowCmd, MotorCmd
import sys

def send_zero_command(motor_ids):
    """
    发送标零命令到指定电机

    Args:
        motor_ids: 电机ID列表，如 [1] 或 [1, 2, 3]
    """
    pub = rospy.Publisher('/lowcmd', LowCmd, queue_size=10)

    # 等待发布器连接
    rospy.sleep(0.1)

    msg = LowCmd()

    # 为每个电机创建标零命令
    for motor_id in motor_ids:
        if 1 <= motor_id <= 30:
            cmd = MotorCmd()
            cmd.id = motor_id
            cmd.mode = 255        # 标零命令标识（关键！）
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
    # 初始化 ROS1 节点
    rospy.init_node('motor_zero_sender', anonymous=True)

    # 解析命令行参数
    motor_ids = []

    if len(sys.argv) < 2:
        print('用法: python3 send_motor_zero.py <motor_id> [<motor_id> ...]')
        print('示例:')
        print('  python3 send_motor_zero.py 1           # 标零电机1')
        print('  python3 send_motor_zero.py 1 2 3       # 标零电机1,2,3')
        print('  python3 send_motor_zero.py 1-30        # 标零电机1-30')
        sys.exit(1)

    # 解析电机ID参数
    for arg in sys.argv[1:]:
        if '-' in arg:
            # 范围模式，如 "1-10"
            try:
                start, end = map(int, arg.split('-'))
                motor_ids.extend(range(start, end + 1))
            except ValueError:
                print(f'错误: 无效的范围格式 "{arg}"')
                sys.exit(1)
        else:
            # 单个ID
            try:
                motor_ids.append(int(arg))
            except ValueError:
                print(f'错误: 无效的电机ID "{arg}"')
                sys.exit(1)

    # 发送标零命令
    send_zero_command(motor_ids)

if __name__ == '__main__':
    main()
```

### 4. 关键注意事项

1. **mode=255**: 这是标零命令的关键标识，linaro 主机会检测这个值并发送 `FF FF FF FF FF FF FF FE` 数据帧
2. **消息类型**: 使用 `unitree_legged_msgs::LowCmd` 和 `MotorCmd`
3. **Topic**: 发布到 `/lowcmd` topic
4. **电机ID范围**: 1-30

### 5. 测试流程

```bash
# 步骤1: 确保所有电机已使能
# 在 linaro 主机上执行:
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable 1-30

# 步骤2: 启动 motor_controller_with_enable 接收命令
# 在 linaro 主机上执行:
source /home/linaro/controller_ws/install/setup.bash
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable-motor-cmd --auto-start -v

# 步骤3: 在 ludan 主机上发送标零命令
python3 send_motor_zero.py 1-30
```

### 6. 预期结果

在 linaro 主机的 motor_controller_with_enable 中应该看到：
```
[DEBUG] 收到 30 个电机命令: ID1(mode=255) ID2(mode=255) ...
[ZERO] 已发送标零命令到电机 1
[ZERO] 已发送标零命令到电机 2
...
[ZERO] 已发送标零命令到电机 30
```

## 相关文件位置

- ludan 主机 ROS1 消息定义: `unitree_legged_msgs`
- linaro 主机接收代码: `/home/linaro/ludan_sdk/example/g1/low_level/motor_controller_with_enable.cpp`
- linaro 主机测试脚本: `/home/linaro/controller_ws/install/motor_zero/lib/motor_zero/zero_motors.py`

## 参考资料

DM 电机标零命令数据帧: `FF FF FF FF FF FF FF FE`（与使能命令 `FF FF FF FF FF FF FF FC` 类似）
