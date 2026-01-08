# can_send_test 操作指南

## 概述

`can_send_test` 是一个用于模拟 DM 电机 CAN 数据的测试工具。它生成 30 个电机的模拟反馈数据，并通过共享内存传递给 `can_motor_feedback_publisher` 进行测试。

## 功能特性

- ✅ 模拟 30 个 DM 电机的 CAN 反馈数据
- ✅ 支持通过 JSON 配置文件自定义电机数据
- ✅ 支持多种数据生成模式（固定值、增量、正弦波）
- ✅ 兼容 DM 电机 CAN 格式（8 字节）
- ✅ 共享内存输出，无需真实硬件

## 快速开始

### 1. 编译

```bash
cd /home/linaro/ludan_sdk/multi_port_motor_feedback/build
cmake ..
make can_send_test
```

### 2. 运行（默认模式）

```bash
cd /home/linaro/ludan_sdk/multi_port_motor_feedback/build
./can_send_test
```

程序会：
- 创建共享内存 `/can_motor_test_shm`
- 以默认配置生成 30 个电机的模拟数据
- 每 100ms 更新一次数据

### 3. 启动 ROS2 发布节点（测试模式）

```bash
source /home/linaro/ludan_sdk/multi_port_motor_feedback/multi_port_motor_feedback_ros2/install/setup.bash
ros2 run multi_port_motor_feedback can_motor_feedback_publisher --test-mode
```

### 4. 验证数据流

```bash
# 查看话题数据
ros2 topic echo /motor_feedback
```

---

## 命令行参数

```
Usage: ./can_send_test [options]

Options:
  --config <file>   JSON configuration file
  --interval <n>    Send interval in ms (default: 100)
  --quiet           Don't print each frame
  -h, --help        Show this help message
```

### 参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--config` | 指定 JSON 配置文件路径 | 无 |
| `--interval` | 数据发送间隔（毫秒） | 100 |
| `--quiet` | 减少输出日志 | false |

---

## JSON 配置文件格式

### 基本结构

```json
{
  "send_interval_ms": 100,
  "verbose": true,
  "p_max": 6.28318,
  "v_max": 45.0,
  "t_max": 18.0,
  "motors": [
    {
      "id": 1,
      "enabled": true,
      "position": 0.0,
      "velocity": 0.0,
      "torque": 0.0,
      "temp_mos": 25.0,
      "temp_rotor": 25.0,
      "error_code": 1
    }
  ]
}
```

### 全局配置项

| 字段 | 类型 | 说明 | 默认值 |
|------|------|------|--------|
| `send_interval_ms` | int | 发送间隔（毫秒） | 100 |
| `verbose` | bool | 是否打印详细日志 | true |
| `p_max` | float | 位置最大值（弧度） | 2π (6.28318) |
| `v_max` | float | 速度最大值（rad/s） | 45.0 |
| `t_max` | float | 转矩最大值（Nm） | 18.0 |

### 电机配置项

| 字段 | 类型 | 说明 | 默认值 |
|------|------|------|--------|
| `id` | int | 电机 ID (1-30) | 必填 |
| `enabled` | bool | 是否启用该电机 | true |
| `position` | float | 位置（弧度） | 0.0 |
| `velocity` | float | 速度（rad/s） | 0.0 |
| `torque` | float | 转矩（Nm） | 0.0 |
| `temp_mos` | float | MOS 温度（°C） | 25.0 |
| `temp_rotor` | float | 转子温度（°C） | 25.0 |
| `error_code` | int | 错误码 | 1 (使能) |
| `data_mode` | int | 数据模式 | 0 |
| `pos_increment` | float | 位置增量（模式1） | 0.0 |
| `vel_amplitude` | float | 速度幅度（模式2） | 0.0 |
| `vel_frequency` | float | 速度频率 Hz（模式2） | 0.0 |
| `p_max` | float | 单个电机的位置最大值 | 使用全局值 |
| `v_max` | float | 单个电机的速度最大值 | 使用全局值 |
| `t_max` | float | 单个电机的转矩最大值 | 使用全局值 |

### 数据模式 (data_mode)

| 模式 | 名称 | 说明 |
|------|------|------|
| 0 | 固定值 | 使用配置的固定值（默认） |
| 1 | 增量 | 位置按 `pos_increment` 逐帧增加 |
| 2 | 正弦波 | 速度按 `vel_amplitude * sin(2π * vel_frequency * t)` 变化 |
| 3 | 自定义 | 增量 + 正弦波组合 |

### 错误码 (error_code)

| 值 | 名称 | 说明 |
|-----|------|------|
| 0x0 | 失能 | 电机失能 |
| 0x1 | 使能 | 电机正常使能 |
| 0x8 | 超压 | 过电压 |
| 0x9 | 欠压 | 欠电压 |
| 0xA | 过电流 | 过电流 |
| 0xB | MOS过温 | MOS 过热 |
| 0xC | 线圈过温 | 电机线圈过热 |
| 0xD | 通讯丢失 | 通讯丢失 |
| 0xE | 过载 | 过载 |

---

## 配置示例

### 示例 1: 默认配置（所有电机固定值）

```json
{
  "send_interval_ms": 100,
  "motors": [
    {
      "id": 1,
      "enabled": true,
      "position": 1.57,
      "velocity": 5.0,
      "torque": 1.0
    },
    {
      "id": 2,
      "enabled": true,
      "position": -1.57,
      "velocity": -5.0,
      "torque": -1.0
    }
  ]
}
```

运行：
```bash
./can_send_test --config motor_config.json
```

### 示例 2: 正弦波速度（模拟运动）

```json
{
  "send_interval_ms": 50,
  "motors": [
    {
      "id": 1,
      "enabled": true,
      "data_mode": 2,
      "vel_amplitude": 10.0,
      "vel_frequency": 0.5,
      "position": 0.0
    },
    {
      "id": 2,
      "enabled": true,
      "data_mode": 2,
      "vel_amplitude": 15.0,
      "vel_frequency": 0.3,
      "position": 1.57
    }
  ]
}
```

### 示例 3: 增量位置（模拟连续转动）

```json
{
  "send_interval_ms": 100,
  "motors": [
    {
      "id": 1,
      "enabled": true,
      "data_mode": 1,
      "pos_increment": 0.1,
      "velocity": 5.0,
      "torque": 1.0
    }
  ]
}
```

### 示例 4: 自定义范围参数

```json
{
  "send_interval_ms": 100,
  "p_max": 12.56636,
  "v_max": 90.0,
  "t_max": 36.0,
  "motors": [
    {
      "id": 1,
      "enabled": true,
      "position": 3.14,
      "velocity": 45.0,
      "torque": 18.0
    }
  ]
}
```

### 示例 5: 混合模式（不同电机不同模式）

```json
{
  "send_interval_ms": 100,
  "motors": [
    {
      "id": 1,
      "data_mode": 0,
      "position": 0.0,
      "velocity": 0.0,
      "torque": 0.0
    },
    {
      "id": 2,
      "data_mode": 1,
      "pos_increment": 0.05,
      "velocity": 2.0
    },
    {
      "id": 3,
      "data_mode": 2,
      "vel_amplitude": 8.0,
      "vel_frequency": 1.0
    },
    {
      "id": 4,
      "data_mode": 3,
      "pos_increment": 0.02,
      "vel_amplitude": 5.0,
      "vel_frequency": 0.5
    }
  ]
}
```

---

## 输出说明

### 控制台输出

```
=== CAN Motor Send Test (Shared Memory) ===
Shared Memory: /can_motor_test_shm
Total motors: 30 (10+7+7+6)
Send interval: 100ms
===========================================
[Shared Memory] Created: /can_motor_test_shm (size: 4024 bytes)

[Send] Starting motor data generation (Ctrl+C to stop)...
[TX] Sequence 1 | Frames: 30
  Motor  1 Data: 11 00 00 00 00 00 00 00
  Motor 02 Data: 12 00 00 00 00 00 00 00
  Motor 03 Data: 13 00 00 00 00 00 00 00
  ...
```

### 数据格式说明

每个 CAN 帧包含 8 字节：

| 字节 | 位 | 内容 |
|------|-----|------|
| D[0] | [3:0] | 电机 ID |
| D[0] | [7:4] | 错误码 |
| D[1] | [15:8] | 位置高字节 |
| D[2] | [7:0] | 位置低字节 |
| D[3] | [11:4] | 速度高字节 |
| D[4] | [3:0] | 速度低字节 |
| D[4] | [7:4] | 转矩高字节 |
| D[5] | [7:0] | 转矩低字节 |
| D[6] | - | MOS 温度 |
| D[7] | - | 转子温度 |

---

## 故障排除

### 问题 1: 启动失败

```
ERROR: Failed to open config file: xxx.json
```

**解决方法**: 检查 JSON 文件路径是否正确，文件是否存在。

### 问题 2: JSON 解析失败

```
ERROR: Failed to parse JSON config
```

**解决方法**: 使用 JSON 验证工具检查配置文件格式是否正确。

### 问题 3: ROS2 发布节点无法连接

```
[Test Mode] Failed to open shared memory: No such file or directory
```

**解决方法**: 确保 `can_send_test` 正在运行，并已创建共享内存。

### 问题 4: 没有数据输出

检查以下几点：
1. 电机是否启用 (`enabled: true`)
2. 共享内存是否创建成功
3. ROS2 节点是否使用 `--test-mode` 参数

---

## 电机分布

| 端口 | 电机 ID | 数量 |
|------|--------|------|
| vcan0 (模拟端口 0) | 1-10 | 10 |
| vcan1 (模拟端口 1) | 11-17 | 7 |
| vcan2 (模拟端口 2) | 18-24 | 7 |
| vcan3 (模拟端口 3) | 25-30 | 6 |
| **总计** | **1-30** | **30** |

---

## 完整测试流程

### 终端 1: 启动数据发送

```bash
cd /home/linaro/ludan_sdk/multi_port_motor_feedback/build
./can_send_test --config my_config.json
```

### 终端 2: 启动 ROS2 发布节点

```bash
source /home/linaro/ludan_sdk/multi_port_motor_feedback/multi_port_motor_feedback_ros2/install/setup.bash
ros2 run multi_port_motor_feedback can_motor_feedback_publisher --test-mode
```

### 终端 3: 启动订阅者（可选）

```bash
cd /home/linaro/motor_feedback_api/motor_feedback_api
python3 motor_subscriber.py
```

### 终端 4: 监控话题（可选）

```bash
source /home/linaro/ludan_sdk/multi_port_motor_feedback/multi_port_motor_feedback_ros2/install/setup.bash
ros2 topic echo /motor_feedback
```

---

## 退出程序

按 `Ctrl+C` 退出，程序会自动清理共享内存。

---

## 注意事项

1. **必须先启动 `can_send_test`，再启动 `can_motor_feedback_publisher --test-mode`**
2. 共享内存名称固定为 `/can_motor_test_shm`
3. 电机 ID 范围：1-30
4. 温度范围：0-125°C
5. 数据更新频率：默认 10Hz (100ms 间隔)

---

## 技术支持

如有问题，请检查：
1. 系统是否支持 POSIX 共享内存 (`shm_open`, `mmap`)
2. ROS2 环境是否正确配置
3. 是否有权限创建共享内存
