# 四端口电机测试程序 (motor_test10)

## 📋 概述

完整的四端口电机测试系统，用于验证端口 8000、8001、8002、8003 的数据下发功能。

### 端口映射

| 端口 | CAN 通道 | 映射电机 | 说明 |
|------|----------|----------|------|
| 8000 | CAN0 | 电机 1, 2, 3 | 默认映射 |
| 8001 | CAN1 | 电机 4 | 特殊映射 |
| 8002 | CAN2 | 电机 5 | 特殊映射 |
| 8003 | CAN3 | 电机 6 | 特殊映射 |

---

## 🚀 快速开始

### 自动化测试（推荐）

```bash
cd /home/linaro/ludan_sdk/example/g1/motor_test10
./test_all_ports.sh
```

**自动执行流程**：
1. 使能电机 (ID: 1-6)
2. 启动 Motor Controller
3. 启动 UDP Listener
4. 发送测试数据（单次 + 持续 + 正弦波）
5. 显示测试结果

---

## 📝 手动测试步骤

### 1. 使能电机

```bash
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable 1
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable 2
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable 3
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable 4
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable 5
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable --enable 6
```

### 2. 启动控制器（终端1）

```bash
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable \
    --enable-motor-cmd --auto-start
```

### 3. 启动监听器（终端2）

```bash
cd /home/linaro/ludan_sdk/scripts
python3 listener_8888_to_ros2.py 8888
```

### 4. 发送测试数据（终端3）

```bash
cd /home/linaro/ludan_sdk/example/g1/motor_test10

# 测试所有四个端口
python3 four_port_udp_sender.py --all

# 测试单个端口
python3 four_port_udp_sender.py --port 0  # 端口 8000
python3 four_port_udp_sender.py --port 1  # 端口 8001
python3 four_port_udp_sender.py --port 2  # 端口 8002
python3 four_port_udp_sender.py --port 3  # 端口 8003

# 持续发送 (50Hz, 10秒)
python3 four_port_udp_sender.py --continuous 10

# 正弦波测试
python3 four_port_udp_sender.py --sine 10
```

---

## ⏱️ 超时自动停止

**新功能**：Controller 现在具备自动超时检测功能。

### 工作原理

- 当 UDP 数据停止发送时，Controller 会继续发送最后收到的命令值
- 如果 **500ms** 内没有收到新数据，电机自动标记为非活跃状态
- Controller 停止发送该电机的数据

### 日志输出

```
[TIMEOUT] Motor 1 超时 (0.512s > 0.5s), 停止发送
[TIMEOUT] Motor 2 超时 (0.513s > 0.5s), 停止发送
...
[DEBUG] 端口分配: 8000=0 8001=0 8002=0 8003=0 (总计=0)
```

### 优势

- **无需手动停止**：不需要发送 `mode=0` 命令
- **自动清理**：避免忘记停止导致一直发送
- **快速响应**：500ms 后自动停止

详细说明请参考：[TIMEOUT_MECHANISM.md](TIMEOUT_MECHANISM.md)

---

## 📊 数据格式

### UDP 发送的 JSON 格式

```json
{
  "timestamp": 1767532655.063,
  "sequence": 0,
  "motor_cmd": {
    "1": {"q": 0.5, "dq": 0.0, "tau": 0.0, "mode": 1, "kp": 20.0, "kd": 2.0},
    "2": {"q": 0.3, "dq": 0.0, "tau": 0.0, "mode": 1, "kp": 20.0, "kd": 2.0},
    "3": {"q": -0.2, "dq": 0.0, "tau": 0.0, "mode": 1, "kp": 20.0, "kd": 2.0},
    "4": {"q": 0.8, "dq": 0.0, "tau": 0.0, "mode": 1, "kp": 20.0, "kd": 2.0},
    "5": {"q": 0.6, "dq": 0.0, "tau": 0.0, "mode": 1, "kp": 20.0, "kd": 2.0},
    "6": {"q": 0.4, "dq": 0.0, "tau": 0.0, "mode": 1, "kp": 20.0, "kd": 2.0}
  },
  "mode_pr": 0,
  "mode_machine": 1
}
```

### 字段说明

| 字段 | 类型 | 说明 |
|------|------|------|
| `q` | float | 目标位置（弧度）|
| `dq` | float | 目标速度（弧度/秒）|
| `tau` | float | 目标力矩（N·m）|
| `kp` | float | 位置增益 |
| `kd` | float | 速度增益 |
| `mode` | int | 电机模式 (0=非活跃, 1=活跃) |

---

## 🔍 预期结果

### Controller 调试输出

```
[DEBUG] 收到 6 个电机命令: ID1 ID2 ID3 ID4 ID5 ID6
[DEBUG] 端口分配: 8000=3 8001=1 8002=1 8003=1 (总计=6)
```

**解读**：
- `8000=3`: 端口 8000 发送 3 个电机（1, 2, 3）
- `8001=1`: 端口 8001 发送 1 个电机（4）
- `8002=1`: 端口 8002 发送 1 个电机（5）
- `8003=1`: 端口 8003 发送 1 个电机（6）

### CAN 总线发送

```
[SYS] Chnl 0: TxCAN: 3      # CAN0 发送 3 帧 (电机 1,2,3 → 端口 8000)
[SYS] Chnl 1: TxCAN: 1      # CAN1 发送 1 帧 (电机 4 → 端口 8001)
[SYS] Chnl 2: TxCAN: 1      # CAN2 发送 1 帧 (电机 5 → 端口 8002)
[SYS] Chnl 3: TxCAN: 1      # CAN3 发送 1 帧 (电机 6 → 端口 8003)
```

---

## 🛠️ 故障排查

### 问题 1: 端口 8000 发送了不应发送的数据

**原因**：`mode=0` 的电机被标记为活跃

**解决**：
```cpp
// 确保只有 mode != 0 的电机才活跃
if (motor_cmd.mode != 0) {
    active_motors_[motor_id] = true;
}
```

### 问题 2: 电机不转

**检查清单**：
1. 电机是否已使能？
2. `--enable-motor-cmd` 参数是否添加？
3. `--auto-start` 参数是否添加？
4. 端口连接是否成功？

### 问题 3: 端口映射错误

**验证**：
```bash
# 查看调试输出中的端口分配
[DEBUG] 端口分配: 8000=3 8001=1 8002=1 8003=1
```

### 问题 4: 超时检测不工作

**检查**：
1. 确认使用最新编译版本
2. 查看日志中的 `[TIMEOUT]` 消息
3. 验证 UDP 是否确实停止发送

**调整超时时间**：
编辑 `motor_controller_with_enable.cpp` 中的 `COMMAND_TIMEOUT_SECONDS`

---

## 📦 文件结构

```
motor_test10/
├── four_port_udp_sender.py   # UDP 发送脚本
├── test_all_ports.sh          # 自动化测试脚本
├── stop_motors.py             # 手动停止脚本（可选）
├── TIMEOUT_MECHANISM.md       # 超时机制详细说明
├── WHY_CONTINUE_SENDING.md    # 旧版本说明（已过时）
└── README.md                  # 本文档
```

---

## 🔧 高级用法

### 指定目标地址

```bash
python3 four_port_udp_sender.py --all --target 192.168.1.100
```

### 自定义电机参数

编辑 `four_port_udp_sender.py` 中的 `MotorConfig`：

```python
MotorConfig(
    id=4,
    q=1.5,           # 位置（弧度）
    dq=0.5,          # 速度（弧度/秒）
    tau=0.1,         # 力矩（N·m）
    kp=30.0,         # 位置增益
    kd=3.0,          # 速度增益
    mode=1           # 模式
)
```

---

## 📞 技术支持

如有问题，请检查：
1. `/tmp/motor_test10_controller.log` - 控制器日志
2. `/tmp/motor_test10_listener.log` - Listener 日志
3. Controller 调试输出中的 `[DEBUG]` 信息
