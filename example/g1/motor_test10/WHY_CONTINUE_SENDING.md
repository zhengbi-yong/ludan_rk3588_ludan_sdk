# 为什么 Controller 持续发送数据？

> **⚠️ 注意**：本文档描述的是**旧版本**的行为。
>
> **新版本已更新**：Controller 现在具备**自动超时检测**功能，UDP 停止发送后 500ms 会自动停止。
>
> 请参考：[TIMEOUT_MECHANISM.md](TIMEOUT_MECHANISM.md)

---

## 问题现象

运行 `python3 four_port_udp_sender.py --continuous 10` 发送结束后，Controller 仍在发送数据。

---

## 这是正常行为！

### 原因：500Hz 插值发送机制

```
时间线:
─────────────────────────────────────────────────────────────────>
│
│  UDP 发送阶段 (10秒)
│  ├─ 0s:  [q=0.5, q=0.3, ...]
│  ├─ 1s:  [q=0.6, q=0.4, ...]
│  ├─ 2s:  [q=0.7, q=0.5, ...]
│  └─ ... (500 条消息)
│
│  UDP 停止 ─────────────────────────────────┐
│                                        │
│                                        │
│  Controller 继续发送阶段              │
│  ├─ 使用最后的命令值                   │
│  ├─ 500Hz 插值发送                     │
│  └─ 保持电机位置                       │
│
│  收到停止命令 (mode=0) ─────────────────┤
│                                        │
│  停止发送                           ──┘
│
─────────────────────────────────────────────────────────────────>
```

---

## 为什么这样设计？

### 1. 保持电机位置

DM 电机需要**持续接收命令**来保持位置：

```cpp
// motor_controller_with_enable.cpp:1914-1919
MotorCommandCan interpolateCommand(prev, curr, ...) {
    // 即使没有新数据，也使用 curr 的值
    // 这确保电机保持在上一个命令的位置
    return curr;  // ← 关键：返回最后一个命令
}
```

### 2. 活跃电机状态持久化

```cpp
// motor_controller_with_enable.cpp:1750-1752
if (motor_cmd.mode != 0) {
    active_motors_[motor_id] = true;  // ← 一旦设置，永不重置
}
```

**`active_motors_` 数组特性**：
- ✅ 设置为 `true` 后，不会自动重置
- ✅ 即使 UDP 停止发送，电机仍然保持活跃
- ✅ 只有显式发送 `mode=0` 才能取消

### 3. 防止电机突然失能

如果控制器突然停止发送：
- ❌ 电机可能进入失能状态
- ❌ 位置可能丢失
- ❌ 机器人可能跌倒

**持续发送**确保：
- ✅ 电机保持使能状态
- ✅ 位置得以保持
- ✅ 机器人安全

---

## 如何停止发送？

### 方法 1：发送停止命令（推荐）

```bash
cd /home/linaro/ludan_sdk/example/g1/motor_test10
python3 stop_motors.py
```

**原理**：
```python
# 发送 mode=0 的消息
motor_cmd[str(i)] = {
    "q": 0.0,
    "dq": 0.0,
    "tau": 0.0,
    "mode": 0,  # ← 关键：设置为非活跃
    "kp": 0.0,
    "kd": 0.0
}
```

**效果**：
```cpp
// Controller 收到 mode=0
if (motor_cmd.mode != 0) {
    active_motors_[motor_id] = true;  // ← 不执行
}
// active_motors_[motor_id] 保持 false

// CollectMotorCommands 跳过
if (!active_motors_[motor_id]) {
    continue;  // ← 不收集该电机
}

// 端口分配为空
port_commands[8000] = []  // ← 不发送
```

### 方法 2：停止 Controller 进程

```bash
# 找到进程并停止
ps aux | grep motor_controller
kill <PID>

# 或者使用 pkill
pkill -f motor_controller
```

### 方法 3：Ctrl+C

如果 Controller 在前台运行，直接按 `Ctrl+C`。

---

## 验证停止成功

### Controller 日志显示

**停止前**：
```
[STAT] ROS2_msg: 500 (+50/s) | Send: 15000 (+1500/s)
[DEBUG] 端口分配: 8000=3 8001=1 8002=1 8003=1 (总计=6)
```

**停止后**：
```
[STAT] ROS2_msg: 503 (+0/s)   | Send: 0 (+0/s)      ← Send 不再增加
[DEBUG] 端口分配: 8000=0 8001=0 8002=0 8003=0 (总计=0)
```

### CAN 统计

```bash
# 查看 Controller 日志的最后几行
tail -20 /tmp/motor_test10_controller.log

# 应该看到 TxCAN 不再增加
[SYS] Chnl 0: TxCAN: 0(S: 0)   ← 不再发送
[SYS] Chnl 1: TxCAN: 0(S: 0)   ← 不再发送
[SYS] Chnl 2: TxCAN: 0(S: 0)   ← 不再发送
[SYS] Chnl 3: TxCAN: 0(S: 0)   ← 不再发送
```

---

## 完整测试流程

### 推荐流程

```bash
# 1. 启动 Controller
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable \
    --enable-motor-cmd --auto-start

# 2. 启动 Listener
cd /home/linaro/ludan_sdk/scripts
python3 listener_8888_to_ros2.py 8888

# 3. 发送测试数据
cd /home/linaro/ludan_sdk/example/g1/motor_test10
python3 four_port_udp_sender.py --continuous 10

# 4. 停止发送（重要！）
python3 stop_motors.py

# 5. 验证停止
# Controller 应该不再发送数据
```

---

## 常见问题

### Q: 为什么不设计成自动停止？

**A**: 机器人控制的安全性考虑：
- 保持位置需要持续发送
- 防止意外失能导致机器人跌倒
- 符合工业控制标准

### Q: stop_motors.py 不起作用？

**A**: 检查以下几点：
1. Listener 是否还在运行？
2. UDP 数据是否成功发送到 8888 端口？
3. Controller 是否收到了 `/lowcmd` 消息？

### Q: 如何让某个端口停止发送？

**A**: 只需要将该端口的电机设置为 `mode=0`：

```python
# 只停止端口 8000 (电机 1, 2, 3)
motor_cmd = {
    "1": {"mode": 0, "q": 0, ...},
    "2": {"mode": 0, "q": 0, ...},
    "3": {"mode": 0, "q": 0, ...},
    "4": {"mode": 1, "q": 0.5, ...},  # 其他端口保持活跃
    ...
}
```

---

## 总结

| 行为 | 原因 | 解决方法 |
|------|------|----------|
| Controller 持续发送 | 保持电机位置 | 这是正常的 |
| UDP 停止后仍发送 | 活跃状态持久化 | 发送 `mode=0` 命令 |
| 无法自动停止 | 安全设计 | 使用 `stop_motors.py` |
| 想立即停止 | - | `pkill motor_controller` |
