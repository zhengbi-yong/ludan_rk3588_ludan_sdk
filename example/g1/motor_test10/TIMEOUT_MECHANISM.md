# 超时检测机制说明

## 概述

Controller 现在具备自动超时检测功能，当 UDP 数据停止发送时，会自动停止向电机发送控制命令。

---

## 超时检测原理

### 超时阈值

```cpp
// motor_controller_with_enable.cpp:1918
const double COMMAND_TIMEOUT_SECONDS = 0.5;  // 500ms 超时
```

**含义**：如果超过 500ms 没有收到新的电机命令数据，该电机将被标记为非活跃状态，停止发送。

### 检测逻辑

```cpp
// motor_controller_with_enable.cpp:1931-1944
double last_command_time = std::chrono::duration<double>(history.current_timestamp.time_since_epoch()).count();
double time_since_last_command = current_timestamp_double - last_command_time;

if (time_since_last_command > COMMAND_TIMEOUT_SECONDS) {
    // 超时：取消该电机的活跃状态
    std::cout << "[TIMEOUT] Motor " << motor_id
              << " 超时 (" << time_since_last_command << "s > "
              << COMMAND_TIMEOUT_SECONDS << "s), 停止发送" << std::endl;
    active_motors_[motor_id] = false;  // 取消活跃状态
    continue;  // 跳过该电机
}
```

---

## 行为说明

### 时间线

```
时间线:
─────────────────────────────────────────────────────────────────>
│
│  UDP 发送阶段 (10秒, 50Hz)
│  ├─ 0s:  [q=0.5, q=0.3, ...] ← Motor 1-6 活跃
│  ├─ 1s:  [q=0.6, q=0.4, ...] ← 继续发送
│  ├─ 2s:  [q=0.7, q=0.5, ...] ← 继续发送
│  └─ ... (500 条消息)
│
│  UDP 停止发送 ─────────────────────────────┐
│                                          │
│  Controller 继续阶段 (最多 500ms)       │
│  ├─ 使用最后的命令值                     │
│  ├─ 500Hz 插值发送                       │
│  └─ 等待新数据                           │
│                                          │
│  500ms 超时 ─────────────────────────────┤
│                                          │
│  自动停止发送                          ──┘
│  ├─ [TIMEOUT] Motor 1 超时, 停止发送    │
│  ├─ [TIMEOUT] Motor 2 超时, 停止发送    │
│  ├─ [TIMEOUT] Motor 3 超时, 停止发送    │
│  └─ ... 所有电机停止                     │
│
─────────────────────────────────────────────────────────────────>
```

### 关键特性

1. **自动检测**：无需手动发送停止命令
2. **快速响应**：500ms 后自动停止
3. **逐个电机**：每个电机独立检测超时
4. **日志输出**：显示超时的电机 ID

---

## 日志示例

### UDP 发送期间

```
[DEBUG] 收到 6 个电机命令: ID1 ID2 ID3 ID4 ID5 ID6
[DEBUG] 端口分配: 8000=3 8001=1 8002=1 8003=1 (总计=6)
[STAT] ROS2_msg: 500 (+50/s) | Send: 15000 (+1500/s)
```

### UDP 停止后（500ms 内）

```
[STAT] ROS2_msg: 503 (+0/s)   | Send: 15300 (+1500/s)  ← 继续发送一小段时间
[DEBUG] 端口分配: 8000=3 8001=1 8002=1 8003=1 (总计=6)
```

### 超时触发

```
[TIMEOUT] Motor 1 超时 (0.512s > 0.5s), 停止发送
[TIMEOUT] Motor 2 超时 (0.513s > 0.5s), 停止发送
[TIMEOUT] Motor 3 超时 (0.514s > 0.5s), 停止发送
[TIMEOUT] Motor 4 超时 (0.515s > 0.5s), 停止发送
[TIMEOUT] Motor 5 超时 (0.516s > 0.5s), 停止发送
[TIMEOUT] Motor 6 超时 (0.517s > 0.5s), 停止发送
```

### 完全停止后

```
[DEBUG] 端口分配: 8000=0 8001=0 8002=0 8003=0 (总计=0)  ← 没有电机发送
[STAT] ROS2_msg: 503 (+0/s)   | Send: 15300 (+0/s)      ← Send 不再增加
[SYS] Chnl 0: TxCAN: 0(S: 0)   ← 不再发送
[SYS] Chnl 1: TxCAN: 0(S: 0)   ← 不再发送
[SYS] Chnl 2: TxCAN: 0(S: 0)   ← 不再发送
[SYS] Chnl 3: TxCAN: 0(S: 0)   ← 不再发送
```

---

## 测试方法

### 自动化测试（推荐）

```bash
cd /home/linaro/ludan_sdk/example/g1/motor_test10
./test_all_ports.sh
```

脚本会自动：
1. 启动 Controller
2. 启动 Listener
3. 发送测试数据
4. 等待超时触发
5. 显示结果

### 手动测试

```bash
# 1. 启动 Controller
/home/linaro/ludan_sdk/build/bin/motor_controller_with_enable \
    --enable-motor-cmd --auto-start

# 2. 启动 Listener
cd /home/linaro/ludan_sdk/scripts
python3 listener_8888_to_ros2.py 8888

# 3. 发送数据（持续 10 秒）
cd /home/linaro/ludan_sdk/example/g1/motor_test10
python3 four_port_udp_sender.py --continuous 10

# 4. 观察超时日志（发送结束后 500ms）
tail -f /tmp/motor_test10_controller.log | grep TIMEOUT
```

---

## 配置超时时间

### 修改超时阈值

编辑 `motor_controller_with_enable.cpp:1918`:

```cpp
// 更短的超时时间（更快响应）
const double COMMAND_TIMEOUT_SECONDS = 0.2;  // 200ms

// 更长的超时时间（更宽松）
const double COMMAND_TIMEOUT_SECONDS = 1.0;  // 1000ms
```

### 重新编译

```bash
cd /home/linaro/ludan_sdk/example/g1/low_level/build/motor_controller_with_enable
make -j4
cp motor_controller_with_enable /home/linaro/ludan_sdk/build/bin/
```

---

## 与旧版本的区别

### 旧版本（无超时检测）

| 行为 | 说明 |
|------|------|
| UDP 停止后 | 继续发送最后的命令值 |
| 停止方式 | 必须手动发送 `mode=0` 命令 |
| 问题 | 如果忘记发送停止命令，会一直发送 |

### 新版本（有超时检测）

| 行为 | 说明 |
|------|------|
| UDP 停止后 | 继续发送最多 500ms |
| 停止方式 | **自动停止**（无需手动命令） |
| 优势 | 无需担心忘记停止 |

---

## 故障排查

### Q: 超时检测不工作？

**检查清单**：
1. 确认使用的是最新编译的版本
2. 检查日志中是否有 `[TIMEOUT]` 消息
3. 验证 UDP 数据确实停止发送了

```bash
# 查看编译时间
ls -l /home/linaro/ludan_sdk/build/bin/motor_controller_with_enable

# 查看超时日志
grep TIMEOUT /tmp/motor_test10_controller.log
```

### Q: 超时时间太短/太长？

**A**: 根据应用场景调整：
- 高频控制（>100Hz）：200ms 超时
- 普通控制（50Hz）：500ms 超时（默认）
- 低频控制（<20Hz）：1000ms 超时

### Q: 想手动停止电机？

**A**: 仍然可以使用 `stop_motors.py`：

```bash
cd /home/linaro/ludan_sdk/example/g1/motor_test10
python3 stop_motors.py
```

这会立即发送 `mode=0` 命令，无需等待超时。

---

## 总结

| 特性 | 旧版本 | 新版本 |
|------|--------|--------|
| 超时检测 | ❌ 无 | ✅ 有 |
| 自动停止 | ❌ 需手动 | ✅ 自动 |
| 停止时间 | 取决于手动操作 | 500ms |
| 安全性 | 可能忘记停止 | 自动停止，更安全 |
