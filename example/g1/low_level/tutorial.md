# ZLG CANFDNET 电机控制教程

本教程介绍如何使用 ZLG CANFDNET 设备进行电机通信和控制。

## 目录

1. [环境准备](#环境准备)
2. [SDK 编译](#sdk-编译)
3. [项目配置](#项目配置)
4. [电机使能测试](#电机使能测试)
5. [稳定性测试](#稳定性测试)
6. [长时间测试](#长时间测试)
7. [电机反馈解码](#电机反馈解码)

---

## 环境准备

### 硬件设备

- ZLG CANFDNET-400U 设备
- 电机（支持 CAN 通信）
- 网络连接（无线或有线）

### 网络配置

确保 ZLG 设备的 IP 地址可访问。本教程中使用：
- 设备 IP: `192.168.1.5`
- 端口: `8002` (对应 CAN2 通道)
- CAN 通道: CAN2

**重要提示**: 如果系统有多个网络接口（如 eth0 和 wlan0），确保路由配置正确。错误的 IP 配置会导致 CAN 总线通信失败。

```bash
# 检查网络配置
ip addr show

# 如果 eth0 未使用，不要给它分配与目标网段相同的 IP
# 让 wlan0 通过正确的网关路由到 192.168.1.5
```

---

## SDK 编译

### ZLG CANFDNET SDK 位置

SDK 路径: `/home/linaro/ludan_sdk/LINUX_CANFDNET_V1.3.0.5_example`

### 编译步骤

```bash
cd /home/linaro/ludan_sdk/LINUX_CANFDNET_V1.3.0.5_example
mkdir -p build && cd build
cmake ..
make
```

编译成功后，库文件位于：
```
lib/linux_x64/libCANFDNET.so
```

---

## 项目配置

### CMakeLists.txt 配置

在项目的 `CMakeLists.txt` 中添加 ZLG SDK 支持：

```cmake
# 设置 ZLG SDK 路径
set(ZLG_SDK_DIR "/home/linaro/ludan_sdk/LINUX_CANFDNET_V1.3.0.5_example")

# 添加可执行文件
add_executable(zlg_test low_level/zlg_test.cpp)

# 设置包含目录
target_include_directories(zlg_test PRIVATE
    "${ZLG_SDK_DIR}/src"
    "${ZLG_SDK_DIR}/src/zlgcan"
)

# 设置 RPATH 用于运行时库搜索
set_target_properties(zlg_test PROPERTIES
    INSTALL_RPATH "${ZLG_SDK_DIR}/lib/linux_x64"
    BUILD_WITH_INSTALL_RPATH TRUE
)

# 链接库
target_link_libraries(zlg_test
    "${ZLG_SDK_DIR}/lib/linux_x64/libCANFDNET.so"
    pthread
)
```

### 头文件引用

```cpp
#include "CANFDNET.h"
#include "zlgcan/canframe.h"
#include "zlgcan/zlgcan.h"
```

---

## 电机使能测试

### 编译电机控制器

```bash
cd /home/linaro/ludan_sdk/build
cmake ..
make motor_controller_with_enable
```

### 使能单个电机

```bash
./bin/motor_controller_with_enable 192.168.1.5:8002 --enable 1
```

### 使能多个电机

```bash
# 使能电机 1-5
./bin/motor_controller_with_enable 192.168.1.5:8002 --enable 1-5

# 使能电机 1-30
./bin/motor_controller_with_enable 192.168.1.5:8002 --enable 1-30
```

### 失能电机

```bash
./bin/motor_controller_with_enable 192.168.1.5:8002 --disable 1-30
```

### 预期输出

```
========================================
    Motor Enable/Disable Test
========================================
Target: 192.168.1.5:8002
Channel: CAN2
========================================
[1/4] Device opened
[2/4] CAN channel initialized (CAN-FD 1M/5M)
[3/4] CAN started
[4/4] Buffer cleared, ready!
=====================================

Enabling motors 1 to 30...
Motor 1: ENABLE command sent - Success (TX_Err: 16, RX_Err: 0)
Motor 2: ENABLE command sent - Success (TX_Err: 16, RX_Err: 0)
...
=====================================

Summary:
Successfully enabled: 30/30 motors
Failed: 0 motors
=====================================
```

---

## 稳定性测试

### 编译稳定性测试程序

```bash
make zlg_stability_test
```

### 运行测试

```bash
./bin/zlg_stability_test 192.168.1.5:8002
```

### 测试内容

1. **单帧测试** - 测试不同延迟下的发送频率
   - 1000μs 延迟
   - 100μs 延迟
   - 10μs 延迟
   - 无延迟

2. **突发模式测试** - 连续发送 1000 帧

3. **10 秒稳定性测试** - 持续发送 10 秒

### 预期结果

```
========================================
    ZLG CAN Stability Test
========================================

=== Test 1: Single Frame with 1000us delay ===
Frames sent: 100, Success: 100, Failed: 0
Success rate: 100.00%

=== Test 2: Single Frame with 100us delay ===
Frames sent: 100, Success: 100, Failed: 0
Success rate: 100.00%

=== Test 3: Single Frame with 10us delay ===
Frames sent: 100, Success: 100, Failed: 0
Success rate: 100.00%

=== Test 4: Single Frame with no delay ===
Frames sent: 100, Success: 100, Failed: 0
Success rate: 100.00%

=== Test 5: Burst mode (1000 frames) ===
Peak TX rate: 421453 fps
Frames sent: 1000, Success: 1000, Failed: 0
Success rate: 100.00%

=== Test 6: 10-second stability test ===
Duration: 10.00 seconds
Total sent: 49512 frames
Total recv: 51234 frames
Avg TX rate: 4951.20 fps
Avg RX rate: 5123.40 fps
TX_Error: 0, RX_Error: 128
```

---

## 长时间测试

### 编译 5 分钟测试程序

```bash
make zlg_5min_test
```

### 运行测试

```bash
./bin/zlg_5min_test 192.168.1.5:8002
```

### 测试内容

- 持续运行 300 秒（5 分钟）
- 每秒发送约 4800 帧使能命令
- 接收电机反馈帧
- 每秒保存一帧数据到日志文件

### 输出文件

测试结果保存在: `motor_test_log.txt`

### 预期结果

```
========================================
    ZLG 5-Minute Extended Test
========================================
Duration: 300 seconds (5 min)
=====================================

=== Starting 300 Second Extended Test ===

   Time    |  Sent/s  | Recv/s |  Sent Tot | Recv Tot | TX_Err | RX_Err | Saved
==================================================================================
[    1.0s ] |    4963 |    2737 |      4963 |      2737 |      0 |    127 |     1
[    2.0s ] |    4740 |    2592 |      9703 |      5329 |      0 |    134 |     2
...
[  300.0s ] |    4850 |    2489 |  1444600 |   741260 |      0 |    134 |   300
==================================================================================

=== Final Test Results ===
Test Duration: 300.00 seconds
----------------------------------------
Total Sent:       1,444,600 frames
Total Recv:         741,260 frames
Total TX Errors:          0 frames
----------------------------------------
Avg TX Rate:       4815.33 fps
Avg RX Rate:       2470.87 fps
----------------------------------------
Final TX_Error:             0
Final RX_Error:           134
----------------------------------------
Frames Saved:             300 frames
=====================================
```

---

## 电机反馈解码

### 电机反馈帧格式

根据电机通信协议，反馈帧格式如下：

| 字节 | 位 | 含义 |
|------|----|----|
| Data[0] | 7-0 | 模式 (Mode) |
| Data[1] | 7 | 转矩符号位 |
| Data[1] | 6-0 | 转矩值 (Torque) |
| Data[2] | 7 | 速度符号位 |
| Data[2] | 6-0 | 速度值 (Speed) |
| Data[3] | 7-0 | 位置 (Position) |
| Data[4] | 7-4 | 温度 (Temperature) |
| Data[4] | 3-0 | 错误码 (Error) |
| Data[5] | 6-0 | 状态 (Status) |
| Data[6-7] | - | 保留 |

### 编译解码测试程序

```bash
make zlg_5min_decode_test
```

### 运行测试

```bash
./bin/zlg_5min_decode_test 192.168.1.5:8002
```

### 输出文件

解码后的数据保存在: `motor_test_frame_decode.txt`

### 解码输出格式

```
#  0 | Time:     1000 ms | ID:   0 | Mode:  17 | Torque:  -15 | Speed:  -89 | Pos: 127 | Temp:  15 | Err:  7 | Status: 0x7d
```

### 字段说明

- **#**: 序号
- **Time**: 时间戳（毫秒）
- **ID**: 电机 CAN ID
- **Mode**: 工作模式
- **Torque**: 转矩（有符号）
- **Speed**: 速度（有符号）
- **Pos**: 位置
- **Temp**: 温度
- **Err**: 错误码
- **Status**: 状态值

---

## 常见问题

### 1. CAN 总线错误 - TX Error Counter 过高

**症状**: 发送时 ZLG 设备 CAN 通道红灯闪烁，TX Error Counter 达到 128。

**原因**: CAN 总线通信失败，可能的原因：
- 网络路由配置错误
- CAN 总线终端电阻问题
- 波特率配置不匹配
- 电机未上电或未连接

**解决方案**:
1. 检查网络路由配置
2. 确保 CAN 总线终端电阻正确（120Ω）
3. 验证波特率配置（仲裁段 1Mbps，数据段 5Mbps）
4. 确保电机已上电并正确连接

### 2. "Connection refused" 错误

**症状**: 无法连接到 ZLG 设备的 TCP 端口。

**解决方案**:
1. 检查设备 IP 地址是否正确
2. 确认设备已上电
3. 检查网络连接
4. 尝试 ping 设备 IP

### 3. RX Error Counter 较高

**说明**: RX Error Counter 在 127-135 范围内是正常的，这是电机响应时的正常行为。

### 4. 编译错误 - 找不到 ZCAN 函数

**解决方案**:
1. 确保 SDK 已正确编译
2. 检查 CMakeLists.txt 中的库路径配置
3. 确认 RPATH 设置正确

---

## 代码示例

### 基本 CAN 初始化

```cpp
#include "CANFDNET.h"
#include "zlgcan/zlgcan.h"

// 打开设备
DEVICE_HANDLE device_handle = ZCAN_OpenDevice(ZCAN_CANFDNET_400U_TCP, 0, 0);
if (device_handle == INVALID_DEVICE_HANDLE) {
    std::cerr << "Failed to open device" << std::endl;
    return false;
}

// 配置 CAN-FD
ZCAN_CHANNEL_INIT_CONFIG init_config;
memset(&init_config, 0, sizeof(init_config));
init_config.can_type = TYPE_CANFD;
init_config.canfd.acc_code = 0;
init_config.canfd.acc_mask = 0;
init_config.canfd.abit_timing = 1000000;  // 1Mbps 仲裁
init_config.canfd.dbit_timing = 5000000;  // 5Mbps 数据
init_config.canfd.brp = 0;
init_config.canfd.filter = 0;
init_config.canfd.mode = 0;

CHANNEL_HANDLE channel_handle = ZCAN_InitCAN(device_handle, channel, &init_config);
if (channel_handle == INVALID_CHANNEL_HANDLE) {
    std::cerr << "Failed to initialize CAN channel" << std::endl;
    return false;
}

// 设置 TCP 连接参数
UINT val = 0;  // TCP 客户端模式
ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, channel, CMD_TCP_TYPE, &val);
ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, channel, CMD_DESIP, (void*)ip.c_str());
val = port;
ZCAN_SetReference(ZCAN_CANFDNET_400U_TCP, 0, channel, CMD_DESPORT, &val);

// 启动 CAN
if (ZCAN_StartCAN(channel_handle) != STATUS_OK) {
    std::cerr << "Failed to start CAN" << std::endl;
    return false;
}
```

### 发送使能命令

```cpp
// 准备使能命令帧
ZCAN_Transmit_Data cmd_frame;
memset(&cmd_frame, 0, sizeof(cmd_frame));
cmd_frame.frame.can_id = motor_id;  // 电机 ID
cmd_frame.frame.can_dlc = 8;
memset(cmd_frame.frame.data, 0xFF, 7);
cmd_frame.frame.data[7] = 0xFC;  // 使能命令

// 发送命令
if (ZCAN_Transmit(channel_handle, &cmd_frame, 1) == 1) {
    std::cout << "Motor " << motor_id << " enabled successfully" << std::endl;
} else {
    std::cerr << "Failed to enable motor " << motor_id << std::endl;
}
```

### 接收反馈帧

```cpp
// 接收 CAN 标准帧
ZCAN_Receive_Data recv_frames[100];
UINT recv_count = ZCAN_Receive(channel_handle, recv_frames, 100, 0);

// 接收 CAN-FD 帧
ZCAN_ReceiveFD_Data recv_fd_frames[50];
UINT recv_fd_count = ZCAN_ReceiveFD(channel_handle, recv_fd_frames, 50, 0);

// 处理接收到的帧
for (UINT i = 0; i < recv_count; i++) {
    uint32_t can_id = recv_frames[i].frame.can_id & 0x7FF;
    uint8_t dlc = recv_frames[i].frame.can_dlc;
    uint8_t* data = recv_frames[i].frame.data;

    // 解码数据...
}
```

### 读取 CAN 错误计数器

```cpp
ZCAN_CHANNEL_STATUS status;
if (ZCAN_ReadChannelStatus(channel_handle, &status) == STATUS_OK) {
    BYTE tx_err = status.regTECounter;
    BYTE rx_err = status.regRECounter;
    std::cout << "TX Error: " << (int)tx_err << ", RX Error: " << (int)rx_err << std::endl;
}
```

### 清理资源

```cpp
if (channel_handle != INVALID_CHANNEL_HANDLE) {
    ZCAN_ResetCAN(channel_handle);
}
if (device_handle != INVALID_DEVICE_HANDLE) {
    ZCAN_CloseDevice(device_handle);
}
```

---

## 附录

### 可用的测试程序

| 程序名 | 功能 |
|--------|----|
| `motor_controller_with_enable` | 电机使能/失能控制 |
| `zlg_stability_test` | 通信稳定性和极限帧率测试 |
| `zlg_5min_test` | 5 分钟长时间测试 |
| `zlg_5min_decode_test` | 5 分钟测试 + 电机反馈解码 |

### API 版本说明

新版本 ZLG SDK 使用 `ZCAN_SetReference` 替代了旧版的 `ZCAN_SetValue`。

| 旧 API | 新 API |
|--------|--------|
| `ZCAN_SetValue(...)` | `ZCAN_SetReference(...)` |

---

## 参考资料

- ZLG CANFDNET SDK 文档
- CAN 通信协议规范
- Unitree SDK2 文档

---

**最后更新**: 2024-12-24
