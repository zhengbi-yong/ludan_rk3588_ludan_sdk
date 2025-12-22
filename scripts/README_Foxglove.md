# Foxglove ROS2 Bridge Daemon

基于 `fox.sh` 改造的 Foxglove ROS2 WebSocket Bridge 守护进程服务。

## 功能特性

- ✅ **后台守护进程模式** - 独立运行，不依赖终端
- ✅ **PID 文件管理** - 自动管理进程ID和生命周期
- ✅ **端口8765** - 默认端口，自动检测冲突并使用备用端口
- ✅ **日志管理** - 详细日志记录和错误追踪
- ✅ **完整的管理命令** - start/stop/restart/status/logs/topics
- ✅ **ROS2 集成** - 自动检测和发布ROS2 topics
- ✅ **状态监控** - 实时查看连接状态和活动连接数

## 快速开始

### 方法1: 使用管理脚本（推荐）
```bash
# 启动管理界面
./scripts/manage_foxglove.sh

# 或直接命令行操作
./scripts/foxglove_ros2.sh start    # 启动
./scripts/foxglove_ros2.sh stop     # 停止
./scripts/foxglove_ros2.sh status   # 查看状态
./scripts/foxglove_ros2.sh restart  # 重启
./scripts/foxglove_ros2.sh logs     # 查看日志
./scripts/foxglove_ros2.sh topics   # 列出topics
```

### 方法2: 直接命令行
```bash
# 启动守护进程
./scripts/foxglove_ros2.sh start

# 后台运行并立即查看状态
./scripts/foxglove_ros2.sh start && ./scripts/foxglove_ros2.sh status
```

## 配置选项

### 环境变量
```bash
export FOXGLOVE_PORT=8765      # WebSocket端口
export FOXGLOVE_HOST=0.0.0.0   # 绑定地址
export LOG_LEVEL=info           # 日志级别
export ROS_DOMAIN_ID=0         # ROS2域ID
```

### 默认配置
- **端口**: 8765 (8766-8775 备用)
- **地址**: 0.0.0.0 (所有接口)
- **速率限制**: 100 msg/s
- **日志**: `~/.foxglove/logs/`
- **PID文件**: `/tmp/foxglove_ros2.pid`

## 命令参考

### 基本命令
```bash
./scripts/foxglove_ros2.sh start      # 启动守护进程
./scripts/foxglove_ros2.sh stop       # 停止守护进程
./scripts/foxglove_ros2.sh restart    # 重启守护进程
./scripts/foxglove_ros2.sh status     # 查看状态
./scripts/foxglove_ros2.sh logs       # 查看日志(最后50行)
./scripts/foxglove_ros2.sh topics     # 列出可用ROS2 topics
./scripts/foxglove_ros2.sh help       # 显示帮助
```

### 状态信息
守护进程运行时会显示：
- ✅ PID (进程ID)
- ✅ WebSocket URL (ws://0.0.0.0:8765)
- ✅ 活动连接数
- ✅ 日志文件位置

## Foxglove Studio 连接

1. 启动守护进程：
```bash
./scripts/foxglove_ros2.sh start
```

2. 在 Foxglove Studio 中添加连接：
   - URL: `ws://<您的IP地址>:8765`
   - 或 `ws://localhost:8765` (本地连接)

3. 连接后即可查看所有 ROS2 topics 的实时数据

## 故障排除

### 常见问题

**端口被占用**
```
✓ Found Foxglove Bridge package: foxglove_bridge
WARN Port 8765 is already in use. Trying to find another port...
INFO Using alternative port: 8766
```
*解决: 自动使用备用端口 8766-8775*

**ROS2环境未找到**
```
ERROR Could not find ROS2 installation. Please source your ROS2 environment.
```
*解决: 手动执行 `source /opt/ros/humble/setup.bash`*

**包未安装**
```
ERROR Foxglove Bridge package not found
sudo apt install ros-humble-foxglove-bridge
```

### 日志查看
```bash
# 实时日志
tail -f ~/.foxglove/logs/foxglove_ros2_*.log

# 查看最近的日志
./scripts/foxglove_ros2.sh logs

# 查看完整日志文件
cat ~/.foxglove/logs/foxglove_ros2_*.log
```

### 进程管理
```bash
# 查看所有Foxglove相关进程
ps aux | grep foxglove

# 强制停止所有进程
pkill -f foxglove

# 检查端口占用
netstat -tlnp | grep :8765
```

## 系统集成

### 开机自启动（可选）
添加到 crontab：
```bash
crontab -e
@reboot /bin/bash /path/to/unitree_sdk2/scripts/foxglove_ros2.sh start
```

### systemd 服务（可选）
创建 systemd 服务文件 `/etc/systemd/system/foxglove-ros2.service`

## 技术细节

### 守护进程特性
- **双fork分离**: 完全脱离终端
- **PID文件管理**: 防止重复启动
- **信号处理**: 优雅停止和重启
- **日志重定向**: 标准输出和错误都记录到文件

### 安全性
- **权限检查**: 确保有足够的权限启动服务
- **端口验证**: 自动检测端口冲突
- **进程监控**: 定期检查进程健康状态

### 性能优化
- **后台运行**: 不占用终端资源
- **异步日志**: 日志写入不阻塞主进程
- **智能重试**: 自动处理临时性故障

## 更新日志

### v1.0 (当前版本)
- 基于原始 fox.sh 改造
- 添加守护进程模式
- 实现完整的管理命令集
- 增强错误处理和日志记录
- 添加状态监控和连接数显示

---

**🚀 现在您就有了一个完整的 Foxglove ROS2 后台服务！**