#!/bin/bash
# Jetson LowCmd Publisher Launcher
# 在Jetson上启动/lowcmd话题发布器

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 默认参数
RK3588_IP="${RK3588_IP:-192.168.1.20}"
RK3588_PORT="${RK3588_PORT:-8888}"
PUBLISH_RATE="${PUBLISH_RATE:-50}"
SINE_AMPLITUDE="${SINE_AMPLITUDE:-0.3}"
SINE_FREQUENCY="${SINE_FREQUENCY:-0.5}"
USE_NETWORK="${USE_NETWORK:-true}"
USE_DDS_BRIDGE="${USE_DDS_BRIDGE:-true}"
TARGET_JOINTS="${TARGET_JOINTS:-[4,5,10,11]}"

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 显示配置
show_config() {
    echo "=================================================="
    echo "    Jetson LowCmd Publisher Configuration      "
    echo "=================================================="
    echo "Target RK3588:      $RK3588_IP:$RK3588_PORT"
    echo "Publish Rate:       $PUBLISH_RATE Hz"
    echo "Sine Amplitude:     $SINE_AMPLITUDE rad"
    echo "Sine Frequency:     $SINE_FREQUENCY Hz"
    echo "Target Joints:      $TARGET_JOINTS"
    echo "Network Mode:       $USE_NETWORK"
    echo "DDS Bridge:         $USE_DDS_BRIDGE"
    echo "=================================================="
    echo ""
}

# 检查环境
check_environment() {
    log_info "Checking environment..."

    # 检查ROS1
    if [ -z "$ROS_DISTRO" ]; then
        log_error "ROS1 environment not sourced!"
        log_info "Please run: source /opt/ros/noetic/setup.bash"
        exit 1
    fi
    log_success "ROS1 ($ROS_DISTRO) found"

    # 检查Python3
    if ! command -v python3 &> /dev/null; then
        log_error "Python3 not found"
        exit 1
    fi
    log_success "Python3 found"

    # 检查发布器脚本
    if [ ! -f "$(dirname $0)/jetson_lowcmd_publisher.py" ]; then
        log_error "Publisher script not found!"
        exit 1
    fi
    log_success "Publisher script found"

    # 设置可执行权限
    chmod +x "$(dirname $0)/jetson_lowcmd_publisher.py"
}

# 网络连通性测试
test_network() {
    if [ "$USE_NETWORK" = "true" ]; then
        log_info "Testing network connectivity to RK3588 ($RK3588_IP)..."

        if ping -c 2 "$RK3588_IP" &> /dev/null; then
            log_success "✅ Network connectivity to RK3588 OK"
        else
            log_warning "⚠️ Cannot reach RK3588 at $RK3588_IP"
            log_info "Network mode will be disabled"
            USE_NETWORK="false"
        fi
    fi
}

# 启动ROS核心
start_ros_core() {
    log_info "Checking ROS Core..."

    # 检查roscore是否在运行
    if pgrep -f "roscore" > /dev/null; then
        log_info "✅ ROS Core is already running"
    else
        log_info "Starting ROS Core..."
        roscore &
        ROSCORE_PID=$!
        sleep 3

        if pgrep -f "roscore" > /dev/null; then
            log_success "✅ ROS Core started (PID: $ROSCORE_PID)"
            echo $ROSCORE_PID > /tmp/roscore.pid
        else
            log_error "❌ Failed to start ROS Core"
            exit 1
        fi
    fi
}

# 启动话题监控
start_topic_monitor() {
    log_info "Starting topic monitor..."

    # 在后台启动话题列表监控
    (
        while true; do
            sleep 5
            if [ -n "$(rostopic list | grep '/lowcmd')" ]; then
                log_success "✅ /lowcmd topic is being published"
                break
            fi
        done
    ) &
    MONITOR_PID=$!

    echo $MONITOR_PID > /tmp/topic_monitor.pid
}

# 启动发布器
start_publisher() {
    log_info "Starting Jetson LowCmd Publisher..."

    # 准备启动参数
    PUBLISHER_ARGS=" \
        --rk3588_ip:=$RK3588_IP \
        --rk3588_port:=$RK3588_PORT \
        --publish_rate:=$PUBLISH_RATE \
        --sine_amplitude:=$SINE_AMPLITUDE \
        --sine_frequency:=$SINE_FREQUENCY \
        --target_joints:=\"$TARGET_JOINTS\" \
        --use_network:=$USE_NETWORK \
        --use_dds_bridge:=$USE_DDS_BRIDGE"

    # 启动发布器
    log_info "Publisher arguments: $PUBLISHER_ARGS"

    python3 "$(dirname $0)/jetson_lowcmd_publisher.py" $PUBLISHER_ARGS &
    PUBLISHER_PID=$!

    sleep 2

    if kill -0 $PUBLISHER_PID 2>/dev/null; then
        log_success "✅ Publisher started successfully (PID: $PUBLISHER_PID)"
        echo $PUBLISHER_PID > /tmp/publisher.pid
    else
        log_error "❌ Failed to start publisher"
        exit 1
    fi
}

# 显示状态信息
show_status() {
    sleep 3  # 等待发布器完全启动

    log_info "System Status:"
    echo "------------------------------------------------"

    # 显示ROS话题
    log_info "ROS Topics:"
    if command -v rostopic &> /dev/null; then
        rostopic list 2>/dev/null | grep -E "(lowcmd|joint)" || log_warning "No lowcmd/joint topics found"
    fi

    echo ""

    # 显示ROS节点
    log_info "ROS Nodes:"
    if command -v rosnode &> /dev/null; then
        rosnode list 2>/dev/null | grep "publisher" || log_warning "No publisher nodes found"
    fi

    echo ""

    # 显示话题频率
    log_info "Topic Frequencies:"
    if command -v rostopic &> /dev/null && rostopic list | grep -q "/lowcmd"; then
        timeout 10s rostopic hz /lowcmd 2>/dev/null | head -5 || log_warning "Cannot measure /lowcmd frequency"
    else
        log_warning "/lowcmd topic not found for frequency measurement"
    fi

    echo "------------------------------------------------"
    echo ""
    echo "🎯 Publisher is running!"
    echo "📡 Publishing to topic: /lowcmd"
    echo "🌐 Sending to: $RK3588_IP:$RK3588_PORT"
    echo "📄 Bridge file: /tmp/lowcmd_data.json"
    echo ""
    echo "🔍 To monitor topics:"
    echo "   rostopic echo /lowcmd"
    echo "   rostopic echo /joint_states"
    echo "   rostopic hz /lowcmd"
    echo ""
    echo "⏹️  Press Ctrl+C to stop"
}

# 清理函数
cleanup() {
    log_info "Cleaning up..."

    # 停止发布器
    if [ -f /tmp/publisher.pid ]; then
        PUBLISHER_PID=$(cat /tmp/publisher.pid)
        if kill -0 $PUBLISHER_PID 2>/dev/null; then
            kill $PUBLISHER_PID
            log_success "Publisher stopped"
        fi
        rm -f /tmp/publisher.pid
    fi

    # 停止话题监控
    if [ -f /tmp/topic_monitor.pid ]; then
        MONITOR_PID=$(cat /tmp/topic_monitor.pid)
        if kill -0 $MONITOR_PID 2>/dev/null; then
            kill $MONITOR_PID 2>/dev/null || true
        fi
        rm -f /tmp/topic_monitor.pid
    fi

    # 停止ROS Core（如果是我们启动的）
    if [ -f /tmp/roscore.pid ]; then
        ROSCORE_PID=$(cat /tmp/roscore.pid)
        if kill -0 $ROSCORE_PID 2>/dev/null; then
            kill $ROSCORE_PID 2>/dev/null || true
            log_success "ROS Core stopped"
        fi
        rm -f /tmp/roscore.pid
    fi

    # 清理桥接文件
    rm -f /tmp/lowcmd_data.json

    log_success "Cleanup completed"
    exit 0
}

# 主函数
main() {
    show_config

    # 设置信号处理
    trap cleanup SIGINT SIGTERM

    check_environment
    test_network
    start_ros_core
    start_topic_monitor
    start_publisher
    show_status

    # 等待用户中断
    while true; do
        sleep 1
    done
}

# 命令行参数处理
while [[ $# -gt 0 ]]; do
    case $1 in
        --rk3588_ip)
            RK3588_IP="$2"
            shift 2
            ;;
        --rk3588_port)
            RK3588_PORT="$2"
            shift 2
            ;;
        --rate|--publish_rate)
            PUBLISH_RATE="$2"
            shift 2
            ;;
        --amplitude)
            SINE_AMPLITUDE="$2"
            shift 2
            ;;
        --frequency)
            SINE_FREQUENCY="$2"
            shift 2
            ;;
        --joints)
            TARGET_JOINTS="$2"
            shift 2
            ;;
        --no-network)
            USE_NETWORK="false"
            shift
            ;;
        --no-bridge)
            USE_DDS_BRIDGE="false"
            shift
            ;;
        --help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  --rk3588_ip IP       RK3588 IP address (default: 192.168.1.20)"
            echo "  --rk3588_port PORT   RK3588 port (default: 8888)"
            echo "  --rate Hz            Publish rate (default: 50)"
            echo "  --amplitude VALUE    Sine wave amplitude (default: 0.3)"
            echo "  --frequency VALUE    Sine wave frequency (default: 0.5)"
            echo "  --joints [1,2,3]     Target joint IDs (default: [4,5,10,11])"
            echo "  --no-network         Disable network transmission"
            echo "  --no-bridge          Disable DDS bridge file"
            echo "  --help               Show this help"
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# 执行主函数
main