#!/bin/bash
# ROS1 to DDS Deployment Script
# 在Jetson上部署ROS1到DDS桥接

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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

# 默认参数
RK3588_IP="${RK3588_IP:-192.168.1.20}"
RK3588_PORT="${RK3588_PORT:-8888}"
USE_DDS="${USE_DDS:-false}"
PUBLISH_RATE="${PUBLISH_RATE:-50}"
SINE_AMPLITUDE="${SINE_AMPLITUDE:-0.3}"
SINE_FREQUENCY="${SINE_FREQUENCY:-0.5}"
TARGET_JOINTS="${TARGET_JOINTS:-[0,1,2]}"
TEST_COMMUNICATION="${TEST_COMMUNICATION:-true}"

# 显示配置
show_config() {
    log_info "ROS1 to DDS Bridge Configuration:"
    echo "  RK3588 IP: $RK3588_IP"
    echo "  RK3588 Port: $RK3588_PORT"
    echo "  Use DDS: $USE_DDS"
    echo "  Publish Rate: $PUBLISH_RATE Hz"
    echo "  Sine Amplitude: $SINE_AMPLITUDE rad"
    echo "  Sine Frequency: $SINE_FREQUENCY Hz"
    echo "  Target Joints: $TARGET_JOINTS"
    echo "  Test Communication: $TEST_COMMUNICATION"
    echo ""
}

# 检查环境
check_environment() {
    log_info "Checking environment..."

    # 检查ROS1
    if [ -z "$ROS_DISTRO" ]; then
        log_error "ROS1 environment not sourced. Please run: source /opt/ros/noetic/setup.bash"
        exit 1
    fi
    log_success "ROS1 ($ROS_DISTRO) found"

    # 检查Python
    if ! command -v python3 &> /dev/null; then
        log_error "Python3 not found"
        exit 1
    fi
    log_success "Python3 found"

    # 检查必要的工作目录
    if [ ! -f "/home/linaro/ros1_to_dds_bridge.py" ]; then
        log_error "Bridge script not found at /home/linaro/ros1_to_dds_bridge.py"
        exit 1
    fi
    log_success "Bridge script found"

    # 设置可执行权限
    chmod +x /home/linaro/ros1_to_dds_bridge.py
    chmod +x /home/linaro/test_ros1_dds_communication.py

    log_success "Environment check completed"
}

# 网络连通性测试
test_network_connectivity() {
    log_info "Testing network connectivity to RK3588 ($RK3588_IP)..."

    if ping -c 3 "$RK3588_IP" &> /dev/null; then
        log_success "✅ Network connectivity to RK3588 OK"
    else
        log_error "❌ Cannot reach RK3588 at $RK3588_IP"
        log_info "Please check:"
        echo "  1. Network cable connection"
        echo "  2. IP address configuration"
        echo "  3. Firewall settings"
        echo ""
        log_info "Current network configuration:"
        ip addr show | grep -A 2 "eth0\|enP" | grep "inet"
        exit 1
    fi
}

# 运行通信测试
run_communication_test() {
    if [ "$TEST_COMMUNICATION" = "true" ]; then
        log_info "Running communication test..."

        python3 /home/linaro/test_ros1_dds_communication.py \
            --rk3588_ip:="$RK3588_IP" \
            --test_port:=$((RK3588_PORT + 1)) \
            --test_duration:=10 \
            --test_frequency:=5

        if [ $? -eq 0 ]; then
            log_success "✅ Communication test passed"
        else
            log_warning "⚠️ Communication test failed, but continuing deployment..."
        fi
    fi
}

# 启动ROS1核心
start_ros_core() {
    log_info "Starting ROS Core..."

    # 检查roscore是否已经在运行
    if pgrep -f "roscore" > /dev/null; then
        log_info "ROS Core is already running"
    else
        roscore &
        ROSCORE_PID=$!
        sleep 2
        log_success "ROS Core started (PID: $ROSCORE_PID)"
    fi
}

# 启动桥接节点
start_bridge() {
    log_info "Starting ROS1 to DDS Bridge..."

    # 创建参数配置
    cat > /tmp/bridge_params.yaml << EOF
rk3588_ip: "$RK3588_IP"
rk3588_port: $RK3588_PORT
use_dds: $USE_DDS
publish_rate: $PUBLISH_RATE
sine_amplitude: $SINE_AMPLITUDE
sine_frequency: $SINE_FREQUENCY
target_joints: $TARGET_JOINTS
EOF

    # 启动桥接节点
    python3 /home/linaro/ros1_to_dds_bridge.py &
    BRIDGE_PID=$!

    sleep 2

    if kill -0 $BRIDGE_PID 2>/dev/null; then
        log_success "✅ Bridge started successfully (PID: $BRIDGE_PID)"
    else
        log_error "❌ Failed to start bridge"
        exit 1
    fi

    # 保存PID用于清理
    echo $BRIDGE_PID > /tmp/bridge.pid
    if [ ! -z "$ROSCORE_PID" ]; then
        echo $ROSCORE_PID > /tmp/roscore.pid
    fi
}

# 启动监控
start_monitoring() {
    log_info "Starting monitoring tools..."

    # 启动ROS话题监控
    rostopic list &
    ROSTOPIC_PID=$!

    # 启动ROS节点监控
    rosnode list &
    ROSNODE_PID=$!

    log_success "Monitoring started"
}

# 显示状态信息
show_status() {
    log_info "Bridge Status Information:"
    echo ""

    log_info "ROS Topics:"
    rostopic list 2>/dev/null || log_warning "Cannot list ROS topics"

    echo ""
    log_info "ROS Nodes:"
    rosnode list 2>/dev/null || log_warning "Cannot list ROS nodes"

    echo ""
    log_info "Network Connections:"
    netstat -uln | grep ":$RK3588_PORT\:$((RK3588_PORT + 1))" || log_warning "No active connections on expected ports"

    echo ""
    log_info "Bridge is running. Press Ctrl+C to stop."
}

# 清理函数
cleanup() {
    log_info "Cleaning up..."

    # 停止桥接节点
    if [ -f /tmp/bridge.pid ]; then
        BRIDGE_PID=$(cat /tmp/bridge.pid)
        if kill -0 $BRIDGE_PID 2>/dev/null; then
            kill $BRIDGE_PID
            log_success "Bridge stopped"
        fi
        rm -f /tmp/bridge.pid
    fi

    # 停止ROS Core
    if [ -f /tmp/roscore.pid ]; then
        ROSCORE_PID=$(cat /tmp/roscore.pid)
        if kill -0 $ROSCORE_PID 2>/dev/null; then
            kill $ROSCORE_PID
            log_success "ROS Core stopped"
        fi
        rm -f /tmp/roscore.pid
    fi

    # 停止监控
    kill $ROSTOPIC_PID 2>/dev/null || true
    kill $ROSNODE_PID 2>/dev/null || true

    log_success "Cleanup completed"
    exit 0
}

# 主函数
main() {
    echo "=================================================="
    echo "    ROS1 to DDS Bridge Deployment Script       "
    echo "=================================================="
    echo ""

    show_config

    # 设置信号处理
    trap cleanup SIGINT SIGTERM

    check_environment
    test_network_connectivity
    run_communication_test

    start_ros_core
    start_bridge
    start_monitoring

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
        --use_dds)
            USE_DDS="$2"
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
        --no-test)
            TEST_COMMUNICATION="false"
            shift
            ;;
        *)
            log_error "Unknown option: $1"
            echo "Usage: $0 [--rk3588_ip IP] [--rk3588_port PORT] [--amplitude VALUE] [--frequency VALUE] [--joints [1,2,3]] [--no-test]"
            exit 1
            ;;
    esac
done

# 执行主函数
main