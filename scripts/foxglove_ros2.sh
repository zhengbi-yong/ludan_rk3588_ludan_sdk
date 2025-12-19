#!/bin/bash

# Foxglove ROS2 WebSocket Bridge Daemon
# Modified from fox.sh to run as a background daemon service
# Starts Foxglove Bridge for ROS2 topics visualization in background

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Configuration
DEFAULT_PORT=8765
DEFAULT_HOST="0.0.0.0"
DEFAULT_RATE_LIMIT=100
LOG_LEVEL="${LOG_LEVEL:-info}"
DEFAULT_DOMAIN_ID=0
DAEMON_MODE="${DAEMON_MODE:-true}"

# Parse command line arguments
PORT=${FOXGLOVE_PORT:-$DEFAULT_PORT}
HOST=${FOXGLOVE_HOST:-$DEFAULT_HOST}
RATE_LIMIT=${FOXGLOVE_RATE_LIMIT:-$DEFAULT_RATE_LIMIT}
DOMAIN_ID=${ROS_DOMAIN_ID:-$DEFAULT_DOMAIN_ID}

# PID file for daemon management
PIDFILE="/tmp/foxglove_ros2.pid"
LOG_DIR="$HOME/.foxglove/logs"
LOG_FILE="$LOG_DIR/foxglove_ros2_$(date +%Y%m%d_%H%M%S).log"

# Function to print colored status
print_status() {
    local status=$1
    local message=$2
    case $status in
        "INFO")
            echo -e "${GREEN}[INFO]${NC} $message"
            ;;
        "WARN")
            echo -e "${YELLOW}[WARN]${NC} $message"
            ;;
        "ERROR")
            echo -e "${RED}[ERROR]${NC} $message"
            ;;
        "SUCCESS")
            echo -e "${GREEN}[SUCCESS]${NC} $message"
            ;;
        *)
            echo "[INFO] $message"
            ;;
    esac
}

# Function to check if already running
check_running() {
    if [ -f "$PIDFILE" ]; then
        local pid=$(cat "$PIDFILE")
        if ps -p $pid > /dev/null 2>&1; then
            print_status "WARN" "Foxglove Bridge is already running with PID $pid"
            return 0
        else
            # PID file exists but process not running, clean up
            rm -f "$PIDFILE"
        fi
    fi
    return 1
}

# Function to daemonize
daemonize() {
    # Fork the process
    (
        # Double fork to detach from terminal
        setsid > /dev/null
        exec "$0" "--daemon-child" "$@"
    ) &
    echo $! > "$PIDFILE"
    return 0
}

# Function to start the bridge in daemon mode
start_daemon() {
    print_status "INFO" "Starting Foxglove ROS2 Bridge daemon..."

    # Create necessary directories
    mkdir -p "$LOG_DIR"

    # Set up ROS environment
    if [ -z "$ROS_DISTRO" ]; then
        if [ -f "/opt/ros/humble/setup.bash" ]; then
            source /opt/ros/humble/setup.bash
        elif [ -f "/opt/ros/foxy/setup.bash" ]; then
            source /opt/ros/foxy/setup.bash
        else
            print_status "ERROR" "ROS2 environment not found"
            exit 1
        fi
    fi

    # Source custom packages
    if [ -d "/home/linaro/motor_feedback/install" ]; then
        print_status "INFO" "Sourcing motor_feedback package..."
        source /home/linaro/motor_feedback/install/setup.bash
    fi

    # Set ROS domain ID
    export ROS_DOMAIN_ID=$DOMAIN_ID

    # Find foxglove bridge package
    BRIDGE_PACKAGE=""
    for pkg in "foxglove_bridge" "ros_foxglove_bridge" "foxglove_bridge_ros2"; do
        if ros2 pkg list 2>/dev/null | grep -q "$pkg"; then
            BRIDGE_PACKAGE=$pkg
            break
        fi
    done

    if [ -z "$BRIDGE_PACKAGE" ]; then
        print_status "ERROR" "Foxglove Bridge package not found"
        print_status "INFO" "Install with: sudo apt install ros-\$ROS_DISTRO-foxglove-bridge"
        exit 1
    fi

    # Check if port is available
    if command -v lsof &> /dev/null; then
        if lsof -Pi :$PORT -sTCP:LISTEN -t >/dev/null 2>&1; then
            print_status "WARN" "Port $PORT is already in use"
            # Try alternative ports
            for alt_port in {8766..8775}; do
                if ! lsof -Pi :$alt_port -sTCP:LISTEN -t >/dev/null 2>&1; then
                    PORT=$alt_port
                    print_status "INFO" "Using alternative port: $PORT"
                    break
                fi
            done
        fi
    fi

    # Start the Foxglove Bridge daemon
    print_status "INFO" "Starting bridge on port $PORT..."
    print_status "INFO" "WebSocket URL: ws://$HOST:$PORT"
    print_status "INFO" "Log file: $LOG_FILE"
    print_status "INFO" "PID file: $PIDFILE"

    # Launch foxglove bridge in background
    nohup ros2 launch $BRIDGE_PACKAGE foxglove_bridge_launch.xml \
        port:=$PORT \
        address:=$HOST \
        rate_limit:=$RATE_LIMIT \
        log_level:=$LOG_LEVEL \
        > "$LOG_FILE" 2>&1 &

    local pid=$!
    echo $pid > "$PIDFILE"

    # Wait a moment for startup
    sleep 2

    # Check if started successfully
    if kill -0 $pid 2>/dev/null; then
        print_status "SUCCESS" "Foxglove Bridge daemon started successfully!"
        print_status "INFO" "PID: $pid"
        print_status "INFO" "WebSocket: ws://$HOST:$PORT"
        print_status "INFO" "Connect Foxglove Studio to ws://$HOST:$PORT"

        # Log startup info
        echo "$(date): Foxglove Bridge started (PID: $pid, Port: $PORT)" >> "$LOG_FILE"

        return 0
    else
        print_status "ERROR" "Failed to start Foxglove Bridge"
        rm -f "$PIDFILE"
        return 1
    fi
}

# Function to stop the daemon
stop_daemon() {
    if [ -f "$PIDFILE" ]; then
        local pid=$(cat "$PIDFILE")
        if ps -p $pid > /dev/null 2>&1; then
            print_status "INFO" "Stopping Foxglove Bridge daemon (PID: $pid)..."
            kill $pid

            # Wait for process to stop
            local count=0
            while kill -0 $pid 2>/dev/null && [ $count -lt 10 ]; do
                sleep 1
                count=$((count + 1))
            done

            if kill -0 $pid 2>/dev/null; then
                print_status "WARN" "Force killing Foxglove Bridge daemon"
                kill -9 $pid 2>/dev/null
            fi

            print_status "SUCCESS" "Foxglove Bridge daemon stopped"
            echo "$(date): Foxglove Bridge stopped (PID: $pid)" >> "$LOG_FILE"
        else
            print_status "WARN" "Foxglove Bridge daemon not running (stale PID file)"
        fi
        rm -f "$PIDFILE"
    else
        print_status "INFO" "Foxglove Bridge daemon is not running"
    fi
}

# Function to check daemon status
check_status() {
    if [ -f "$PIDFILE" ]; then
        local pid=$(cat "$PIDFILE")
        if ps -p $pid > /dev/null 2>&1; then
            print_status "SUCCESS" "Foxglove Bridge daemon is running"
            print_status "INFO" "PID: $pid"
            print_status "INFO" "WebSocket: ws://$HOST:$PORT"
            print_status "INFO" "Log: $LOG_FILE"

            # Show network connections
            if command -v netstat &> /dev/null; then
                local connections=$(netstat -an 2>/dev/null | grep ":$PORT " | wc -l)
                print_status "INFO" "Active connections: $connections"
            fi

            return 0
        else
            print_status "INFO" "Foxglove Bridge daemon is not running"
            rm -f "$PIDFILE"
            return 1
        fi
    else
        print_status "INFO" "Foxglove Bridge daemon is not running"
        return 1
    fi
}

# Function to show logs
show_logs() {
    if [ -f "$LOG_FILE" ]; then
        print_status "INFO" "Showing last 50 lines of log file:"
        echo "----------------------------------------"
        tail -50 "$LOG_FILE"
    else
        print_status "INFO" "No log file found"
    fi
}

# Function to show available topics
show_topics() {
    # Check if ROS2 is sourced
    if [ -z "$ROS_DISTRO" ]; then
        if [ -f "/opt/ros/humble/setup.bash" ]; then
            source /opt/ros/humble/setup.bash
        elif [ -f "/opt/ros/foxy/setup.bash" ]; then
            source /opt/ros/foxy/setup.bash
        fi
    fi

    print_status "INFO" "Available ROS2 Topics:"
    ros2 topic list 2>/dev/null | while read topic; do
        type=$(ros2 topic info "$topic" 2>/dev/null | grep "Type:" | cut -d' ' -f2)
        echo "  $topic ($type)"
    done
}

# Function to restart the daemon
restart_daemon() {
    print_status "INFO" "Restarting Foxglove Bridge daemon..."
    stop_daemon
    sleep 1
    start_daemon
}

# Show usage information
show_usage() {
    echo "Foxglove ROS2 WebSocket Bridge Daemon"
    echo ""
    echo "Usage: $0 {start|stop|restart|status|logs|topics|help}"
    echo ""
    echo "Commands:"
    echo "  start    Start the Foxglove Bridge daemon"
    echo "  stop     Stop the Foxglove Bridge daemon"
    echo "  restart  Restart the Foxglove Bridge daemon"
    echo "  status   Show daemon status"
    echo "  logs     Show recent log entries"
    echo "  topics   List available ROS2 topics"
    echo "  help     Show this help message"
    echo ""
    echo "Environment Variables:"
    echo "  FOXGLOVE_PORT     WebSocket port (default: 8765)"
    echo "  FOXGLOVE_HOST     Bind address (default: 0.0.0.0)"
    echo "  LOG_LEVEL         Log level (default: info)"
    echo "  ROS_DOMAIN_ID      ROS2 domain ID (default: 0)"
    echo ""
    echo "Examples:"
    echo "  $0 start                    # Start daemon"
    echo "  $0 start                    # Start on custom port"
    echo "  FOXGLOVE_PORT=8766 $0 start # Start on port 8766"
}

# Handle daemon child process (internal)
if [ "$1" = "--daemon-child" ]; then
    shift
    exec "$@"
    exit 0
fi

# Main command handling
case "${1:-help}" in
    "start")
        print_status "INFO" "Foxglove ROS2 Bridge Daemon"
        echo "======================================"

        if check_running; then
            print_status "ERROR" "Daemon is already running"
            exit 1
        fi

        start_daemon
        ;;
    "stop")
        print_status "INFO" "Foxglove ROS2 Bridge Daemon"
        echo "======================================"

        stop_daemon
        ;;
    "restart")
        print_status "INFO" "Foxglove ROS2 Bridge Daemon"
        echo "======================================"

        restart_daemon
        ;;
    "status")
        print_status "INFO" "Foxglove ROS2 Bridge Daemon Status"
        echo "======================================"

        check_status
        ;;
    "logs")
        print_status "INFO" "Foxglove ROS2 Bridge Daemon Logs"
        echo "======================================"

        show_logs
        ;;
    "topics")
        print_status "INFO" "ROS2 Topics"
        echo "======================================"

        show_topics
        ;;
    "help"|"-h"|"--help")
        show_usage
        ;;
    *)
        print_status "ERROR" "Unknown command: $1"
        echo ""
        show_usage
        exit 1
        ;;
esac