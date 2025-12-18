#!/bin/bash

# Foxglove Bridge Launcher Script
# This script starts the Foxglove WebSocket bridge for ROS2

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
DEFAULT_PORT=8765
DEFAULT_HOST="0.0.0.0"
DEFAULT_RATE_LIMIT=100
LOG_LEVEL="${LOG_LEVEL:-info}"

# Parse command line arguments
PORT=${FOXGLOVE_PORT:-$DEFAULT_PORT}
HOST=${FOXGLOVE_HOST:-$DEFAULT_HOST}
RATE_LIMIT=${FOXGLOVE_RATE_LIMIT:-$DEFAULT_RATE_LIMIT}

# Print banner
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    Foxglove Bridge Launcher${NC}"
echo -e "${BLUE}========================================${NC}"
echo

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
        *)
            echo "[INFO] $message"
            ;;
    esac
}

# Check if ROS2 is sourced
print_status "INFO" "Checking ROS2 environment..."
if [ -z "$ROS_DISTRO" ]; then
    print_status "WARN" "ROS2 environment not sourced. Attempting to source..."
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        print_status "INFO" "ROS2 Humble sourced successfully"
    elif [ -f "/opt/ros/foxy/setup.bash" ]; then
        source /opt/ros/foxy/setup.bash
        print_status "INFO" "ROS2 Foxy sourced successfully"
    else
        print_status "ERROR" "Could not find ROS2 installation. Please source your ROS2 environment."
        exit 1
    fi
fi

# Display configuration
print_status "INFO" "Configuration:"
echo -e "  Host: ${BLUE}$HOST${NC}"
echo -e "  Port: ${BLUE}$PORT${NC}"
echo -e "  Rate Limit: ${BLUE}$RATE_LIMIT${NC} msg/s"
echo -e "  Log Level: ${BLUE}$LOG_LEVEL${NC}"
echo

# Check if foxglove bridge is installed
print_status "INFO" "Checking Foxglove Bridge installation..."
if ! command -v ros2 &> /dev/null; then
    print_status "ERROR" "ROS2 command not found. Please ensure ROS2 is properly installed."
    exit 1
fi

# Try to find foxglove bridge package
BRIDGE_PACKAGE=""
for pkg in "foxglove_bridge" "ros_foxglove_bridge" "foxglove_bridge_ros2"; do
    if ros2 pkg list | grep -q "$pkg"; then
        BRIDGE_PACKAGE=$pkg
        break
    fi
done

if [ -z "$BRIDGE_PACKAGE" ]; then
    print_status "ERROR" "Foxglove Bridge package not found. Please install it:"
    echo "sudo apt install ros-\$ROS_DISTRO-foxglove-bridge"
    exit 1
fi

print_status "INFO" "Found Foxglove Bridge package: $BRIDGE_PACKAGE"

# Check if port is already in use
print_status "INFO" "Checking if port $PORT is available..."
if command -v lsof &> /dev/null; then
    if lsof -Pi :$PORT -sTCP:LISTEN -t >/dev/null; then
        print_status "WARN" "Port $PORT is already in use. Trying to find another port..."
        for alt_port in {8766..8775}; do
            if ! lsof -Pi :$alt_port -sTCP:LISTEN -t >/dev/null; then
                PORT=$alt_port
                print_status "INFO" "Using alternative port: $PORT"
                break
            fi
        done
    fi
fi

# Create log directory
LOG_DIR="$HOME/.foxglove/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/foxglove_bridge_$(date +%Y%m%d_%H%M%S).log"

print_status "INFO" "Log file: $LOG_FILE"

# Function to cleanup on exit
cleanup() {
    print_status "INFO" "Shutting down Foxglove Bridge..."
    # Kill background processes
    jobs -p | xargs -r kill 2>/dev/null || true
    print_status "INFO" "Foxglove Bridge stopped."
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Start Foxglove Bridge
print_status "INFO" "Starting Foxglove WebSocket Bridge..."
print_status "INFO" "WebSocket URL: ws://$HOST:$PORT"
echo

# Create launch file if it doesn't exist or use the built-in one
LAUNCH_FILE=""

# Start the bridge in background
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    port:=$PORT \
    address:=$HOST \
    rate_limit:=$RATE_LIMIT \
    log_level:=$LOG_LEVEL \
    2>&1 | tee "$LOG_FILE" &

BRIDGE_PID=$!

print_status "INFO" "Foxglove Bridge started with PID: $BRIDGE_PID"

# Wait a moment for the bridge to start
sleep 3

# Check if the bridge is running
if kill -0 $BRIDGE_PID 2>/dev/null; then
    print_status "INFO" "Foxglove Bridge is running successfully!"
    echo
    print_status "INFO" "Connection Information:"
    echo -e "  ${GREEN}WebSocket URL:${NC} ws://$HOST:$PORT"
    echo -e "  ${GREEN}Foxglove Studio:${NC} Open https://studio.foxglove.dev/ and connect to ws://$HOST:$PORT"
    echo
    print_status "INFO" "Available ROS2 Topics:"
    ros2 topic list --no.arr | while read topic; do
        type=$(ros2 topic info $topic 2>/dev/null | grep "Type:" | cut -d' ' -f2)
        echo -e "  ${BLUE}$topic${NC} ($type)"
    done
    echo
    print_status "INFO" "Press Ctrl+C to stop the bridge"

    # Wait for the bridge process
    wait $BRIDGE_PID
else
    print_status "ERROR" "Failed to start Foxglove Bridge"
    print_status "INFO" "Check the log file for errors: $LOG_FILE"
    exit 1
fi
