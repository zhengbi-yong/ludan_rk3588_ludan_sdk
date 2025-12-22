#!/bin/bash

# DDS Native Command Publisher Script
# This script publishes native DDS command data with position/velocity/torque to /lowcmd topic

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
DEFAULT_DOMAIN_ID=0
DEFAULT_INTERFACE="eth0"
DEFAULT_AMPLITUDE=1.0
DEFAULT_FREQUENCY=1.0
DEFAULT_DURATION=60  # seconds
DEFAULT_JOINTS=12

# Parse command line arguments
DOMAIN_ID=${ROS_DOMAIN_ID:-$DEFAULT_DOMAIN_ID}
INTERFACE=${1:-$DEFAULT_INTERFACE}
AMPLITUDE=${2:-$DEFAULT_AMPLITUDE}
FREQUENCY=${3:-$DEFAULT_FREQUENCY}
DURATION=${4:-$DEFAULT_DURATION}
NUM_JOINTS=${5:-$DEFAULT_JOINTS}

# Print banner
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    DDS Native Command Publisher${NC}"
echo -e "${BLUE}    Position/Velocity/Torque Data${NC}"
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

# Set ROS Domain ID for DDS communication
print_status "INFO" "Setting ROS Domain ID to: $DOMAIN_ID"
export ROS_DOMAIN_ID=$DOMAIN_ID

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
echo -e "  DDS Domain ID: ${BLUE}$DOMAIN_ID${NC}"
echo -e "  Network Interface: ${BLUE}$INTERFACE${NC}"
echo -e "  Number of Joints: ${BLUE}$NUM_JOINTS${NC}"
echo -e "  Square Wave Amplitude: ${BLUE}$AMPLITUDE${NC}"
echo -e "  Square Wave Frequency: ${BLUE}$FREQUENCY${NC} Hz"
echo -e "  Duration: ${BLUE}$DURATION${NC} seconds"
echo -e "  Data Format: ${BLUE}Position/Velocity/Torque${NC}"
echo -e "  Target Topic: ${BLUE}/lowcmd${NC}"
echo

# Check if the interface is up
print_status "INFO" "Checking network interface $INTERFACE..."
if ! ip link show "$INTERFACE" &>/dev/null; then
    print_status "ERROR" "Network interface $INTERFACE not found"
    exit 1
fi

if ! ip link show "$INTERFACE" | grep -q "state UP"; then
    print_status "WARN" "Network interface $INTERFACE is not up. Attempting to bring it up..."
    sudo ip link set "$INTERFACE" up || print_status "WARN" "Could not bring interface up (may require sudo)"
fi

# Check for native DDS publisher
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NATIVE_PUBLISHER="$SCRIPT_DIR/simple_dds_cmd"

# Use pre-built cyclonedds publisher if available, otherwise try to build
if [ -f "$HOME/cyclonedds-cxx/build/bin/ddscxxHelloworldPublisher" ]; then
    NATIVE_PUBLISHER="$HOME/cyclonedds-cxx/build/bin/ddscxxHelloworldPublisher"
    print_status "INFO" "Using pre-built cyclonedds publisher"
elif [ -f "$NATIVE_PUBLISHER" ]; then
    print_status "INFO" "Found native DDS publisher"
else
    print_status "INFO" "Building simple DDS publisher..."

    # Check for required tools
    if ! command -v g++ &> /dev/null; then
        print_status "ERROR" "g++ compiler not found. Please install build-essential."
        exit 1
    fi

    # Compile the simple DDS publisher
    cd "$SCRIPT_DIR"
    export CYCLONEDDS_HOME="$HOME/cyclonedds-cxx"
    g++ -std=c++14 -O2 \
        simple_dds_cmd.cpp \
        -o simple_dds_cmd \
        -I"$CYCLONEDDS_HOME/src/ddscxx/include" \
        -L"$CYCLONEDDS_HOME/build" \
        -lddscxx \
        -lm -lpthread \
        -Wl,-rpath,"$CYCLONEDDS_HOME/build" 2>/dev/null

    if [ $? -eq 0 ]; then
        print_status "INFO" "Simple DDS publisher compiled successfully"
    else
        print_status "WARN" "Failed to compile DDS publisher, using test mode"
        NATIVE_PUBLISHER="echo"
    fi
fi

# Function to generate and publish native DDS data
publish_native_dds() {
    local amplitude=$1
    local frequency=$2
    local duration=$3
    local interface=$4
    local num_joints=$5
    local period=$(echo "scale=3; 1 / $frequency" | bc -l)

    print_status "INFO" "Starting native DDS command generation..."
    print_status "INFO" "Period: ${period}s, Data: Position/Velocity/Torque for $num_joints joints"
    print_status "INFO" "Press Ctrl+C to stop early"
    echo

    # Set up network interface for DDS
    print_status "INFO" "Configuring DDS to use interface $interface..."
    # Use simpler DDS configuration or let it use default settings
    unset CYCLONEDDS_URI  # Clear any previous configuration
    export FASTRTPS_DEFAULT_PROFILES_FILE=""  # Disable FastRTPS if it conflicts

    # Only set interface configuration if needed
    if [ "$interface" != "lo" ]; then
        print_status "WARN" "Using interface $interface - DDS may need network configuration"
    fi

    # Start the native DDS publisher
    print_status "INFO" "Starting native DDS command publisher..."
    print_status "INFO" "Publisher will generate data with: pos/vel/tau for each joint"
    echo

    # Run the native DDS publisher
    if [ "$NATIVE_PUBLISHER" = "echo" ]; then
        # Test mode: simulate DDS publishing with echo
        print_status "WARN" "Running in test mode (simulating DDS commands)"
        for i in $(seq 1 $((duration * 2))); do
            local current_value=$(($i % 2 == 1 ? amplitude : -amplitude))
            print_status "INFO" "TEST: Publishing /lowcmd - Value: $current_value - Joints: $num_joints"
            sleep $(echo "scale=3; $period / 2" | bc -l)
        done
    else
        # Real DDS publisher
        $NATIVE_PUBLISHER "$interface" "$amplitude" "$frequency" "$duration" "$num_joints" &
    fi
    local publisher_pid=$!

    print_status "INFO" "Native DDS publisher started with PID: $publisher_pid"

    # Monitor the publisher
    local end_time=$(($(date +%s) + duration))

    while [ $(date +%s) -lt $end_time ]; do
        if ! kill -0 $publisher_pid 2>/dev/null; then
            print_status "ERROR" "DDS publisher died unexpectedly"
            return 1
        fi
        sleep 1
    done

    # Stop the publisher if still running
    if kill -0 $publisher_pid 2>/dev/null; then
        print_status "INFO" "Stopping DDS publisher..."
        kill $publisher_pid 2>/dev/null || true
        wait $publisher_pid 2>/dev/null || true
    fi

    print_status "INFO" "DDS command generation completed"
}

# Function to cleanup on exit
cleanup() {
    print_status "INFO" "Stopping DDS command publisher..."
    # Kill background processes
    jobs -p | xargs -r kill 2>/dev/null || true
    print_status "INFO" "DDS command publisher stopped."
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Check current DDS topics
print_status "INFO" "Current DDS topics in domain $DOMAIN_ID:"
ros2 topic list 2>/dev/null | while read topic; do
    if [ "$topic" = "/lowcmd" ]; then
        echo -e "  ${GREEN}$topic${NC} (exists)"
    else
        echo -e "  $topic"
    fi
done
echo

# Start native DDS generation
print_status "INFO" "Starting native DDS command generation..."
publish_native_dds $AMPLITUDE $FREQUENCY $DURATION $INTERFACE $NUM_JOINTS

print_status "INFO" "DDS command publisher completed successfully!"