#!/bin/bash
# Build script for motor_test3 (FDCAN version)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

echo "========================================="
echo "  Motor Test3 Build Script (FDCAN Mode)"
echo "========================================="

# Create build directory
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# Configure
echo "Configuring..."
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build
echo "Building..."
make -j$(nproc)

echo ""
echo "========================================="
echo "  Build Complete!"
echo "========================================="
echo ""
echo "Executable: ${BUILD_DIR}/motor_test3"
echo ""
echo "Usage:"
echo "  cd ${BUILD_DIR}"
echo "  ./motor_test3 [options]"
echo ""
echo "Options:"
echo "  --motor-id <id>       Motor ID to control (default: 9)"
echo "  --amplitude <rad>     Sine wave amplitude in rad (default: 0.5)"
echo "  --start-freq <hz>     Start frequency in Hz (default: 0.1)"
echo "  --end-freq <hz>       End frequency in Hz (default: 20.0)"
echo "  --freq-step <hz>      Frequency step in Hz (default: 0.05)"
echo "  --cycles <n>          Cycles per frequency point (default: 1.0)"
echo "  --kp <value>          Position gain (default: 10.0)"
echo "  --kd <value>          Velocity gain (default: 1.5)"
echo "  --rate <hz>           Control rate in Hz (default: 500)"
echo "  --no-enable           Don't auto-enable motor"
echo "  -h, --help            Show help message"
echo ""
echo "Note: This version sends FDCAN frames (not classic CAN)"
echo ""
echo "Example:"
echo "  ./motor_test3 --motor-id 11 --start-freq 0.1 --end-freq 20.0 --freq-step 0.2 --cycles 10.0"
echo ""
