#!/bin/bash
# Build script for motor_test

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

echo "========================================="
echo "  Motor Test Build Script"
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
echo "Executable: ${BUILD_DIR}/motor_test"
echo ""
echo "Usage:"
echo "  cd ${BUILD_DIR}"
echo "  ./motor_test [options]"
echo ""
echo "Options:"
echo "  --motor-id <id>       Motor ID to control (default: 1)"
echo "  --amplitude <rad>     Sine wave amplitude in rad (default: 1.0)"
echo "  --frequency <hz>      Sine wave frequency in Hz (default: 0.5)"
echo "  --kp <value>          Position gain (default: 50.0)"
echo "  --kd <value>          Velocity gain (default: 1.5)"
echo "  --rate <hz>           Control rate in Hz (default: 500)"
echo "  --no-enable           Don't auto-enable motor"
echo "  -h, --help            Show help message"
echo ""
echo "Example:"
echo "  ./motor_test --motor-id 1 --amplitude 0.5 --frequency 1.0"
echo ""
