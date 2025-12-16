#!/bin/bash

# Test script for G1 ankle swing example without real robot
# Usage: ./test_ankle_swing.sh [network_interface]

if [ $# -lt 1 ]; then
    INTERFACE="lo"  # Default to loopback
else
    INTERFACE=$1
fi

echo "========================================"
echo "G1 Ankle Swing Test - Debug Mode"
echo "========================================"
echo "Network Interface: $INTERFACE"
echo ""

# Kill any existing processes
echo "Cleaning up any existing processes..."
pkill -f "g1_ankle_swing_example"
pkill -f "command_monitor"
pkill -f "state_simulator"
sleep 1

echo "Starting debug session..."
echo ""

# Start state simulator in background
echo "1. Starting state simulator..."
./state_simulator $INTERFACE &
SIMULATOR_PID=$!
echo "   PID: $SIMULATOR_PID"
sleep 2

# Start command monitor in background
echo "2. Starting command monitor..."
./command_monitor $INTERFACE &
MONITOR_PID=$!
echo "   PID: $MONITOR_PID"
sleep 1

# Start the debug version of ankle swing example
echo "3. Starting G1 ankle swing example (debug version)..."
./g1_ankle_swing_example_debug $INTERFACE &
EXAMPLE_PID=$!
echo "   PID: $EXAMPLE_PID"
echo ""

echo "========================================"
echo "All processes started successfully!"
echo ""
echo "Command Monitor will show all sent commands"
echo "Ankle Swing Example will show its internal state"
echo "State Simulator provides fake robot feedback"
echo ""
echo "Press Ctrl+C to stop all processes"
echo "========================================"

# Wait for Ctrl+C
trap 'echo ""; echo "Stopping all processes..."; kill $SIMULATOR_PID $MONITOR_PID $EXAMPLE_PID 2>/dev/null; echo "Done."; exit' INT

# Keep script running
wait