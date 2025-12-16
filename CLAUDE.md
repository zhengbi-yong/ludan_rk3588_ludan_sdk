# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is the Unitree SDK2 - a robotics SDK for Unitree robots (version 2). It provides C++ APIs for controlling and communicating with various Unitree robot models including A2, B2, G1, H1, GO2, and Ludan platforms.

## Build Commands

### Standard Build with Examples
```bash
mkdir build && cd build
cmake ..
make
```

### Build without Examples
```bash
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=OFF
make
```

### Install SDK System-wide
```bash
mkdir build && cd build
cmake ..
sudo make install
```

### Install SDK to Custom Directory
```bash
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install
```

### Build Specific Architecture
The build system automatically detects architecture and links appropriate pre-compiled libraries from `lib/${CMAKE_SYSTEM_PROCESSOR}/`.

## Project Structure

### Core Architecture
- `/include/unitree/robot/` - Main API headers organized by:
  - `client/` - Client-side communication interfaces
  - `server/` - Server-side communication interfaces
  - `channel/` - Data channel abstractions (publishers/subscribers)
  - Robot-specific directories: `a2/`, `b2/`, `g1/`, `h1/`, `go2/`, `ludan/`
- `/lib/${ARCH}/` - Pre-compiled static libraries (aarch64, x86_64)
- `/thirdparty/` - DDS (Data Distribution Service) libraries for real-time communication

### Build System
- Uses CMake 3.5+ with C++17 standard
- Links against static `libunitree_sdk2.a` library
- Depends on DDS libraries (`ddsc`, `ddscxx`) for real-time communication
- Automatic architecture detection for library selection

## Development Workflow

### Using the SDK in Your Project
```cmake
find_package(unitree_sdk2 REQUIRED)
target_link_libraries(your_target unitree_sdk2)
```

Note: If installed to non-standard location, add path to `CMAKE_PREFIX_PATH`.

### Robot-Specific APIs
Each robot model has dedicated APIs for:
- Motion control and sport modes
- Video streaming (front/back cameras)
- Robot state monitoring
- Audio processing
- Configuration management

### Example Programs
Located in `/example/` organized by robot model:
- `helloworld/` - Basic DDS publisher/subscriber example
- `g1/`, `b2/`, `a2/` - Robot-specific basic examples
- `ludan/` - Advanced examples including AGV, dexterous arm control

## Testing and Development

### Running Examples and Tests
Examples serve as both documentation and functional tests. Built examples are located in `/build/bin/`:

```bash
# Basic DDS communication test (run in separate terminals)
./test_publisher
./test_subscriber

# Robot-specific examples (if robot hardware available)
./g1_low_level_example
./b2_example
./h1_example

# Development/testing without robot hardware
./example/g1/low_level/test_ankle_swing.sh [interface]  # Uses state simulator
```

### Development Mode Testing
For development without physical robots, use the debug examples and simulators:
- `state_simulator` - Provides fake robot state feedback
- `command_monitor` - Logs all commands sent to robot
- `*_debug.cpp` versions - Debug versions with enhanced logging

The test script `example/g1/low_level/test_ankle_swing.sh` demonstrates the complete development workflow.

## Environment Dependencies

Required system packages (Ubuntu 20.04):
- cmake (3.10+)
- g++ (9.4.0)
- libyaml-cpp-dev
- libeigen3-dev
- libboost-all-dev
- libspdlog-dev
- libfmt-dev

## Architecture Support

- Pre-compiled libraries for aarch64 (ARM64) and x86_64
- Cross-platform support for embedded and desktop development
- Docker development container available with GPU/Display forwarding

## Key Technical Details

- Communication via DDS middleware for real-time performance
- Static linking provided for easy integration
- Header-only public API with static library implementation
- Thread-safe operations with built-in synchronization
- CRC validation for data integrity