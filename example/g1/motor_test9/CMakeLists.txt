cmake_minimum_required(VERSION 3.5)
project(motor_test9 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# ZLG CANFDNET SDK paths
set(ZCAN_SDK_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../LINUX_CANFDNET_V1.3.0.5_example)
set(ZCAN_INCLUDE_DIR ${ZCAN_SDK_ROOT}/src)
set(ZCAN_LIB_DIR ${ZCAN_SDK_ROOT}/lib/linux_x64)

# Include directories
include_directories(${ZCAN_INCLUDE_DIR})

# Source files
set(SOURCES
    src/motor_test9.cpp
)

# Create executable
add_executable(motor_test9 ${SOURCES})

# Link ZLG library
target_link_libraries(motor_test9 ${ZCAN_LIB_DIR}/libCANFDNET.a)

# Link pthread
target_link_libraries(motor_test9 pthread)

# Installation
install(TARGETS motor_test9 DESTINATION bin)

message(STATUS "Motor Test9 Configuration:")
message(STATUS "  ZCAN SDK: ${ZCAN_SDK_ROOT}")
message(STATUS "  Output: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/motor_test9")
