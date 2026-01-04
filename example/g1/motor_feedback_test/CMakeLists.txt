cmake_minimum_required(VERSION 3.5)
project(motor_feedback_test CXX)

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
set(SOURCES_READER
    motor_feedback_reader.cpp
)

set(SOURCES_MONITOR
    can_monitor.cpp
)

# Create executables
add_executable(motor_feedback_reader ${SOURCES_READER})
add_executable(can_monitor ${SOURCES_MONITOR})

# Link ZLG library
target_link_libraries(motor_feedback_reader ${ZCAN_LIB_DIR}/libCANFDNET.a)
target_link_libraries(can_monitor ${ZCAN_LIB_DIR}/libCANFDNET.a)

# Link pthread
target_link_libraries(motor_feedback_reader pthread)
target_link_libraries(can_monitor pthread)

# Installation
install(TARGETS motor_feedback_reader can_monitor DESTINATION bin)

# Print build information
message(STATUS "Motor Feedback Reader Configuration:")
message(STATUS "  ZCAN SDK: ${ZCAN_SDK_ROOT}")
message(STATUS "  Output: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/motor_feedback_reader")
