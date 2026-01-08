cmake_minimum_required(VERSION 3.10)
project(multi_port_motor_feedback)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ZLG CANFDNET SDK paths
set(ZLG_SDK_DIR "${CMAKE_SOURCE_DIR}/../LINUX_CANFDNET_V1.3.0.5_example")

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${ZLG_SDK_DIR}/src
    ${ZLG_SDK_DIR}/src/zlgcan
    ${ZLG_SDK_DIR}/src/cJSON
)

# Link directories
link_directories(
    ${ZLG_SDK_DIR}/lib/linux_x64
)

# CANFDNET source files (compile directly to avoid ABI issues)
set(CANFDNET_SOURCES
    ${ZLG_SDK_DIR}/src/CANFDNET.cpp
    ${ZLG_SDK_DIR}/src/Device.cpp
    ${ZLG_SDK_DIR}/src/DeviceMgr.cpp
    ${ZLG_SDK_DIR}/src/NetPacket.cpp
    ${ZLG_SDK_DIR}/src/log.cpp
    ${ZLG_SDK_DIR}/src/common.cpp
    ${ZLG_SDK_DIR}/src/platform/linux/network.cpp
    ${ZLG_SDK_DIR}/src/service/TcpClient.cpp
    ${ZLG_SDK_DIR}/src/service/TcpServer.cpp
    ${ZLG_SDK_DIR}/src/service/UdpService.cpp
    ${ZLG_SDK_DIR}/src/cJSON/cJSON.c
)

# Zlib source files (needed for CANFDNET)
file(GLOB ZLIB_SOURCES ${ZLG_SDK_DIR}/src/3rd/zlib/*.c)

# CAN send test executable (using virtual CAN / SocketCAN)
add_executable(can_send_test
    src/can_send_test.cpp
    ${ZLG_SDK_DIR}/src/cJSON/cJSON.c
)
target_link_libraries(can_send_test pthread)

# Note: can_to_ros2.cpp was removed as it's a duplicate of can_motor_feedback_publisher.cpp
# The ROS2 package at multi_port_motor_feedback_ros2/ contains the actual CAN->ROS2 publisher

# Install non-ROS2 targets
install(TARGETS can_send_test DESTINATION bin)
