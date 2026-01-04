cmake_minimum_required(VERSION 3.10)
project(multi_port_motor_feedback)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ZLG CANFDNET SDK paths
set(ZLG_SDK_DIR "${CMAKE_SOURCE_DIR}/../LINUX_CANFDNET_V1.3.0.5_example")

# Include directories
include_directories(
    ${ZLG_SDK_DIR}/src
    ${ZLG_SDK_DIR}/src/zlgcan
)

# Link directories
link_directories(
    ${ZLG_SDK_DIR}/lib/linux_x64
)

# Source files
set(SOURCES
    src/multi_port_motor_feedback.cpp
)

# Create main executable
add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} CANFDNET pthread)

# CAN send test executable
add_executable(can_send_test src/can_send_test.cpp)
target_link_libraries(can_send_test CANFDNET pthread)

# Install target
install(TARGETS ${PROJECT_NAME} can_send_test DESTINATION bin)
