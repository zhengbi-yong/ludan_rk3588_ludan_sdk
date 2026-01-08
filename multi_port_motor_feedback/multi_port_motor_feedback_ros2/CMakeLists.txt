cmake_minimum_required(VERSION 3.5)
project(multi_port_motor_feedback
  VERSION 1.0.0
)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==================== ZLG CANFDNET SDK Paths ====================
set(ZLG_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../LINUX_CANFDNET_V1.3.0.5_example")

# Include directories
include_directories(
  ${ZLG_SDK_DIR}/src
  ${ZLG_SDK_DIR}/src/zlgcan
)

# Link directories
link_directories(
  ${ZLG_SDK_DIR}/lib/linux_x64
)

# ==================== ROS2 Dependencies ====================
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# ==================== Generate Messages ====================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorState.msg"
  "msg/LowState.msg"
  DEPENDENCIES std_msgs
)

# ==================== ZLG CANFDNET Sources ====================
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

# Zlib source files
file(GLOB ZLIB_SOURCES ${ZLG_SDK_DIR}/src/3rd/zlib/*.c)

# ==================== CAN Publisher Executable ====================
add_executable(can_motor_feedback_publisher
  src/can_motor_feedback_publisher.cpp
  ${CANFDNET_SOURCES}
  ${ZLIB_SOURCES}
)

target_include_directories(can_motor_feedback_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ZLG_SDK_DIR}/src
  ${ZLG_SDK_DIR}/src/zlgcan
)

ament_target_dependencies(can_motor_feedback_publisher
  rclcpp
  std_msgs
)

# Link against the generated interfaces and pthread
target_link_libraries(can_motor_feedback_publisher
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  pthread
)

# ==================== Install Targets ====================
install(TARGETS can_motor_feedback_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# ==================== Testing ====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
