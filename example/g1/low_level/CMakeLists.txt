cmake_minimum_required(VERSION 3.10)
project(motor_controller_with_enable)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(xixilowcmd REQUIRED)

# ZLG CANFDNET SDK paths
set(ZLG_SDK_ROOT "/home/linaro/ludan_sdk/LINUX_CANFDNET_V1.3.0.5_example")
set(ZLG_INCLUDE_DIRS "${ZLG_SDK_ROOT}/src" "${ZLG_SDK_ROOT}/src/zlgcan")
set(ZLG_LIB_DIR "${ZLG_SDK_ROOT}/lib/linux_x64")

# Unitree SDK include
set(UNITREE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../include")

# Include directories
include_directories(
  ${ZLG_INCLUDE_DIRS}
  ${UNITREE_INCLUDE_DIR}
)

# ==================== motor_controller_with_enable (with ZLG SDK) ====================
add_executable(motor_controller_with_enable motor_controller_with_enable.cpp)

# Link ROS2 libraries
ament_target_dependencies(motor_controller_with_enable
  rclcpp
  std_msgs
  sensor_msgs
  xixilowcmd
)

# Link ZLG CANFDNET library
target_link_libraries(motor_controller_with_enable
  ${ZLG_LIB_DIR}/libCANFDNET.so
  pthread
)

# ==================== motor_control_example (ROS2 only, no ZLG SDK) ====================
add_executable(motor_control_example motor_control_example.cpp)

# Link ROS2 libraries only (no ZLG SDK needed)
ament_target_dependencies(motor_control_example
  rclcpp
  xixilowcmd
)

# ==================== enable_motor_publisher (ROS2 only, no ZLG SDK) ====================
add_executable(enable_motor_publisher enable_motor_publisher.cpp)

# Link ROS2 libraries only
ament_target_dependencies(enable_motor_publisher
  rclcpp
  xixilowcmd
)

# Install targets
install(TARGETS motor_controller_with_enable motor_control_example enable_motor_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
