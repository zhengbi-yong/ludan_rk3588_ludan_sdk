cmake_minimum_required(VERSION 3.10)
project(motor_feedback)

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
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ZLG CANFDNET SDK paths (matching motor_controller_with_enable)
set(ZLG_SDK_ROOT "/home/linaro/ludan_sdk/LINUX_CANFDNET_V1.3.0.5_example")
set(ZLG_INCLUDE_DIR "${ZLG_SDK_ROOT}/src/zlgcan")
set(ZLG_SRC_DIR "${ZLG_SDK_ROOT}/src")
set(ZLG_LIB_DIR "${ZLG_SDK_ROOT}/lib/linux_x64")

# Include directories (must include both src and src/zlgcan for CANFDNET.h and zlgcan.h)
include_directories(
  ${ZLG_SRC_DIR}
  ${ZLG_INCLUDE_DIR}
)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorFeedback.msg"
  DEPENDENCIES std_msgs
)

# Motor Feedback Publisher
add_executable(motor_feedback_publisher src/motor_feedback_publisher.cpp)

ament_target_dependencies(motor_feedback_publisher
  rclcpp
  std_msgs
)

# Link ZLG CANFDNET library
target_link_libraries(motor_feedback_publisher
  ${ZLG_LIB_DIR}/libCANFDNET.so
  pthread
)

# Set RPATH so the executable can find libCANFDNET.so at runtime
set_target_properties(motor_feedback_publisher PROPERTIES
  INSTALL_RPATH "${ZLG_LIB_DIR}"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# Link generated interfaces
target_link_libraries(motor_feedback_publisher
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Motor Feedback Subscriber
add_executable(motor_feedback_subscriber src/motor_feedback_subscriber.cpp)

ament_target_dependencies(motor_feedback_subscriber
  rclcpp
  std_msgs
)

target_link_libraries(motor_feedback_subscriber
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Install targets
install(TARGETS
  motor_feedback_publisher
  motor_feedback_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
