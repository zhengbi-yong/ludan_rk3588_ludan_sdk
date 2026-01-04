cmake_minimum_required(VERSION 3.10)
project(motor_test2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# ZLG CANFDNET SDK paths
set(ZLG_SDK_ROOT "/home/linaro/ludan_sdk/LINUX_CANFDNET_V1.3.0.5_example")
set(ZLG_INCLUDE_DIRS "${ZLG_SDK_ROOT}/src" "${ZLG_SDK_ROOT}/src/zlgcan")
set(ZLG_LIB_DIR "${ZLG_SDK_ROOT}/lib/linux_x64")

# Include directories
include_directories(${ZLG_INCLUDE_DIRS})

# Add executable
add_executable(motor_test2 motor_test2.cpp)

# Link ZLG CANFDNET library
target_link_libraries(motor_test2
  ${ZLG_LIB_DIR}/libCANFDNET.so
  pthread
)

# Install target
install(TARGETS motor_test2 DESTINATION bin)
