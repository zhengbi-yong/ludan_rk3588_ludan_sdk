cmake_minimum_required(VERSION 3.16)
project(motor_dds_proj)

# 查找 ROS2 构建工具
find_package(ament_cmake REQUIRED)
# 查找 CycloneDDS 包
find_package(CycloneDDS REQUIRED)

# 设置 C 标准
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 包含生成的头文件目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/generated)

# 添加可执行文件
add_executable(motor_dds_sub src/sub.c
    ${CMAKE_CURRENT_SOURCE_DIR}/generated/rt_lowcmd.c
)

# 链接 CycloneDDS 库
target_link_libraries(motor_dds_sub CycloneDDS::ddsc)

# 设置编译选项
target_compile_options(motor_dds_sub PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

# 安装可执行文件
install(TARGETS motor_dds_sub
  DESTINATION lib/${PROJECT_NAME}
)

# ament_cmake 必须放在最后
ament_package()