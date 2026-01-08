cmake_minimum_required(VERSION 3.5)
project(cantoudp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# CANFDNET SDK paths
set(CANFDNET_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../LINUX_CANFDNET_V1.3.0.5_example)
set(CANFDNET_SRC_DIR ${CANFDNET_ROOT}/src)

# Include directories
include_directories(${CANFDNET_SRC_DIR})
include_directories(${CANFDNET_SRC_DIR}/zlgcan)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Find yaml-cpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# Source files
set(SOURCES
    src/cantoudp.cpp
    src/motor_config.cpp
)

# CANFDNET source files (compile directly to avoid ABI issues)
set(CANFDNET_SOURCES
    ${CANFDNET_SRC_DIR}/CANFDNET.cpp
    ${CANFDNET_SRC_DIR}/Device.cpp
    ${CANFDNET_SRC_DIR}/DeviceMgr.cpp
    ${CANFDNET_SRC_DIR}/NetPacket.cpp
    ${CANFDNET_SRC_DIR}/log.cpp
    ${CANFDNET_SRC_DIR}/common.cpp
    ${CANFDNET_SRC_DIR}/platform/linux/network.cpp
    ${CANFDNET_SRC_DIR}/service/TcpClient.cpp
    ${CANFDNET_SRC_DIR}/service/TcpServer.cpp
    ${CANFDNET_SRC_DIR}/service/UdpService.cpp
    ${CANFDNET_SRC_DIR}/cJSON/cJSON.c
)

# Zlib source files (needed for CANFDNET)
file(GLOB ZLIB_SOURCES ${CANFDNET_SRC_DIR}/3rd/zlib/*.c)

# Create executable
add_executable(cantoudp ${SOURCES} ${CANFDNET_SOURCES} ${ZLIB_SOURCES})

# Link libraries
target_link_libraries(cantoudp pthread ${YAML_CPP_LIBRARIES})
target_include_directories(cantoudp PRIVATE ${YAML_CPP_INCLUDE_DIRS})

# Install target
install(TARGETS cantoudp DESTINATION bin)
