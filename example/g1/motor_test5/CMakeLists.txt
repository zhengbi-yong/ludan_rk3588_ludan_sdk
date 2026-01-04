cmake_minimum_required(VERSION 3.5)
project(motor_test5 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# ZLG CANFDNET SDK paths
set(ZCAN_SDK_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../LINUX_CANFDNET_V1.3.0.5_example)
set(ZCAN_INCLUDE_DIR ${ZCAN_SDK_ROOT}/src)
set(ZCAN_LIB_DIR ${ZCAN_SDK_ROOT}/lib/linux_x64)

include_directories(${ZCAN_INCLUDE_DIR})

set(SOURCES src/motor_test5.cpp)

add_executable(motor_test5 ${SOURCES})
target_link_libraries(motor_test5 ${ZCAN_LIB_DIR}/libCANFDNET.a pthread)

install(TARGETS motor_test5 DESTINATION bin)

message(STATUS "Motor Test5: Send 8A 3C 7F F0 10 33 37 FF (3 times)")
