cmake_minimum_required(VERSION 3.10)
project(CANFDNET CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files for main CANFDNET program
set(SOURCES
    CANFDNET.cpp
    stdafx.cpp
)

# Header files
set(HEADERS
    canframe.h
    config.h
    stdafx.h
    linux_compat.h
    targetver.h
    typedef.h
    zlgcan.h
)

# Build main CANFDNET executable
add_executable(canfdnet ${SOURCES})
target_include_directories(canfdnet PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(canfdnet pthread)
target_compile_definitions(canfdnet PRIVATE LINUX_BUILD)

# Build network CAN test program (direct TCP, no zlgcan_linux)
add_executable(can_test_network can_test_network.cpp)
target_include_directories(can_test_network PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(can_test_network pthread)
target_compile_definitions(can_test_network PRIVATE LINUX_BUILD)
