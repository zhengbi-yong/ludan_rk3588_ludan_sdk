cmake_minimum_required(VERSION 3.5)
project(dds_topics_monitor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Unitree SDK2
find_package(unitree_sdk2 REQUIRED)

# Add executable
add_executable(dds_topics_monitor dds_topics_monitor.cpp)

# Link libraries
target_link_libraries(dds_topics_monitor unitree_sdk2)

# Install target
install(TARGETS dds_topics_monitor DESTINATION bin)