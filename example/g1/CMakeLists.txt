add_executable(g1_loco_client high_level/g1_loco_client_example.cpp)
target_link_libraries(g1_loco_client unitree_sdk2)

add_executable(g1_arm5_sdk_dds_example high_level/g1_arm5_sdk_dds_example.cpp)
target_link_libraries(g1_arm5_sdk_dds_example unitree_sdk2)

add_executable(g1_arm7_sdk_dds_example high_level/g1_arm7_sdk_dds_example.cpp)
target_link_libraries(g1_arm7_sdk_dds_example unitree_sdk2)

add_executable(g1_ankle_swing_example low_level/g1_ankle_swing_example.cpp)
target_link_libraries(g1_ankle_swing_example unitree_sdk2)

# Debug versions for testing without robot
add_executable(g1_ankle_swing_example_debug low_level/g1_ankle_swing_example_debug.cpp)
target_link_libraries(g1_ankle_swing_example_debug unitree_sdk2)

add_executable(command_monitor low_level/command_monitor.cpp)
target_link_libraries(command_monitor unitree_sdk2)

add_executable(state_simulator low_level/state_simulator.cpp)
target_link_libraries(state_simulator unitree_sdk2)

add_executable(motor_controller low_level/motor_controller.cpp)
target_link_libraries(motor_controller unitree_sdk2)

# CAN library dependencies for motor_controller
if(UNIX AND NOT APPLE)
    target_link_libraries(motor_controller pthread)
endif()

add_executable(demo_commands low_level/demo_commands.cpp)
target_link_libraries(demo_commands unitree_sdk2)

add_executable(deploy_test low_level/deploy_test.cpp)
target_link_libraries(deploy_test unitree_sdk2)

# Network distributed deployment programs
add_executable(deploy_test_jetson low_level/deploy_test_jetson.cpp)
target_link_libraries(deploy_test_jetson ${CMAKE_THREAD_LIBS_INIT})

add_executable(deploy_test_rk3588 low_level/deploy_test_rk3588.cpp)
target_link_libraries(deploy_test_rk3588 unitree_sdk2 ${CMAKE_THREAD_LIBS_INIT})

add_executable(g1_audio_client_example audio/g1_audio_client_example.cpp)
target_link_libraries(g1_audio_client_example unitree_sdk2)

add_executable(g1_dex3_example dex3/g1_dex3_example.cpp)
target_link_libraries(g1_dex3_example unitree_sdk2)

find_package(Boost COMPONENTS program_options)
if(Boost_FOUND)
    add_executable(g1_termination low_level/terminations.cpp)
    target_link_libraries(g1_termination unitree_sdk2 Boost::program_options)

    add_executable(g1_arm_action_example high_level/g1_arm_action_example.cpp)
    target_link_libraries(g1_arm_action_example unitree_sdk2 Boost::program_options)
endif()

find_package(yaml-cpp QUIET)
if(yaml-cpp_FOUND)
    if (${yaml-cpp_VERSION} VERSION_GREATER_EQUAL "0.6")
        message(STATUS "Found yaml-cpp version ${yaml-cpp_VERSION}")
        add_executable(g1_dual_arm_example low_level/g1_dual_arm_example.cpp)
        target_link_libraries(g1_dual_arm_example PRIVATE unitree_sdk2 yaml-cpp)
        target_compile_definitions(g1_dual_arm_example PUBLIC BLIB_DIR="${CMAKE_CURRENT_SOURCE_DIR}/low_level/behavior_lib/")
    else()
        message(STATUS "yaml-cpp version ${yaml-cpp_VERSION} is too old, skipping build of g1_dual_arm_example.")
    endif()
endif()

add_executable(g1_agv_client_example agv/g1_agv_client_example.cpp)
target_link_libraries(g1_agv_client_example unitree_sdk2)
