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

# Check if ROS2 environment is available
if(DEFINED ENV{ROS_DISTRO})
    # ROS2 version with xixilowcmd support
    find_package(rclcpp QUIET)
    if(rclcpp_FOUND)
        # Try to find xixilowcmd in controller_ws
        set(ROS2_WORKSPACE "/home/linaro/controller_ws")
        if(EXISTS "${ROS2_WORKSPACE}/install/xixilowcmd")
            add_executable(motor_controller_ros2 low_level/motor_controller.cpp)

            # Set include directories
            target_include_directories(motor_controller_ros2 PRIVATE
                "${ROS2_WORKSPACE}/install/xixilowcmd/include"
                "${ROS2_WORKSPACE}/install/xixilowcmd/local/include"
                "${ROS2_WORKSPACE}/build/xixilowcmd/rosidl_generator_cpp"
                "/opt/ros/humble/include"
            )

            # Set RPATH for runtime library search
            set_target_properties(motor_controller_ros2 PROPERTIES
                INSTALL_RPATH "${ROS2_WORKSPACE}/install/xixilowcmd/lib"
                BUILD_WITH_INSTALL_RPATH TRUE
            )

            # Link libraries - include ROS2 libraries and xixilowcmd
            target_link_libraries(motor_controller_ros2
                rclcpp::rclcpp
                ${ROS2_WORKSPACE}/install/xixilowcmd/lib/libxixilowcmd__rosidl_typesupport_cpp.so
                ${ROS2_WORKSPACE}/install/xixilowcmd/lib/libxixilowcmd__rosidl_generator_c.so
                pthread
            )

            # Add ROS2 include directories
            target_include_directories(motor_controller_ros2 PRIVATE
                ${rclcpp_INCLUDE_DIRS}
            )

            # Compile with ROS2 support
            target_compile_definitions(motor_controller_ros2 PRIVATE USE_ROS2=1)

            # Rename the binary to motor_controller
            set_target_properties(motor_controller_ros2 PROPERTIES OUTPUT_NAME motor_controller)

            # Build motor_controller_with_enable with ROS2 and xixilowcmd support
            add_executable(motor_controller_with_enable_ros2 low_level/motor_controller_with_enable.cpp)

            # ZLG CANFDNET SDK paths (using newly compiled ARM64 library)
            set(ZLG_SDK_DIR "/home/linaro/ludan_sdk/LINUX_CANFDNET_V1.3.0.5_example")

            # Set include directories
            target_include_directories(motor_controller_with_enable_ros2 PRIVATE
                "${ROS2_WORKSPACE}/install/xixilowcmd/include"
                "${ROS2_WORKSPACE}/install/xixilowcmd/local/include"
                "${ROS2_WORKSPACE}/build/xixilowcmd/rosidl_generator_cpp"
                "/opt/ros/humble/include"
                "${ZLG_SDK_DIR}/src"
                "${ZLG_SDK_DIR}/src/zlgcan"
            )

            # Set RPATH for runtime library search
            set_target_properties(motor_controller_with_enable_ros2 PROPERTIES
                INSTALL_RPATH "${ROS2_WORKSPACE}/install/xixilowcmd/lib;${ZLG_SDK_DIR}/lib/linux_x64"
                BUILD_WITH_INSTALL_RPATH TRUE
            )

            # Link libraries - include ROS2 libraries, xixilowcmd and ZLG CANFDNET
            target_link_libraries(motor_controller_with_enable_ros2
                rclcpp::rclcpp
                ${ROS2_WORKSPACE}/install/xixilowcmd/lib/libxixilowcmd__rosidl_typesupport_cpp.so
                ${ROS2_WORKSPACE}/install/xixilowcmd/lib/libxixilowcmd__rosidl_generator_c.so
                "${ZLG_SDK_DIR}/lib/linux_x64/libCANFDNET.so"
                pthread
            )

            # Add ROS2 include directories
            target_include_directories(motor_controller_with_enable_ros2 PRIVATE
                ${rclcpp_INCLUDE_DIRS}
            )

            # Compile with ROS2 support
            target_compile_definitions(motor_controller_with_enable_ros2 PRIVATE USE_ROS2=1)

            # Rename the binary to motor_controller_with_enable
            set_target_properties(motor_controller_with_enable_ros2 PROPERTIES OUTPUT_NAME motor_controller_with_enable)

            # Build zlg_stability_test with ZLG CANFDNET SDK
            add_executable(zlg_stability_test low_level/zlg_stability_test.cpp)

            # Set include directories
            target_include_directories(zlg_stability_test PRIVATE
                "${ZLG_SDK_DIR}/src"
                "${ZLG_SDK_DIR}/src/zlgcan"
            )

            # Set RPATH for runtime library search
            set_target_properties(zlg_stability_test PROPERTIES
                INSTALL_RPATH "${ZLG_SDK_DIR}/lib/linux_x64"
                BUILD_WITH_INSTALL_RPATH TRUE
            )

            # Link libraries - ZLG CANFDNET
            target_link_libraries(zlg_stability_test
                "${ZLG_SDK_DIR}/lib/linux_x64/libCANFDNET.so"
                pthread
            )

            message(STATUS "Building zlg_stability_test with ZLG CANFDNET support")

            # Build zlg_5min_test with ZLG CANFDNET SDK
            add_executable(zlg_5min_test low_level/zlg_5min_test.cpp)

            # Set include directories
            target_include_directories(zlg_5min_test PRIVATE
                "${ZLG_SDK_DIR}/src"
                "${ZLG_SDK_DIR}/src/zlgcan"
            )

            # Set RPATH for runtime library search
            set_target_properties(zlg_5min_test PROPERTIES
                INSTALL_RPATH "${ZLG_SDK_DIR}/lib/linux_x64"
                BUILD_WITH_INSTALL_RPATH TRUE
            )

            # Link libraries - ZLG CANFDNET
            target_link_libraries(zlg_5min_test
                "${ZLG_SDK_DIR}/lib/linux_x64/libCANFDNET.so"
                pthread
            )

            message(STATUS "Building zlg_5min_test with ZLG CANFDNET support")

            # Build zlg_5min_decode_test with ZLG CANFDNET SDK
            add_executable(zlg_5min_decode_test low_level/zlg_5min_decode_test.cpp)

            # Set include directories
            target_include_directories(zlg_5min_decode_test PRIVATE
                "${ZLG_SDK_DIR}/src"
                "${ZLG_SDK_DIR}/src/zlgcan"
            )

            # Set RPATH for runtime library search
            set_target_properties(zlg_5min_decode_test PROPERTIES
                INSTALL_RPATH "${ZLG_SDK_DIR}/lib/linux_x64"
                BUILD_WITH_INSTALL_RPATH TRUE
            )

            # Link libraries - ZLG CANFDNET
            target_link_libraries(zlg_5min_decode_test
                "${ZLG_SDK_DIR}/lib/linux_x64/libCANFDNET.so"
                pthread
            )

            message(STATUS "Building zlg_5min_decode_test with ZLG CANFDNET support")

            # Build motor_controller_with_enable_zhengbi with ROS2 and xixilowcmd support
            add_executable(motor_controller_with_enable_zhengbi_ros2 low_level/motor_controller_with_enable_zhengbi.cpp)

            # Set include directories
            target_include_directories(motor_controller_with_enable_zhengbi_ros2 PRIVATE
                "${ROS2_WORKSPACE}/install/xixilowcmd/include"
                "${ROS2_WORKSPACE}/install/xixilowcmd/local/include"
                "${ROS2_WORKSPACE}/build/xixilowcmd/rosidl_generator_cpp"
                "/opt/ros/humble/include"
            )

            # Set RPATH for runtime library search
            set_target_properties(motor_controller_with_enable_zhengbi_ros2 PROPERTIES
                INSTALL_RPATH "${ROS2_WORKSPACE}/install/xixilowcmd/lib"
                BUILD_WITH_INSTALL_RPATH TRUE
            )

            # Link libraries - include ROS2 libraries and xixilowcmd
            target_link_libraries(motor_controller_with_enable_zhengbi_ros2
                rclcpp::rclcpp
                ${ROS2_WORKSPACE}/install/xixilowcmd/lib/libxixilowcmd__rosidl_typesupport_cpp.so
                ${ROS2_WORKSPACE}/install/xixilowcmd/lib/libxixilowcmd__rosidl_generator_c.so
                pthread
            )

            # Add ROS2 include directories
            target_include_directories(motor_controller_with_enable_zhengbi_ros2 PRIVATE
                ${rclcpp_INCLUDE_DIRS}
            )

            # Compile with ROS2 support
            target_compile_definitions(motor_controller_with_enable_zhengbi_ros2 PRIVATE USE_ROS2=1)

            # Rename the binary to motor_controller_with_enable_zhengbi
            set_target_properties(motor_controller_with_enable_zhengbi_ros2 PROPERTIES OUTPUT_NAME motor_controller_with_enable_zhengbi)

            message(STATUS "Building motor_controller with ROS2 and xixilowcmd support")
            message(STATUS "Building motor_controller_with_enable with ROS2, xixilowcmd and enable functionality")
            message(STATUS "Building motor_controller_with_enable_zhengbi with ROS2, xixilowcmd and enable functionality")
        else()
            # Fallback to original version
            add_executable(motor_controller low_level/motor_controller.cpp)
            target_link_libraries(motor_controller unitree_sdk2)

            # Fallback motor_controller_with_enable
            add_executable(motor_controller_with_enable low_level/motor_controller_with_enable.cpp)
            target_link_libraries(motor_controller_with_enable unitree_sdk2)

            # Fallback motor_controller_with_enable_zhengbi
            add_executable(motor_controller_with_enable_zhengbi low_level/motor_controller_with_enable_zhengbi.cpp)
            target_link_libraries(motor_controller_with_enable_zhengbi unitree_sdk2)

            message(STATUS "Controller workspace not found, building motor_controller with original SDK2 support")
            message(STATUS "Building motor_controller_with_enable with original SDK2 support")
            message(STATUS "Building motor_controller_with_enable_zhengbi with original SDK2 support")
        endif()
    else()
        # Fallback to original version
        add_executable(motor_controller low_level/motor_controller.cpp)
        target_link_libraries(motor_controller unitree_sdk2)

        # Fallback motor_controller_with_enable
        add_executable(motor_controller_with_enable low_level/motor_controller_with_enable.cpp)
        target_link_libraries(motor_controller_with_enable unitree_sdk2)

        # Fallback motor_controller_with_enable_zhengbi
        add_executable(motor_controller_with_enable_zhengbi low_level/motor_controller_with_enable_zhengbi.cpp)
        target_link_libraries(motor_controller_with_enable_zhengbi unitree_sdk2)

        message(STATUS "rclcpp not found, building motor_controller with original SDK2 support")
        message(STATUS "Building motor_controller_with_enable with original SDK2 support")
        message(STATUS "Building motor_controller_with_enable_zhengbi with original SDK2 support")
    endif()
else()
    # No ROS2 environment - use original version
    add_executable(motor_controller low_level/motor_controller.cpp)
    target_link_libraries(motor_controller unitree_sdk2)

    # No ROS2 environment - motor_controller_with_enable version
    add_executable(motor_controller_with_enable low_level/motor_controller_with_enable.cpp)
    target_link_libraries(motor_controller_with_enable unitree_sdk2)

    # No ROS2 environment - motor_controller_with_enable_zhengbi version
    add_executable(motor_controller_with_enable_zhengbi low_level/motor_controller_with_enable_zhengbi.cpp)
    target_link_libraries(motor_controller_with_enable_zhengbi unitree_sdk2)

    message(STATUS "No ROS2 environment, building motor_controller with original SDK2 support")
    message(STATUS "No ROS2 environment, building motor_controller_with_enable with original SDK2 support")
    message(STATUS "No ROS2 environment, building motor_controller_with_enable_zhengbi with original SDK2 support")
endif()

# Additional library dependencies for all versions
if(UNIX AND NOT APPLE)
    if(TARGET motor_controller)
        target_link_libraries(motor_controller pthread)
    endif()
    if(TARGET motor_controller_ros2)
        target_link_libraries(motor_controller_ros2 pthread)
    endif()
    if(TARGET motor_controller_with_enable)
        target_link_libraries(motor_controller_with_enable pthread)
    endif()
    if(TARGET motor_controller_with_enable_ros2)
        target_link_libraries(motor_controller_with_enable_ros2 pthread)
    endif()
    if(TARGET motor_controller_with_enable_zhengbi)
        target_link_libraries(motor_controller_with_enable_zhengbi pthread)
    endif()
    if(TARGET motor_controller_with_enable_zhengbi_ros2)
        target_link_libraries(motor_controller_with_enable_zhengbi_ros2 pthread)
    endif()
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
