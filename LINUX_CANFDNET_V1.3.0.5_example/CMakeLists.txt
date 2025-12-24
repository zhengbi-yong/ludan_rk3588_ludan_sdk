cmake_minimum_required(VERSION 2.6)
project(CANFDNET)

include_directories(./src) 
include_directories(./src/zlgcan)

set(CMAKE_CXX_STANDARD 11)

add_definitions(-D _CRT_SECURE_NO_WARNINGS)

IF(UNIX) 
    set(PLATFORM_TAG linux)
    find_package(Threads)
ELSE() 
    set(PLATFORM_TAG windows)
ENDIF()

set(SRC
    ${PROJECT_SOURCE_DIR}/src/CANFDNET.cpp
    ${PROJECT_SOURCE_DIR}/src/Device.cpp
    ${PROJECT_SOURCE_DIR}/src/DeviceMgr.cpp
    ${PROJECT_SOURCE_DIR}/src/NetPacket.cpp
    ${PROJECT_SOURCE_DIR}/src/CANFDNET.def
    ${PROJECT_SOURCE_DIR}/src/log.cpp
    ${PROJECT_SOURCE_DIR}/src/common.cpp
    ${PROJECT_SOURCE_DIR}/src/platform/${PLATFORM_TAG}/network.cpp  
    ${PROJECT_SOURCE_DIR}/src/service/TcpClient.cpp
    ${PROJECT_SOURCE_DIR}/src/service/TcpServer.cpp
    ${PROJECT_SOURCE_DIR}/src/service/UdpService.cpp
    ${PROJECT_SOURCE_DIR}/src/cJSON/cJSON.c
)

file(GLOB_RECURSE HEADERS src/*.h src/*.hpp)
file(GLOB_RECURSE ZLIB src/3rd/zlib/*.h src/3rd/zlib/*.c)

IF("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib/${PLATFORM_TAG}_x64/)
ELSE()
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib/${PLATFORM_TAG}/)
ENDIF()

#CANFDNET 
set(LIB_NAME "CANFDNET")

IF(WIN32)
source_group("Include" FILES ${HEADERS})
source_group("zlib" FILES ${ZLIB})
set(SRC_TCP_UDP ${SRC} ${ZLIB} ${HEADERS} ${PROJECT_SOURCE_DIR}/src/${LIB_NAME}.rc)
ELSE()
set(SRC_TCP_UDP ${SRC} ${ZLIB})
ENDIF()

add_library(${LIB_NAME} SHARED ${SRC_TCP_UDP})

IF(WIN32)
target_link_libraries(${LIB_NAME} Winmm.lib)
ENDIF()

IF(UNIX)
    add_library(CANFDNET_TCP_STATIC STATIC ${SRC_TCP_UDP}) 

    # static and shared lib have the same name
    set_target_properties(CANFDNET_TCP_STATIC PROPERTIES OUTPUT_NAME ${LIB_NAME})
    set_target_properties(CANFDNET_TCP_STATIC PROPERTIES CLEAN_DIRECT_OUTPUT 1)
    set_target_properties(${LIB_NAME} PROPERTIES CLEAN_DIRECT_OUTPUT 1)
    
    target_link_libraries(${LIB_NAME} ${CMAKE_THREAD_LIBS_INIT})
ENDIF()

IF(1)
# test lib
set(EXECUTABLE_OUTPUT_PATH ${LIBRARY_OUTPUT_PATH})
add_executable(canfdnet_test ${PROJECT_SOURCE_DIR}/test/test.cpp ${PROJECT_SOURCE_DIR}/src/common.cpp ${PROJECT_SOURCE_DIR}/src/log.cpp)
target_link_libraries(canfdnet_test ${LIB_NAME})
ENDIF()
