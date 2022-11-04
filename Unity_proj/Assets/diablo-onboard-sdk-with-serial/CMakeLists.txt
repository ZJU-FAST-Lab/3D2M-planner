cmake_minimum_required(VERSION 3.0.2)
project(diablo_sdk)

add_compile_options(
		-std=c++17
		-Wno-psabi
)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Boost REQUIRED COMPONENTS system thread chrono)

include_directories(
    api/inc
    /usr/local/include/VulcanSerial
    ${Boost_INCLUDE_DIRS}
)

link_libraries(
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_CHRONO_LIBRARY}
    /usr/local/lib/libVulcanSerial.a
    sqlite3
)



add_definitions(-D_LOG_CONSOLE)

## Declare a C++ library
## Source code for OSDK CORE
FILE(GLOB OSDK_LIB_SRCS
        api/src/*.cpp
    )

add_library(
    ${PROJECT_NAME}
    STATIC
    ${OSDK_LIB_SRCS}
)

add_definitions(-D_WITH_ROS)

find_package(
        catkin REQUIRED
        message_generation
        roscpp
        rospy
        std_msgs
        geometry_msgs 
        #roslib
    )

    add_message_files(
        FILES
        OSDK_ACCL.msg
        OSDK_GYRO.msg
        OSDK_LEGMOTORS.msg
        OSDK_POWER.msg
        OSDK_QUATERNION.msg
        OSDK_RC.msg
        OSDK_STATUS.msg
        Diablo_Ctrl.msg
    )

    generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs 
    )

    catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES 
        CATKIN_DEPENDS 
        message_runtime
		# DEPENDS system_lib
    )

    include_directories(
        ${catkin_INCLUDE_DIRS}
    )

    link_libraries(
        ${catkin_LIBRARIES}
    )
    
    add_executable(virtual_rc_example ${OSDK_LIB_SRCS} 
    ${CMAKE_CURRENT_SOURCE_DIR}/example/virtual_rc/main.cpp)
    add_dependencies(virtual_rc_example diablo_sdk_generate_messages_cpp)
    add_compile_options(-l sqlite3)

    add_executable(movement_ctrl_example ${OSDK_LIB_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/example/movement_ctrl/main.cpp)
    add_dependencies(movement_ctrl_example diablo_sdk_generate_messages_cpp)
    add_compile_options(-l sqlite3)

    add_executable(robot_status_example ${OSDK_LIB_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/example/robot_status/main.cpp)
    add_dependencies(robot_status_example diablo_sdk_generate_messages_cpp)
    add_compile_options(-l sqlite3)

    add_executable(diablo_ctrl_example ${OSDK_LIB_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/example/diablo_ctrl/main.cpp)
    add_dependencies(diablo_ctrl_example diablo_sdk_generate_messages_cpp)
    add_compile_options(-l sqlite3)
    



