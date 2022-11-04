cmake_minimum_required(VERSION 2.8.3)
project(read_pcd)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pcl_conversions
  rviz_plugin_tutorials
)
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(OpenCV REQUIRED)

message(STATUS "Opnecv ;ibrary status: ")
message(STATUS "> version: ${OpenCV_VERSION} ")
message(STATUS "libraries: ${OpenCV_LIBS} ")
message(STATUS "> include: ${OpenCV_INCLUDE_DIRS}  ")

catkin_package(
 CATKIN_DEPENDS rviz_plugin_tutorials
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  #${OpenCV_INCLUDE_DIRS}
  )
message("1111111111111111111111111111111111")
message(${PCL_INCLUDE_DIRS})
message("1111111111111111111111111111111111")
message(${PCL_LIBRARIES})
  
add_executable (read_pcd src/read_pcd.cpp )
target_link_libraries(read_pcd
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  #${OpenCV_LIBS}
  )
