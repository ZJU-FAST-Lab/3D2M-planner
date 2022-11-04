cmake_minimum_required(VERSION 3.0.2)
project(planner_algorithm)
set(CMAKE_BUILD_TYPE "RELEASE")

set(ENABLE_CUDA false)

find_package(catkin REQUIRED COMPONENTS
  map_manager

  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES planner_algorithm
 CATKIN_DEPENDS map_manager geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)


add_library(planner_algorithm_lib
src/front_end_Astar.cpp
src/back_end_optimizer.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)


if(ENABLE_CUDA)
  find_package(CUDA REQUIRED)
  CUDA_ADD_LIBRARY( planner_algorithm_gpu_lib
        src/back_end_optimizer_cuda.cu
        OPTIONS  -rdc=true 
        -Xcompiler '-fPIC'
    )


else(ENABLE_CUDA)




endif(ENABLE_CUDA)

