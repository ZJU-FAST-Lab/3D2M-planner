cmake_minimum_required(VERSION 2.8.3)
project(fake_diablo)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  std_msgs
  tf
)
find_package(Eigen3 REQUIRED)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs tf
#  DEPENDS system_lib
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable (fake_diablo src/poscmd_2_odom.cpp )
target_link_libraries(fake_diablo
  ${catkin_LIBRARIES})

  add_executable (fake_model src/fake_model.cpp )
  target_link_libraries(fake_model
    ${catkin_LIBRARIES})
