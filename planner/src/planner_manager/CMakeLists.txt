cmake_minimum_required(VERSION 3.0.2)
project(planner_manager)
set(CMAKE_BUILD_TYPE "RELEASE")


find_package(catkin REQUIRED COMPONENTS
  map_manager
  planner_algorithm
  mpc

  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs

  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

add_message_files(FILES
  Polynome.msg
	DVHcontrol.msg
)
generate_messages(DEPENDENCIES
	std_msgs
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES planner_manager
  CATKIN_DEPENDS mpc map_manager planner_algorithm roscpp rospy std_msgs sensor_msgs geometry_msgs
#  DEPENDS system_lib
  message_runtime
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_executable(scf_planner_node src/planner_manager_node.cpp 
                                src/planner_manager.cpp )

target_link_libraries(scf_planner_node
    ${catkin_LIBRARIES}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_LIBRARIES}
    map_manager_lib
    planner_algorithm_lib
)

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
