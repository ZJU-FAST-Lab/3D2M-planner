cmake_minimum_required(VERSION 3.0.2)
project(globalmap_gene)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)


find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES globalmap_gene
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}

)

add_executable(globalmap_gene_node src/globalmap_gene_node.cpp)

target_link_libraries(globalmap_gene_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)


