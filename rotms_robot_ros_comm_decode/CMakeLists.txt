cmake_minimum_required(VERSION 3.0.2)
project(rotms_robot_ros_comm_decode)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  roslib
  )

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
  DEPENDS YAML_CPP
)

include_directories(include ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDEDIR} )

add_executable(node_decode_robctrl src/nodes/node_decode_robctrl.cpp)
target_link_libraries(node_decode_robctrl ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})