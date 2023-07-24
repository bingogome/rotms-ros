cmake_minimum_required(VERSION 3.5)
project(rotms_ros_misc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  roslaunch
  roslib
  tf2
  tf2_ros
  rotms_ros_utility
  )

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

roslaunch_add_file_check(launch)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rotms_ros_utility
  DEPENDS YAML_CPP
  )

add_library(${PROJECT_NAME} 
  src/dummy.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC 
  include/rotms_ros_misc
  ../rotms_ros_utility/include/rotms_ros_utility
  ${catkin_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDEDIR} 
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(node_mep_request_save_poses src/nodes/node_mep_request_save_poses.cpp)
target_link_libraries(node_mep_request_save_poses ${PROJECT_NAME})