cmake_minimum_required(VERSION 3.5)
project(rotms_ros_comm_relay)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  roslaunch
  roslib
  rotms_ros_utility
  )

find_package(Boost REQUIRED COMPONENTS system)

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
  include/rotms_ros_comm_relay
  ../rotms_ros_utility/include/rotms_ros_utility
  ${catkin_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDEDIR} 
  ${Boost_INCLUDE_DIRS}
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(node_comm_relay src/nodes/node_comm_relay.cpp)
target_link_libraries(node_comm_relay ${PROJECT_NAME})
