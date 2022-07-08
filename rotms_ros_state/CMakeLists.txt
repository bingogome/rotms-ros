cmake_minimum_required(VERSION 3.5)
project(rotms_ros_state)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  roslib
  roslaunch
  rotms_ros_operations
  rotms_ros_msgs
  )

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

roslaunch_add_file_check(launch)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS YAML_CPP
)

add_library(${PROJECT_NAME} 
  src/state_machine.cpp
  src/state_machine_states.cpp
  src/flag_machine.cpp
  ../rotms_ros_operations/src/rotms_operations.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include/rotms_ros_state
  ../rotms_ros_operations/include/rotms_ros_operations
  ${catkin_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDEDIR} 
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(test_node_state src/nodes/test_node_state.cpp)
target_link_libraries(test_node_state ${PROJECT_NAME})
