cmake_minimum_required(VERSION 3.5)
project(rotms_ros_state)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  roslib
  roslaunch
  rotms_ros_operations
  rotms_ros_msgs
  rotms_ros_universal_utility
  )

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

roslaunch_add_file_check(launch)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rotms_ros_universal_utility
  DEPENDS YAML_CPP
)

add_library(${PROJECT_NAME} 
  src/state_machine.cpp
  src/flag_machine.cpp
  src/flag_machine_tool.cpp
  src/flag_machine_tms.cpp
  src/state_machine_tool.cpp
  src/state_machine_tms.cpp
  src/state_machine_tool_states.cpp
  src/state_machine_tms_states.cpp
  ../rotms_ros_operations/src/operations_tms.cpp
  ../rotms_ros_operations/src/operations_tool.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include/rotms_ros_state
  ../rotms_ros_operations/include/rotms_ros_operations
  ../rotms_ros_universal_utility/include/rotms_ros_universal_utility
  ${catkin_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDEDIR} 
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(test_node_state src/nodes/test_node_state.cpp)
target_link_libraries(test_node_state ${PROJECT_NAME})
