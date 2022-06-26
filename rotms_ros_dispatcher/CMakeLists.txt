cmake_minimum_required(VERSION 3.5)
project(rotms_ros_dispatcher)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  roslib
  roslaunch
  rotms_ros_state
  )

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

roslaunch_add_file_check(launch)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rotms_ros_state
  DEPENDS YAML_CPP
)

add_library(${PROJECT_NAME} 
  src/rotms_dispatcher.cpp
  ../rotms_ros_state/src/flag_machine.cpp
  ../rotms_ros_state/src/state_machine.cpp
  ../rotms_ros_state/src/state_machine_states.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include/rotms_ros_dispatcher
  ../rotms_ros_state/include/rotms_ros_state
  ${catkin_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDEDIR} 
  ${Boost_INCLUDE_DIRS}
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(node_dispatcher src/nodes/node_dispatcher.cpp)
target_link_libraries(node_dispatcher ${PROJECT_NAME})
