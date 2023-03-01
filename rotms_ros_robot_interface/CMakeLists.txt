cmake_minimum_required(VERSION 3.5)
project(rotms_ros_robot_interface)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  roslib
  roslaunch
  rotms_ros_msgs
  rotms_ros_utility
  )

find_package(Boost REQUIRED COMPONENTS system)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

roslaunch_add_file_check(launch)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rotms_ros_msgs rotms_ros_utility
  DEPENDS YAML_CPP
)

add_library(${PROJECT_NAME} 
  src/kst_servoing.cpp
  src/rotms_ros_robot_interface.cpp

)

target_include_directories(${PROJECT_NAME} PUBLIC
  include/rotms_ros_robot_interface
  ../rotms_ros_utility/include/rotms_ros_utility
  ${catkin_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDEDIR} 
  ${Boost_INCLUDE_DIRS}
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(node_robot src/nodes/node_robot.cpp)
target_link_libraries(node_robot ${PROJECT_NAME})
add_executable(node_robot_end src/nodes/node_robot_end.cpp)
target_link_libraries(node_robot_end ${PROJECT_NAME})