cmake_minimum_required(VERSION 3.5)
project(rotms_ros_kinematics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  roslaunch
  roslib
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
  src/dummy.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC 
  include/rotms_ros_kinematics
  ${catkin_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDEDIR} 
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(node_calibration_data src/nodes/node_calibration_data.cpp)
target_link_libraries(node_calibration_data ${PROJECT_NAME})
