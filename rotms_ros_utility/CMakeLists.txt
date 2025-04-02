cmake_minimum_required(VERSION 3.5)
project(rotms_ros_utility)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  roslib
  rospy
  )

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
  CATKIN_DEPENDS roscpp rospy
  DEPENDS YAML_CPP
)

include_directories(
  include
  ${PYTHON_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
  src/dummy.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include/rotms_ros_utility
  ${catkin_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDEDIR} 
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(node_icp_digitization src/nodes/node_icp_digitization.cpp)
target_link_libraries(node_icp_digitization ${PROJECT_NAME})

catkin_install_python(PROGRAMS
  src/nodes/node_icp.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)