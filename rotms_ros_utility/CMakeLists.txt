cmake_minimum_required(VERSION 3.5)
project(rotms_ros_utility)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  roslib
  rospy
  )

find_package(PythonLibs REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy
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
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  src/nodes/node_icp.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)