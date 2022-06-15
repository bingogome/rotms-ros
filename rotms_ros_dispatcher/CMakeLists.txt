cmake_minimum_required(VERSION 3.5)
project(rotms_ros_dispatcher)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  roslib
  roslaunch
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
  src/registration_funcs.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include/rotms_ros_dispatcher
  ${catkin_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDEDIR} 
  ${Boost_INCLUDE_DIRS}
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(node src/nodes/node.cpp)
target_link_libraries(node ${PROJECT_NAME})
