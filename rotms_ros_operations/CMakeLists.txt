cmake_minimum_required(VERSION 3.5)
project(rotms_ros_operations)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  roslib
  )

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp 
  DEPENDS YAML_CPP
)

add_library(${PROJECT_NAME} 
  src/rotms_operations.cpp
  src/registration_funcs.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include/rotms_ros_operations
  ${catkin_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDEDIR} 
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(test_node_operations src/nodes/test_node_operations.cpp)
target_link_libraries(test_node_operations ${PROJECT_NAME})
