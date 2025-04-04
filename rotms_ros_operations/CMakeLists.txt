cmake_minimum_required(VERSION 3.5)
project(rotms_ros_operations)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  roslib
  rotms_ros_msgs
  rotms_ros_utility
  )

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rotms_ros_utility
  DEPENDS YAML_CPP
)

add_library(${PROJECT_NAME} 
  src/operations.cpp
  src/operations_toolplan.cpp
  src/operations_registration.cpp
  src/operations_digitization.cpp
  src/operations_robot.cpp
  src/operations_utility.cpp
  src/registration_funcs.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include/rotms_ros_operations
  ../rotms_ros_utility/include/rotms_ros_utility
  ${catkin_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDEDIR} 
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(test_node_operations src/nodes/test_node_operations.cpp)
target_link_libraries(test_node_operations ${PROJECT_NAME})
