cmake_minimum_required(VERSION 3.5)
project(rotms_ros_comm_decode)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  roslib
  roslaunch
  rotms_ros_utility
  )

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

roslaunch_add_file_check(launch)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rotms_ros_utility
  DEPENDS YAML_CPP
)

add_library(${PROJECT_NAME} 
  src/decode_node.cpp
  src/function_map_robctrl.cpp
  src/function_map_medimg.cpp
  src/function_map_targetviz.cpp
  src/function_map_mep.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include/rotms_ros_comm_decode
  ../rotms_ros_utility/include/rotms_ros_utility
  ${catkin_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDEDIR} 
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(node_decode_medimg src/nodes/node_decode_medimg.cpp)
target_link_libraries(node_decode_medimg ${PROJECT_NAME})
add_executable(node_decode_robctrl src/nodes/node_decode_robctrl.cpp)
target_link_libraries(node_decode_robctrl ${PROJECT_NAME})
add_executable(node_decode_targetviz src/nodes/node_decode_targetviz.cpp)
target_link_libraries(node_decode_targetviz ${PROJECT_NAME})
add_executable(node_decode_mep src/nodes/node_decode_mep.cpp)
target_link_libraries(node_decode_mep ${PROJECT_NAME})