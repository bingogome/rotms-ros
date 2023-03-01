cmake_minimum_required(VERSION 3.5)
project(rotms_ros_kinematics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  roslaunch
  roslib
  tf2
  rotms_ros_msgs
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
  src/transform_conversions.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC 
  include/rotms_ros_kinematics
  ../rotms_ros_utility/include/rotms_ros_utility
  ${catkin_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDEDIR} 
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(node_calibration_data src/nodes/node_calibration_data.cpp)
target_link_libraries(node_calibration_data ${PROJECT_NAME})

add_executable(node_opttracker_tr_bodyref_ptrtip src/nodes/node_opttracker_tr_bodyref_ptrtip.cpp)
target_link_libraries(node_opttracker_tr_bodyref_ptrtip ${PROJECT_NAME})

add_executable(node_robot_eff src/nodes/node_robot_eff.cpp)
target_link_libraries(node_robot_eff ${PROJECT_NAME})

add_executable(node_viz_tr_body_tool src/nodes/node_viz_tr_body_tool.cpp)
target_link_libraries(node_viz_tr_body_tool ${PROJECT_NAME})

add_executable(node_TRE_t_body_ptrtip src/nodes/node_TRE_t_body_ptrtip.cpp)
target_link_libraries(node_TRE_t_body_ptrtip ${PROJECT_NAME})