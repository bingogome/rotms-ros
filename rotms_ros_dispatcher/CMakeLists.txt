cmake_minimum_required(VERSION 3.5)
project(rotms_ros_dispatcher)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  roslib
  roslaunch
  rotms_ros_state
  rotms_ros_operations
  rotms_ros_msgs
  rotms_ros_universal_utility
  )

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

roslaunch_add_file_check(launch)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rotms_ros_state rotms_ros_msgs rotms_ros_universal_utility
  DEPENDS YAML_CPP
)

add_library(${PROJECT_NAME} 
  src/rotms_dispatcher.cpp
  src/dispatcher_utility.cpp

  ../rotms_ros_state/src/flag_machine.cpp
  ../rotms_ros_state/src/state_machine.cpp

  ../rotms_ros_operations/src/operations_registration.cpp
  ../rotms_ros_state/src/flag_machine_registration.cpp
  ../rotms_ros_state/src/state_machine_registration.cpp
  ../rotms_ros_state/src/state_machine_registration_states.cpp

  ../rotms_ros_operations/src/operations_digitization.cpp
  ../rotms_ros_state/src/flag_machine_digitization.cpp
  ../rotms_ros_state/src/state_machine_digitization.cpp
  ../rotms_ros_state/src/state_machine_digitization_states.cpp

  ../rotms_ros_operations/src/operations_toolplan.cpp
  ../rotms_ros_state/src/state_machine_toolplan.cpp
  ../rotms_ros_state/src/flag_machine_toolplan.cpp
  ../rotms_ros_state/src/state_machine_toolplan_states.cpp

  ../rotms_ros_operations/src/operations_robot.cpp
  ../rotms_ros_state/src/state_machine_robot.cpp
  ../rotms_ros_state/src/flag_machine_robot.cpp
  ../rotms_ros_state/src/state_machine_robot_states.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include/rotms_ros_dispatcher
  ../rotms_ros_operations/include/rotms_ros_operations
  ../rotms_ros_state/include/rotms_ros_state
  ../rotms_ros_universal_utility/include/rotms_ros_universal_utility
  ${catkin_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDEDIR} 
  ${Boost_INCLUDE_DIRS}
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(node_dispatcher src/nodes/node_dispatcher.cpp)
target_link_libraries(node_dispatcher ${PROJECT_NAME})
