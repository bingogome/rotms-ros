cmake_minimum_required(VERSION 3.5)
project(rotms_ros_msgs)

find_package(catkin REQUIRED COMPONENTS 
  message_generation
  geometry_msgs
  std_msgs
  )

add_service_files(DIRECTORY srv)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
  )