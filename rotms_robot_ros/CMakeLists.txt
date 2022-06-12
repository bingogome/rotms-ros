cmake_minimum_required(VERSION 3.5)
project(rotms_robot_ros)

find_package(catkin REQUIRED COMPONENTS 
  roslaunch
  )

roslaunch_add_file_check(launch)

catkin_package()