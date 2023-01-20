#!/bin/bash
directory="$( pwd; )";
echo [rotms-ros INFO] Writing links to ROS workspace ...
ln -s ${directory}/rotms_ros ${directory}/../ws_rotms/src/rotms_ros
echo [rotms-ros INFO] Wrote rotms_ros 1/10
ln -s ${directory}/rotms_ros_comm ${directory}/../ws_rotms/src/rotms_ros_comm
echo [rotms-ros INFO] Wrote rotms_ros_comm 2/10
ln -s ${directory}/rotms_ros_comm_decode ${directory}/../ws_rotms/src/rotms_ros_comm_decode
echo [rotms-ros INFO] Wrote rotms_ros_comm_decode 3/10
ln -s ${directory}/rotms_ros_dispatcher ${directory}/../ws_rotms/src/rotms_ros_dispatcher
echo [rotms-ros INFO] Wrote rotms_ros_dispatcher 4/10
ln -s ${directory}/rotms_ros_kinematics ${directory}/../ws_rotms/src/rotms_ros_kinematics
echo [rotms-ros INFO] Wrote rotms_ros_kinematics 5/10
ln -s ${directory}/rotms_ros_operations ${directory}/../ws_rotms/src/rotms_ros_operations
echo [rotms-ros INFO] Wrote rotms_ros_operations 6/10
ln -s ${directory}/rotms_ros_robot_interface ${directory}/../ws_rotms/src/rotms_ros_robot_interface
echo [rotms-ros INFO] Wrote rotms_ros_robot_interface 7/10
ln -s ${directory}/rotms_ros_state ${directory}/../ws_rotms/src/rotms_ros_state
echo [rotms-ros INFO] Wrote rotms_ros_state 8/10
ln -s ${directory}/rotms_ros_msgs ${directory}/../ws_rotms/src/rotms_ros_msgs
echo [rotms-ros INFO] Wrote rotms_ros_msgs 9/10
ln -s ${directory}/rotms_ros_utility ${directory}/../ws_rotms/src/rotms_ros_utility
echo [rotms-ros INFO] Wrote rotms_ros_utility 10/10
echo Complete.