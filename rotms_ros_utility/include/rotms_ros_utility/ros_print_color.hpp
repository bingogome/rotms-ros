#pragma once
#include "ros_print_color_def.hpp"
#include <ros/ros.h>

#define ROS_BLACK_STREAM(x)   ROS_INFO_STREAM(print_color_ros::BLACK   << x << print_color_ros::ENDCOLOR)
#define ROS_RED_STREAM(x)     ROS_INFO_STREAM(print_color_ros::RED     << x << print_color_ros::ENDCOLOR)
#define ROS_GREEN_STREAM(x)   ROS_INFO_STREAM(print_color_ros::GREEN   << x << print_color_ros::ENDCOLOR)
#define ROS_YELLOW_STREAM(x)  ROS_INFO_STREAM(print_color_ros::YELLOW  << x << print_color_ros::ENDCOLOR)
#define ROS_BLUE_STREAM(x)    ROS_INFO_STREAM(print_color_ros::BLUE    << x << print_color_ros::ENDCOLOR)
#define ROS_MAGENTA_STREAM(x) ROS_INFO_STREAM(print_color_ros::MAGENTA << x << print_color_ros::ENDCOLOR)
#define ROS_CYAN_STREAM(x)    ROS_INFO_STREAM(print_color_ros::CYAN    << x << print_color_ros::ENDCOLOR)

#define ROS_BLACK_STREAM_COND(c, x)   ROS_INFO_STREAM_COND(c, print_color_ros::BLACK   << x << print_color_ros::ENDCOLOR)
#define ROS_RED_STREAM_COND(c, x)     ROS_INFO_STREAM_COND(c, print_color_ros::RED     << x << print_color_ros::ENDCOLOR)
#define ROS_GREEN_STREAM_COND(c, x)   ROS_INFO_STREAM_COND(c, print_color_ros::GREEN   << x << print_color_ros::ENDCOLOR)
#define ROS_YELLOW_STREAM_COND(c, x)  ROS_INFO_STREAM_COND(c, print_color_ros::YELLOW  << x << print_color_ros::ENDCOLOR)
#define ROS_BLUE_STREAM_COND(c, x)    ROS_INFO_STREAM_COND(c, print_color_ros::BLUE    << x << print_color_ros::ENDCOLOR)
#define ROS_MAGENTA_STREAM_COND(c, x) ROS_INFO_STREAM_COND(c, print_color_ros::MAGENTA << x << print_color_ros::ENDCOLOR)
#define ROS_CYAN_STREAM_COND(c, x)    ROS_INFO_STREAM_COND(c, print_color_ros::CYAN    << x << print_color_ros::ENDCOLOR)

// etc ...
// see also https://en.wikipedia.org/wiki/ANSI_escape_code