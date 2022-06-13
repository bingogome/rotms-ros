#include <ros/ros.h>
#include <std_msgs/String.h>

#include "rotms_robot_ros_interface.hpp"

RobotROSInterface::RobotROSInterface(KstServoing& kst, ros::NodeHandle& nh) 
        : 
        kst_(kst), n_(nh) {}

void RobotROSInterface::RobotEndConnectionCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (!flag_end_received_) ROS_INFO("Robot connection ending signal received by ROS node");
    if (msg->data.compare("_end_robot_connection_")==0 && !flag_end_received_)
    {
        flag_end_received_ = true;
        kst_.NetTurnoffServer();
        ROS_INFO("Ending connection signal sent to cabinet");
    }
}
