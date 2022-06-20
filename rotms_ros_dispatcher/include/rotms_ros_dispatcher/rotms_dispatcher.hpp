#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>

class Dispatcher
{

public:

    Dispatcher(ros::NodeHandle& n);
    void RegistrationCallBack();

private:

    ros::NodeHandle& n_;
    ros::Publisher pub_registration_ = 
        n_.advertise<geometry_msgs::Pose>("/Rotms/State/Update/Registration", 5);

    void AutodigitizationCallBack(const std_msgs::String::ConstPtr& msg);
    // void RegistrationCallBack(const std_msgs::String::ConstPtr& msg);
    
    void RegistrationUsePrevCallBack(const std_msgs::String::ConstPtr& msg);

};


