#pragma once
#include <map>
#include <ros/ros.h>
#include <std_msgs/String.h>

class CommDecoder
{

public:
    CommDecoder(ros::NodeHandle& n, const std::string modulesuffix);

protected:
    void SubCallBack(const std_msgs::String::ConstPtr& msg);
    virtual void CmdsProcess();
    ros::NodeHandle& n_;
	ros::Subscriber sub_;
    const std::map<std::string, std::string> cmddict_;
    std::string ss_str_; // whole msg
};