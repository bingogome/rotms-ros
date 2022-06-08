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
    const std::map<std::string, std::string> cmddict_; // lookup table for cmd->CMD
    const std::map<std::string, void(*)(ros::Publisher&)> opsdict_; // lookup table for CMD->operation
    std::string ss_str_; // whole msg
};