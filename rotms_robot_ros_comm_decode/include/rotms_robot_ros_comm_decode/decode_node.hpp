#pragma once
#include <map>
#include <ros/ros.h>
#include <std_msgs/String.h>

class CommDecoderPubs // a dummy class to be inhereted.
{ // The child class should contain the ROS publishers needed for the module
public:
    CommDecoderPubs();
};

typedef void (*OperationFunc)(std::string&);
typedef std::map<std::string, OperationFunc> FuncMap;

class CommDecoder
{

public:

    CommDecoder(
        ros::NodeHandle& n, 
        const std::string modulesuffix, 
        CommDecoderPubs& pubs,
        FuncMap opsdict
        );

private:

    void SubCallBack(const std_msgs::String::ConstPtr& msg);
    void CmdsProcess();
    ros::NodeHandle& n_;
	ros::Subscriber sub_;
    CommDecoderPubs& pubs_;

    char eom_; // End of message indicator
    int msglen_; // general msg length

    const std::map<std::string, std::string> cmddict_; // lookup table for cmd->CMD
    const FuncMap opsdict_; // lookup table for CMD->operation
    
    std::string ss_str_; // whole msg
};

