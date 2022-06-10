#pragma once
#include <map>
#include <ros/ros.h>
#include <std_msgs/String.h>

typedef std::map<std::string, void(*)(CommDecoderPubs&, std::string&)> FuncMap;

class CommDecoder
{

public:

    CommDecoder(
        ros::NodeHandle& n, 
        const std::string modulesuffix, 
        CommDecoderPubs& pubs,
        FuncMap opsdict
        );

protected:

    void SubCallBack(const std_msgs::String::ConstPtr& msg);
    void CmdsProcess();
    ros::NodeHandle& n_;
	ros::Subscriber sub_;
    CommDecoderPubs& pubs_;

    const std::map<std::string, std::string> cmddict_; // lookup table for cmd->CMD
    const FuncMap opsdict_; // lookup table for CMD->operation
    
    std::string ss_str_; // whole msg
};

class CommDecoderPubs
{
public:
    CommDecoderPubs();
protected:
};