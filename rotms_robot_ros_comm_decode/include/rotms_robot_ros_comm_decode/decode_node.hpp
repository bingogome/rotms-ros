#pragma once
#include <map>
#include <ros/ros.h>
#include <std_msgs/String.h>

typedef std::vector<ros::Publisher> PublisherVec;
typedef void (*OperationFunc)(std::string&, PublisherVec&);
typedef std::map<std::string, OperationFunc> FuncMap;


class CommDecoder
{

public:

    CommDecoder(
        ros::NodeHandle& n, 
        const std::string modulesuffix,
        FuncMap opsdict
        );

protected:

    void SubCallBack(const std_msgs::String::ConstPtr& msg);
    virtual void CmdsProcess(std::string& lookupkey, std::string& msgcontent);
    ros::NodeHandle& n_;
	ros::Subscriber sub_;
    PublisherVec pubs_;

    char eom_; // End of message indicator
    int msglen_; // general msg length
    int msgheaderlen_ = 16; // cmd hear length

    const std::map<std::string, std::string> cmddict_; // lookup table for cmd->CMD
    const FuncMap opsdict_; // lookup table for CMD->operation
    
    std::string ss_str_; // whole msg
};

