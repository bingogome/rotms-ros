#include <ros/ros.h>
#include <std_msgs/String.h>

#include "kst_servoing.hpp"

class RobotROSInterface
{
public:
    RobotROSInterface(KstServoing& kst, ros::NodeHandle& nh);
private:
    KstServoing& kst_;
    ros::NodeHandle& n_;
    ros::Subscriber sub_end_ = n_.subscribe(
        "/RobInterface/TerminateNode", 10, 
        &RobotROSInterface::RobotTerminateNodeCallBack, this);
    bool flag_end_handshaked_ = false;
        
    void RobotTerminateNodeCallBack(const std_msgs::String::ConstPtr& msg);
};