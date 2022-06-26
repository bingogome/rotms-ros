/***
MIT License

Copyright (c) 2022 Yihao Liu, Johns Hopkins University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
***/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

#include "kst_servoing.hpp"

class RobotROSInterface
{

public:

    RobotROSInterface(KstServoing& kst, ros::NodeHandle& nh);

private:

    bool flag_end_handshaked_ = false; // Used in terminating robot node.
    bool flag_connected_ = false; // robot cabinet connection state
    KstServoing& kst_;
    ros::NodeHandle& n_;

    ros::Subscriber sub_init_conn_ = n_.subscribe(
        "/RobInterface/InitConnection", 10,
        &RobotROSInterface::RobotInitConnectionCallBack, this);
    ros::Subscriber sub_end_ = n_.subscribe(
        "/RobInterface/TerminateNode", 10, 
        &RobotROSInterface::RobotTerminateNodeCallBack, this);
    ros::Subscriber sub_eef_move_ = n_.subscribe(
        "/RobInterface/MoveEEF", 10,
        &RobotROSInterface::RobotEEFMotionCallBack, this);
    ros::Subscriber sub_get_jnt_ = n_.subscribe(
        "/RobInterface/GetJntPos", 10,
        &RobotROSInterface::RobotGetJntPosCallBack, this);
    ros::Subscriber sub_get_eef_ = n_.subscribe(
        "/RobInterface/GetEEFPose", 10,
        &RobotROSInterface::RobotGetEEFPoseCallBack, this);

    ros::Publisher pub_eef_ = n_.advertise<geometry_msgs::Pose>(
        "/RobInterfaceOut/EEFPose", 10);
    ros::Publisher pub_jnt_ = n_.advertise<geometry_msgs::Pose>(
        "/RobInterfaceOut/JntPos", 10);

    // Start the cabinet connection
    void RobotInitConnectionCallBack(const std_msgs::String::ConstPtr& msg);
    // Move to received EEF pose at a slow speed
    void RobotEEFMotionCallBack(const geometry_msgs::Pose::ConstPtr& msg);
    // Get joint positions
    void RobotGetJntPosCallBack(const std_msgs::String::ConstPtr& msg);
    // Get end-effector pose
    void RobotGetEEFPoseCallBack(const std_msgs::String::ConstPtr& msg);
    // Should be only called when terminating node
    void RobotTerminateNodeCallBack(const std_msgs::String::ConstPtr& msg);
};