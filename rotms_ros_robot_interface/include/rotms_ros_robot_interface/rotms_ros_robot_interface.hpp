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
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <rotms_ros_msgs/GetJnts.h>
#include <rotms_ros_msgs/GetEFF.h>

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

    ros::Subscriber sub_connstatus_query_ = n_.subscribe(
        "/RobInterface/ConnectStatusQuery", 2,
        &RobotROSInterface::RobotConnectStatusQuery, this);
    ros::Subscriber sub_init_conn_ = n_.subscribe(
        "/RobInterface/Connection", 2,
        &RobotROSInterface::RobotInitConnectionCallBack, this);
    ros::Subscriber sub_disconn_ = n_.subscribe(
        "/RobInterface/Connection", 2,
        &RobotROSInterface::RobotDisconnectCallBack, this);
    ros::Subscriber sub_eff_move_ = n_.subscribe(
        "/RobInterface/MoveEFF", 2,
        &RobotROSInterface::RobotEFFMotionCallBack, this);
    ros::Subscriber sub_jnt_move_ = n_.subscribe(
        "/RobInterface/MoveJnt", 2,
        &RobotROSInterface::RobotJntMotionCallBack, this);
    
    ros::Publisher pub_eff_ = n_.advertise<geometry_msgs::Pose>(
        "/RobInterfaceOut/EFFPose", 2);
    ros::Publisher pub_jnt_ = n_.advertise<std_msgs::Float32MultiArray>(
        "/RobInterfaceOut/JntPos", 2);
    ros::Publisher pub_robconnstatus_ = n_.advertise<std_msgs::Bool>(
        "/RobInterfaceOut/RobConnStatus", 2);

    ros::ServiceServer srv_get_jnt_ = n_.advertiseService(
        "/RobInterface/GetJntsPos", &RobotROSInterface::RobotGetJntsPosCallBack, this);
    ros::ServiceServer srv_get_eff_ = n_.advertiseService(
        "/RobInterface/GetEFFPose", &RobotROSInterface::RobotGetEFFPoseCallBack, this);

    // Query the robot connection status
    void RobotConnectStatusQuery(const std_msgs::String::ConstPtr& msg);
    // Start the cabinet connection
    void RobotInitConnectionCallBack(const std_msgs::String::ConstPtr& msg);
    // End the cabinet connection
    void RobotDisconnectCallBack(const std_msgs::String::ConstPtr& msg);
    // Move to received EFF pose at a slow speed
    void RobotEFFMotionCallBack(const geometry_msgs::Pose::ConstPtr& msg);
    // Move to received joints at a slow speed
    void RobotJntMotionCallBack(const std_msgs::Float32MultiArray::ConstPtr& msg);

    // Get joint positions
    bool RobotGetJntsPosCallBack(
        rotms_ros_msgs::GetJnts::Request &req, rotms_ros_msgs::GetJnts::Response &res);
    // Get end-effector pose
    bool RobotGetEFFPoseCallBack(
        rotms_ros_msgs::GetEFF::Request &req, rotms_ros_msgs::GetEFF::Response &res);

    // Should be only called when terminating node
    ros::Subscriber sub_end_ = n_.subscribe(
        "/RobInterface/TerminateNode", 2, 
        &RobotROSInterface::RobotTerminateNodeCallBack, this);
    void RobotTerminateNodeCallBack(const std_msgs::String::ConstPtr& msg);

};