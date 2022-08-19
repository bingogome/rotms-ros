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
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <iostream>
#include <iomanip>

#include "transform_conversions.hpp"
#include "rotms_ros_msgs/PoseValid.h"

class MngrTrBodyTool
{
// Manages the flag of running the calculation of the transform
public:
    
    MngrTrBodyTool(ros::NodeHandle& n) : n_(n){}
    bool run_flag = false;

    tf2::Transform tr_pol_bodyref_;
    tf2::Transform tr_pol_toolref_;

private:

    ros::NodeHandle& n_;
    ros::Subscriber sub_run_ = n_.subscribe(
        "/Kinematics/Flag_body_tool", 2, &MngrTrBodyTool::FlagCallBack, this);
    ros::Subscriber sub_tr_pol_bodyref_ = n_.subscribe(
        "/NDI/HeadRef/local/measured_cp", 2, &MngrTrBodyTool::PolBodyRefCallBack, this);
    ros::Subscriber sub_tr_pol_toolref_ = n_.subscribe(
        "/NDI/CoilRef/local/measured_cp", 2, &MngrTrBodyTool::PolToolRefCallBack, this);

    void FlagCallBack(const std_msgs::String::ConstPtr& msg)
    {
        if(msg->data.compare("_start__")==0) run_flag = true;
        if(msg->data.compare("_end__")==0) run_flag = false;
    }

    void PolBodyRefCallBack(const geometry_msgs::TransformStamped::ConstPtr& msg)
    {
        if(run_flag) tr_pol_bodyref_ = ConvertToTf2Transform(msg);
    }

    void PolToolRefCallBack(const geometry_msgs::TransformStamped::ConstPtr& msg)
    {
        if(run_flag) tr_pol_toolref_ = ConvertToTf2Transform(msg);
    }
};

std::string FormatDouble2String(double a, int dec)
{
	std::stringstream stream;
    stream << std::fixed << std::setprecision(dec) << a;
    std::string s = stream.str();
    return s;
}

int main(int argc, char **argv)
{
    // ROS stuff
    ros::init(argc, argv, "NodeVizTrBodyTool");
    ros::NodeHandle nh;
    ros::Rate rate(60.0);

    // Instantiate the flag manager
    MngrTrBodyTool mngr(nh);

    // Initialize the requred transforms
    rotms_ros_msgs::PoseValidConstPtr tr_bodyref_body = ros::topic::waitForMessage<rotms_ros_msgs::PoseValid>(
            "/Kinematics/TR_bodyref_body");
    ros::Rate check_valid_rate(10);
    while(!tr_bodyref_body->valid)
    {
        tr_bodyref_body = ros::topic::waitForMessage<rotms_ros_msgs::PoseValid>(
            "/Kinematics/TR_bodyref_body");
        check_valid_rate.sleep();
    }
        
    geometry_msgs::PoseConstPtr tr_tool_toolref = ros::topic::waitForMessage<geometry_msgs::Pose>(
        "/Kinematics/TR_tool_toolref");

    // Initialize the tf2 intermediate variables
    tf2::Transform tr_pol_bodyref_;
    tf2::Transform tr_pol_toolref_;
    tf2::Transform tr_bodyref_body_ = ConvertToTf2Transform(tr_bodyref_body);
    tf2::Transform tr_tool_toolref_ = ConvertToTf2Transform(tr_tool_toolref);

    // Initialize the result transform
    tf2::Transform tr_body_tool_;

    // Initialize the result variable and its publisher
    // Format:
    // __msg_pose_0000.00000_0000.00000_0000.00000_0000.00000_0000.00000_0000.00000_0000.00000
    // x, y, z, qx, qy, qz, qw in mm
    ros::Publisher pub_encode_body_tool = nh.advertise<std_msgs::String>(
        "/TargetVizComm/msg_to_send_hi_f", 5);
    std_msgs::String msg_out;

    // Go in the loop, with the flag indicating wether do the calculation or not
    while (nh.ok())
    {
        if (mngr.run_flag)
        {
            tr_pol_bodyref_ = mngr.tr_pol_bodyref_;
            tr_pol_toolref_ = mngr.tr_pol_toolref_;
            tr_body_tool_ = 
                tr_bodyref_body_.inverse() * tr_pol_bodyref_.inverse() *
                tr_pol_toolref_ * tr_tool_toolref_.inverse();
            msg_out.data = "__msg_pose_" + // convert m to mm
                FormatDouble2String(tr_body_tool_.getOrigin().x() * 1000.0, 5) + "_" +
                FormatDouble2String(tr_body_tool_.getOrigin().y() * 1000.0, 5) + "_" +
                FormatDouble2String(tr_body_tool_.getOrigin().z() * 1000.0, 5) + "_" + 
                FormatDouble2String(tr_body_tool_.getRotation().x(), 5) + "_" + 
                FormatDouble2String(tr_body_tool_.getRotation().y(), 5) + "_" + 
                FormatDouble2String(tr_body_tool_.getRotation().z(), 5) + "_" +
                FormatDouble2String(tr_body_tool_.getRotation().w(), 5);
            pub_encode_body_tool.publish(msg_out);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}