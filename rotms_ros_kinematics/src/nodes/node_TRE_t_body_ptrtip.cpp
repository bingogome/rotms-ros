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
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/Point.h>

#include "transform_conversions.hpp"

class TREMngrBodyPtrtip
{
// Manages the flag of running the calculation of the transform
public:
    
    TREMngrBodyPtrtip(ros::NodeHandle& n) : n_(n){}
    bool run_flag = false;

    tf2::Transform tr_body_bodyref_;
    tf2::Vector3 t_bodyref_ptrtip_;

private:

    ros::NodeHandle& n_;
    ros::Subscriber sub_run_ = n_.subscribe(
        "/Kinematics/Flag_t_body_ptrtip", 2, &TREMngrBodyPtrtip::FlagCallBack, this);
    ros::Subscriber sub_tr_bodyref_body_ = n_.subscribe(
        "/Kinematics/TR_bodyref_body", 2, &TREMngrBodyPtrtip::BodyRefBodyCallBack, this);
    ros::Subscriber sub_t_bodyref_ptrtip_ = n_.subscribe(
        "/Kinematics/T_bodyref_ptrtip", 2, &TREMngrBodyPtrtip::BodyRefPtrtipCallBack, this);

    void FlagCallBack(const std_msgs::String::ConstPtr& msg)
    {
        if(msg->data.compare("_start__")==0) run_flag = true;
        if(msg->data.compare("_end__")==0) run_flag = false;
    }

    void BodyRefBodyCallBack(const rotms_ros_msgs::PoseValid::ConstPtr& msg)
    {
        if(msg->valid) tr_body_bodyref_ = ConvertToTf2Transform(msg).inverse();
    }

    void BodyRefPtrtipCallBack(const geometry_msgs::Point::ConstPtr& msg)
    {
        if(run_flag) t_bodyref_ptrtip_ = ConvertToTf2Vector3(msg);
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
    ros::init(argc, argv, "NodeTRETBodyPtrtip");
    ros::NodeHandle nh;
    ros::Rate rate(5.0);

    // Instantiate the flag manager
    TREMngrBodyPtrtip mngr1(nh);

    // Initialize the result 
    tf2::Vector3 t_body_ptrtip_;

    // Initialize the result variable and its publisher
    // Format:
    // __msg_point_0000.00000_0000.00000_0000.00000
    // x, y, z in mm
    ros::Publisher pub_encode_body_tool = nh.advertise<std_msgs::String>(
        "/MedImgComm/msg_to_send_hi_f", 5);
    std_msgs::String msg_out;

    // Go in the loop, with the flag indicating wether do the calculation or not
    while (nh.ok())
    {
        if (mngr1.run_flag)
        {
            t_body_ptrtip_ = mngr1.tr_body_bodyref_ * mngr1.t_bodyref_ptrtip_;
            msg_out.data = "__msg_point_" + // convert m to mm
                FormatDouble2String(t_body_ptrtip_.getX() * 1000.0, 5) + "_" +
                FormatDouble2String(t_body_ptrtip_.getY() * 1000.0, 5) + "_" +
                FormatDouble2String(t_body_ptrtip_.getZ() * 1000.0, 5);
            pub_encode_body_tool.publish(msg_out);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}