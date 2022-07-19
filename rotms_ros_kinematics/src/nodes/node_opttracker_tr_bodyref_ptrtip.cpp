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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>

#include "transform_conversions.hpp"

class MngrTBodyrefPtrtip
{
// Manages the flag of running the calculation of the transform
public:
    
    MngrTBodyrefPtrtip(ros::NodeHandle& n) : n_(n){}
    bool run_flag = false;

    tf2::Transform tr_pol_bodyref_;
    tf2::Transform tr_pol_ptr_;

private:

    ros::NodeHandle& n_;
    ros::Subscriber sub_run_ = n_.subscribe(
        "/Kinematics/Flag_bodyref_ptrtip", 2, &MngrTBodyrefPtrtip::FlagCallBack, this);
    ros::Subscriber sub_tr_pol_bodyref_ = n_.subscribe(
        "/NDI/HeadRef/local/measured_cp", 2, &MngrTBodyrefPtrtip::PolBodyRefCallBack, this);
    ros::Subscriber sub_tr_pol_ptr_ = n_.subscribe(
        "/NDI/PointerNew/local/measured_cp", 2, &MngrTBodyrefPtrtip::PolPtrCallBack, this);

    void FlagCallBack(const std_msgs::String::ConstPtr& msg)
    {
        if(msg->data.compare("_start__")==0) run_flag = true;
        if(msg->data.compare("_end__")==0) run_flag = false;
    }

    void PolBodyRefCallBack(const geometry_msgs::TransformStamped::ConstPtr& msg)
    {
        if(run_flag) tr_pol_bodyref_ = ConvertToTf2Transform(msg);
    }

    void PolPtrCallBack(const geometry_msgs::TransformStamped::ConstPtr& msg)
    {
        if(run_flag) tr_pol_ptr_ = ConvertToTf2Transform(msg);
    }
};

int main(int argc, char **argv)
{
    // ROS stuff
    ros::init(argc, argv, "NodeOpttrackerTrBodyrefPtrtip");
    ros::NodeHandle nh;
    ros::Rate rate(50.0);

    // Instantiate the flag manager
    MngrTBodyrefPtrtip mngr1(nh);

    // Initialize the requred transforms
    geometry_msgs::PoseConstPtr tr_ptr_ptrtip = ros::topic::waitForMessage<geometry_msgs::Pose>(
        "/Kinematics/TR_ptr_ptrtip");

    // Initialize the tf2 intermediate variables
    tf2::Transform tr_pol_bodyref_;
    tf2::Transform tr_pol_ptr_;
    tf2::Transform tr_ptr_ptrtip_ = ConvertToTf2Transform(tr_ptr_ptrtip);

    // Initialize the result transform
    tf2::Transform tr_bodyref_ptrtip_;

    // Initialize the result variable and its publisher
    geometry_msgs::Pose tr_bodyref_ptrtip;
    ros::Publisher pub_bodyref_ptrtip = nh.advertise<geometry_msgs::Point>(
        "/Kinematics/T_bodyref_ptrtip", 5);

    // Go in the loop, with the flag indicating wether do the calculation or not
    while (nh.ok())
    {
        if (mngr1.run_flag)
        {
            tr_pol_bodyref_ = mngr1.tr_pol_bodyref_;
            tr_pol_ptr_ = mngr1.tr_pol_ptr_;

            tr_bodyref_ptrtip_ = tr_pol_bodyref_.inverse() * tr_pol_ptr_ * tr_ptr_ptrtip_;
            tr_bodyref_ptrtip = ConvertToGeometryPose(tr_bodyref_ptrtip_);
            
            pub_bodyref_ptrtip.publish(tr_bodyref_ptrtip.position);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}