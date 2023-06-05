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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "rotms_ros_msgs/PoseValid.h"

class MngrSensorData
{

public:

    MngrSensorData(ros::NodeHandle& n, tf2_ros::StaticTransformBroadcaster& stb) : 
        n_(n), stb_(stb) {}

private:

    ros::NodeHandle& n_;
    tf2_ros::StaticTransformBroadcaster& stb_;
    
    ros::Subscriber sub_expiredEFFOld = n_.subscribe(
        "/Kinematics/ExpiredEFFOld", 2, &MngrSensorData::UpdateEFFOld, this);
    ros::Subscriber sub_derivedeff = n_.subscribe(
        "/Kinematics/TR_derivedeff", 2, &MngrSensorData::UpdateEFFTargeted, this);

    void UpdateEFFOld(const geometry_msgs::Pose::ConstPtr& msg)
    {
        geometry_msgs::TransformStamped tr;

        tr.transform.translation.x = msg->position.x;
        tr.transform.translation.y = msg->position.y;
        tr.transform.translation.z = msg->position.z;
        tr.transform.rotation.x = msg->orientation.x;
        tr.transform.rotation.y = msg->orientation.y;
        tr.transform.rotation.z = msg->orientation.z;
        tr.transform.rotation.w = msg->orientation.w;

        tr.header.frame_id = "robbase";
        tr.child_frame_id = "effold";

        stb_.sendTransform(tr);
        ros::spinOnce();
    }

    void UpdateEFFTargeted(const rotms_ros_msgs::PoseValid::ConstPtr& msg)
    {
        geometry_msgs::TransformStamped tr;

        tr.transform.translation.x = msg->pose.position.x;
        tr.transform.translation.y = msg->pose.position.y;
        tr.transform.translation.z = msg->pose.position.z;
        tr.transform.rotation.x = msg->pose.orientation.x;
        tr.transform.rotation.y = msg->pose.orientation.y;
        tr.transform.rotation.z = msg->pose.orientation.z;
        tr.transform.rotation.w = msg->pose.orientation.w;

        tr.header.frame_id = "robbase";
        tr.child_frame_id = "efftargeted";

        stb_.sendTransform(tr);
        ros::spinOnce();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "NodeCalibrationData");
    ros::NodeHandle nh;

    tf2_ros::StaticTransformBroadcaster stb;
    MngrSensorData mngr(nh, stb);
    
    ros::spin();
    return 0;
}