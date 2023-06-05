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

#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <tf2_ros/static_transform_broadcaster.h>

void ReadAndBroadcastCalibrations(
    std::string tr_str, 
    tf2_ros::StaticTransformBroadcaster& stb, 
    std::string frame_id,
    std::string child_frame_id
    )
{
    std::string packpath = ros::package::getPath("rotms_ros_kinematics");
	YAML::Node f = YAML::LoadFile(packpath + "/share/" + tr_str + ".yaml");

    geometry_msgs::TransformStamped tr;

    tr.transform.translation.x = f["x"].as<double>();
    tr.transform.translation.y = f["y"].as<double>();
    tr.transform.translation.z = f["z"].as<double>();
    tr.transform.rotation.x = f["rx"].as<double>();
    tr.transform.rotation.y = f["ry"].as<double>();
    tr.transform.rotation.z = f["rz"].as<double>();
    tr.transform.rotation.w = f["rw"].as<double>();

    tr.header.frame_id = frame_id;
	tr.child_frame_id = child_frame_id;

    stb.sendTransform(tr);

    ros::spinOnce();
}

class MngrCalibrationData
{

public:

    MngrCalibrationData(ros::NodeHandle& n, tf2_ros::StaticTransformBroadcaster& stb) : 
        n_(n), stb_(stb) {}

private:

    ros::NodeHandle& n_;
    tf2_ros::StaticTransformBroadcaster& stb_;
    
    ros::Subscriber sub_reinit = n_.subscribe(
        "/Kinematics/Query_ReInit", 2, &MngrCalibrationData::ReInitCallback, this);
    ros::Subscriber sub_updateoffset = n_.subscribe(
        "/Kinematics/Update_TR_cntct_offset", 2, &MngrCalibrationData::ChangeOffsetCallBack, this);
    ros::Subscriber sub_reinitoffset = n_.subscribe(
        "/Kinematics/Reinit_TR_cntct_offset", 2, &MngrCalibrationData::ReinitOffsetCallBack, this);

    // Operational data
    ros::Subscriber sub_reg = n_.subscribe(
        "/Kinematics/TR_bodyref_body", 2, &MngrCalibrationData::RegistrationCallBack, this);
    ros::Subscriber sub_body_cntct = n_.subscribe(
        "/Kinematics/TR_body_cntct", 2, &MngrCalibrationData::BodyCntctCallBack, this);

    void ReInitCallback(const std_msgs::String::ConstPtr& msg)
    {
        if(!msg->data.compare("_reinit__")==0) return;
        ReadAndBroadcastCalibrations("cntct_offset", stb_, "cntct", "offset");
        ReadAndBroadcastCalibrations("offset_tool", stb_, "offset", "tool");
        ReadAndBroadcastCalibrations("tool_toolref", stb_, "tool", "toolref");
        ReadAndBroadcastCalibrations("toolref_eff", stb_, "toolref", "effold");
        ReadAndBroadcastCalibrations("ptr_ptrtip", stb_, "ptr", "ptrtip");
        // TODO: need to added reinit bodyref_body (registration) and body_cntct
    }
    
    void ChangeOffsetCallBack(const geometry_msgs::Pose::ConstPtr& msg)
    {
        geometry_msgs::TransformStamped tr;

        tr.transform.translation.x = msg->position.x;
        tr.transform.translation.y = msg->position.y;
        tr.transform.translation.z = msg->position.z;
        tr.transform.rotation.x = msg->orientation.x;
        tr.transform.rotation.y = msg->orientation.y;
        tr.transform.rotation.z = msg->orientation.z;
        tr.transform.rotation.w = msg->orientation.w;

        tr.header.frame_id = "cntct";
        tr.child_frame_id = "offset";

        stb_.sendTransform(tr);
        ros::spinOnce();
    }

    void ReinitOffsetCallBack(const std_msgs::String::ConstPtr& msg)
    {
        if(!msg->data.compare("_reinitoffset__")==0) return;
        ReadAndBroadcastCalibrations("cntct_offset", stb_, "cntct", "offset");
    }

    void RegistrationCallBack(const rotms_ros_msgs::PoseValid::ConstPtr& msg)
    {
        geometry_msgs::TransformStamped tr;

        tr.transform.translation.x = msg->pose.position.x;
        tr.transform.translation.y = msg->pose.position.y;
        tr.transform.translation.z = msg->pose.position.z;
        tr.transform.rotation.x = msg->pose.orientation.x;
        tr.transform.rotation.y = msg->pose.orientation.y;
        tr.transform.rotation.z = msg->pose.orientation.z;
        tr.transform.rotation.w = msg->pose.orientation.w;

        tr.header.frame_id = "bodyref";
        tr.child_frame_id = "body";

        stb_.sendTransform(tr);
        ros::spinOnce();
    }

    void BodyCntctCallBack(const rotms_ros_msgs::PoseValid::ConstPtr& msg)
    {
        geometry_msgs::TransformStamped tr;

        tr.transform.translation.x = msg->pose.position.x;
        tr.transform.translation.y = msg->pose.position.y;
        tr.transform.translation.z = msg->pose.position.z;
        tr.transform.rotation.x = msg->pose.orientation.x;
        tr.transform.rotation.y = msg->pose.orientation.y;
        tr.transform.rotation.z = msg->pose.orientation.z;
        tr.transform.rotation.w = msg->pose.orientation.w;

        tr.header.frame_id = "body";
        tr.child_frame_id = "cntct";

        stb_.sendTransform(tr);
        ros::spinOnce();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "NodeCalibrationData");
    ros::NodeHandle nh;

    tf2_ros::StaticTransformBroadcaster stb;
    MngrCalibrationData mngr(nh, stb);

    ReadAndBroadcastCalibrations("cntct_offset", stb, "cntct", "offset");
    ReadAndBroadcastCalibrations("offset_tool", stb, "offset", "tool");
    ReadAndBroadcastCalibrations("tool_toolref", stb, "tool", "toolref");
    ReadAndBroadcastCalibrations("toolref_eff", stb, "toolref", "effold");
    ReadAndBroadcastCalibrations("ptr_ptrtip", stb, "ptr", "ptrtip");
    
    ros::spin();
    return 0;
}