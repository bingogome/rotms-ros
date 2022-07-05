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

#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <rotms_ros_msgs/GetJnts.h>
#include <rotms_ros_msgs/GetEFF.h>
#include <vector>
#include "state_machine.hpp"

struct VolatileTempDataCache 
{
    int landmark_total = -1;
    std::vector<std::vector<double>> landmark_coords;

    bool toolpose_t_recvd = false;
    bool toolpose_r_recvd = false;
    std::vector<double> toolpose_t;
    std::vector<double> toolpose_r;
};

class Dispatcher
{

public:

    Dispatcher(ros::NodeHandle& n, const std::vector<WorkState*>& states);

private:

    ros::NodeHandle& n_;
    const std::vector<WorkState*>& states_;
    int activated_state_;

    // Dispatcher receiving query signals
    ros::Subscriber sub_medimg_landmarkplanmeta_ = n_.subscribe(
        "/MedImg/LandmarkPlanMeta", 2, &Dispatcher::LandmarkPlanMetaCallBack, this);
    ros::Subscriber sub_medimg_landmarkplanfids_ = n_.subscribe(
        "/MedImg/LandmarkPlanFids", 10, &Dispatcher::LandmarkPlanFidsCallBack, this);
    ros::Subscriber sub_medimg_autodigitization_ = n_.subscribe(
        "/MedImg/StartAct", 2, &Dispatcher::AutodigitizationCallBack, this);
    ros::Subscriber sub_medimg_registration_ = n_.subscribe(
        "/MedImg/StartAct", 2, &Dispatcher::RegistrationCallBack, this);
    ros::Subscriber sub_medimg_toolposeorient_ = n_.subscribe(
        "/MedImg/ToolPoseOrient", 2, &Dispatcher::ToolPoseOrientCallBack, this);
    ros::Subscriber sub_medimg_toolposetrans_ = n_.subscribe(
        "/MedImg/ToolPoseTrans", 2, &Dispatcher::ToolPoseTransCallBack, this);
    ros::Subscriber sub_robctrl_robconnect_ = n_.subscribe(
        "/RobCtrl/RobConnection", 2, &Dispatcher::RobConnectCallBack, this);
    ros::Subscriber sub_robctrl_robdisconnect_ = n_.subscribe(
        "/RobCtrl/RobConnection", 2, &Dispatcher::RobDisconnectCallBack, this);
    ros::Subscriber sub_robctrl_getjnt_ = n_.subscribe(
        "/RobCtrl/GetInfo", 2, &Dispatcher::GetJntsCallBack, this);
    ros::Subscriber sub_robctrl_geteff_ = n_.subscribe(
        "/RobCtrl/GetInfo", 2, &Dispatcher::GetEFFCallBack, this);
    ros::Subscriber sub_robconnstatus_ = n_.subscribe(
        "/RobInterfaceOut/RobConnStatus", 2, &Dispatcher::UpdateRobotConnFlagCallBack, this);

    // Dispatcher sending query response signals
    ros::Publisher pub_robctrlcomm = n_.advertise<std_msgs::String>(
        "/RobCtrlComm/msg_to_send", 2);

    // Cruicial operations (operations that affect main user logic and its states/flags/operations)
    void LandmarkPlanMetaCallBack(const std_msgs::Int16::ConstPtr& msg);
    void AutodigitizationCallBack(const std_msgs::String::ConstPtr& msg);
    void RegistrationCallBack(const std_msgs::String::ConstPtr& msg);
    void RegistrationUsePrevCallBack(const std_msgs::String::ConstPtr& msg);
    void ToolPoseOrientCallBack(const geometry_msgs::Quaternion::ConstPtr& msg);
    void ToolPoseTransCallBack(const geometry_msgs::Point::ConstPtr& msg);

    // Secondary and intermediate operations
    void LandmarkPlanFidsCallBack(const std_msgs::Float32MultiArray::ConstPtr& msg);

    // Robot operations
    void UpdateRobotConnFlagCallBack(const std_msgs::Bool::ConstPtr& msg);
    void RobConnectCallBack(const std_msgs::String::ConstPtr& msg);
    void RobDisconnectCallBack(const std_msgs::String::ConstPtr& msg);
    void GetJntsCallBack(const std_msgs::String::ConstPtr& msg);
    void GetEFFCallBack(const std_msgs::String::ConstPtr& msg);

    // Temp data cache (volatile)
    struct VolatileTempDataCache datacache_;
    void ResetVolatileDataCacheLandmarks();
    void ResetVolatileDataCacheToolPose();

    // Robot interface
    ros::Publisher pub_init_conn_ = n_.advertise<std_msgs::String>(
        "/RobInterface/Connection", 2);
    ros::Publisher pub_disconn_ = n_.advertise<std_msgs::String>(
        "/RobInterface/Connection", 2);
    ros::ServiceClient clt_jnt_ = n_.serviceClient<rotms_ros_msgs::GetJnts>(
        "/RobInterface/GetJntsPos");
    ros::ServiceClient clt_eff_ = n_.serviceClient<rotms_ros_msgs::GetEFF>(
        "/RobInterface/GetEFFPose");

};

void SaveLandmarkPlanData(struct VolatileTempDataCache datacache, std::string f);
void SaveToolPoseData(struct VolatileTempDataCache datacache, std::string f);
std::string FormatDouble2String(double a, int dec);
std::string GetTimeString();
