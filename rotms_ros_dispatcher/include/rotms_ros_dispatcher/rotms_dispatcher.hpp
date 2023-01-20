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

#include <vector>
#include <map>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <rotms_ros_msgs/GetJnts.h>
#include <rotms_ros_msgs/GetEFF.h>
#include <rotms_ros_msgs/PoseValid.h>

#include "state_machine_registration.hpp"
#include "state_machine_digitization.hpp"
#include "state_machine_toolplan.hpp"
#include "state_machine_robot.hpp"
#include "dispatcher_utility.hpp"

struct StateSet
{
    const std::vector<StateRegistration*>&  state_registration;
    const std::vector<StateDigitization*>&  state_digitization;
    const std::vector<StateToolplan*>&      state_toolplan;
    const std::vector<StateRobot*>&         state_robot;
};

class Dispatcher
{

public:

    Dispatcher(ros::NodeHandle& n, struct StateSet& states_set);

private:

    ros::NodeHandle& n_;
    struct StateSet& states_set_;
    std::map<std::string, int> activated_state_{ 
        {"REGISTRATION",    0}, 
        {"DIGITIZATION",    0},
        {"TOOLPLAN",        0},
        {"ROBOT",           0}
    };

    // Dispatcher receiving query signals
    ros::Subscriber sub_medimg_landmarkplanmeta_    = n_.subscribe(
        "/MedImg/LandmarkPlanMeta", 2, &Dispatcher::LandmarkPlanMetaCallBack, this);
    ros::Subscriber sub_medimg_landmarkplanfids_    = n_.subscribe(
        "/MedImg/LandmarkPlanFids", 10, &Dispatcher::LandmarkPlanLandmarksCallBack, this);
    ros::Subscriber sub_medimg_autodigitization_    = n_.subscribe(
        "/MedImg/StartAct", 2, &Dispatcher::DigitizationCallBack, this);
    ros::Subscriber sub_medimg_registration_        = n_.subscribe(
        "/MedImg/StartAct", 2, &Dispatcher::RegistrationCallBack, this);
    ros::Subscriber sub_medimg_tre_                 = n_.subscribe(
        "/MedImg/StartAct", 2, &Dispatcher::TRECalculationCallBack, this);
    ros::Subscriber sub_medimg_toolposeorient_      = n_.subscribe(
        "/MedImg/ToolPoseOrient", 2, &Dispatcher::ToolPoseOrientCallBack, this);
    ros::Subscriber sub_medimg_toolposetrans_       = n_.subscribe(
        "/MedImg/ToolPoseTrans", 2, &Dispatcher::ToolPoseTransCallBack, this);
    ros::Subscriber sub_medimg_icp_                 = n_.subscribe(
        "/MedImg/ICPAct", 2, &Dispatcher::ICPCallBack, this);
    ros::Subscriber sub_robctrl_robconnect_         = n_.subscribe(
        "/RobCtrl/RobConnection", 2, &Dispatcher::RobConnectCallBack, this);
    ros::Subscriber sub_robctrl_robdisconnect_      = n_.subscribe(
        "/RobCtrl/RobConnection", 2, &Dispatcher::RobDisconnectCallBack, this);
    ros::Subscriber sub_robctrl_getjnt_             = n_.subscribe(
        "/RobCtrl/GetInfo", 2, &Dispatcher::GetJntsCallBack, this);
    ros::Subscriber sub_robctrl_geteff_             = n_.subscribe(
        "/RobCtrl/GetInfo", 2, &Dispatcher::GetEFFCallBack, this);
    ros::Subscriber sub_robconnstatus_              = n_.subscribe(
        "/RobInterfaceOut/RobConnStatus", 2, &Dispatcher::UpdateRobotConnFlagCallBack, this);
    ros::Subscriber sub_robctrl_execute_            = n_.subscribe(
        "/RobCtrl/Motion", 2, &Dispatcher::ExecuteMotionToOffsetCallBack, this);
    ros::Subscriber sub_robctrl_executeconfirm_     = n_.subscribe(
        "/RobCtrl/Motion", 2, &Dispatcher::ExecuteConfirmMotionCallBack, this);
    ros::Subscriber sub_robctrl_executebackto_      = n_.subscribe(
        "/RobCtrl/Motion", 2, &Dispatcher::ExecuteBackToCallBack, this);
    ros::Subscriber sub_robctrl_executemanadjust_   = n_.subscribe(
        "/RobCtrl/MotionAdjust", 2, &Dispatcher::ExecuteManualAdjust, this);
    ros::Subscriber sub_robctrl_sessionreinit_      = n_.subscribe(
        "/RobCtrl/Session", 2, &Dispatcher::SessionReinitCallBack, this);
    ros::Subscriber sub_targetviz_                  = n_.subscribe(
        "/TargetViz/Visualize", 2, &Dispatcher::TargetVizCallBack, this);
    ros::Subscriber sub_savetargetplanandreal_      = n_.subscribe(
        "/TargetViz/DataRecord", 2, &Dispatcher::TargetVizSavePlanAndRealPoseCallBack, this);

    // Dispatcher sending query response signals
    ros::Publisher pub_robctrlcomm_                 = n_.advertise<std_msgs::String>(
        "/RobCtrlComm/msg_to_send", 2);
    ros::Publisher pub_medplancomm_                 = n_.advertise<std_msgs::String>(
        "/MedImgComm/msg_to_send", 2);
    ros::Publisher pub_effold_                      = n_.advertise<rotms_ros_msgs::PoseValid>( 
        "/Kinematics/TR_robbase_effold", 1, true); // should only be called by ExecuteMotionToTargetEFFPose
    
    // Dispatcher sending query
    ros::Publisher pub_gettargeteff_                = n_.advertise<std_msgs::String>(
        "/Kinematics/Query_GetTargetEff", 2);
    ros::Publisher pub_changeoffset_                = n_.advertise<geometry_msgs::Pose>(
        "/Kinematics/Update_TR_cntct_offset", 2);
    ros::Publisher pub_reinitoffset_                = n_.advertise<std_msgs::String>(
        "/Kinematics/Reinit_TR_cntct_offset", 2);
    ros::Publisher pub_reinitcaldata_               = n_.advertise<std_msgs::String>(
        "/Kinematics/Query_ReInit", 2);
    ros::Publisher pub_flag_bodytoolviz_            = n_.advertise<std_msgs::String>(
        "/Kinematics/Flag_body_tool", 2);
    ros::Publisher pub_flag_t_body_ptrtip_          = n_.advertise<std_msgs::String>(
        "/Kinematics/Flag_t_body_ptrtip", 2);
    ros::Publisher pub_run_opttracker_tr_bodyref_ptrtip_ = n_.advertise<std_msgs::String>(
        "/Kinematics/Flag_bodyref_ptrtip", 2);
        
    // Cruicial operations (operations that affect main user logic and its states/flags/operations)
    void LandmarkPlanMetaCallBack(const std_msgs::Int16::ConstPtr& msg);
    void DigitizationCallBack(const std_msgs::String::ConstPtr& msg);
    void RegistrationCallBack(const std_msgs::String::ConstPtr& msg);
    void ToolPoseOrientCallBack(const geometry_msgs::Quaternion::ConstPtr& msg);
    void ToolPoseTransCallBack(const geometry_msgs::Point::ConstPtr& msg);

    // Secondary and intermediate operations
    void LandmarkPlanLandmarksCallBack(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void SessionReinitCallBack(const std_msgs::String::ConstPtr& msg);
    void TargetVizCallBack(const std_msgs::String::ConstPtr& msg);
    void TargetVizSavePlanAndRealPoseCallBack(const std_msgs::String::ConstPtr& msg);
    void RegistrationResidualCheck();
    void TRECalculationCallBack(const std_msgs::String::ConstPtr& msg);
    void ICPCallBack(const std_msgs::String::ConstPtr& msg);

    // Robot operations
    void UpdateRobotConnFlagCallBack(const std_msgs::Bool::ConstPtr& msg);
    void RobConnectCallBack(const std_msgs::String::ConstPtr& msg);
    void RobDisconnectCallBack(const std_msgs::String::ConstPtr& msg);
    void GetJntsCallBack(const std_msgs::String::ConstPtr& msg);
    void GetEFFCallBack(const std_msgs::String::ConstPtr& msg);
    void ExecuteMotionToOffsetCallBack(const std_msgs::String::ConstPtr& msg);
    void ExecuteConfirmMotionCallBack(const std_msgs::String::ConstPtr& msg);
    void ExecuteMotionToTargetEFFPose();
    void ExecuteBackToCallBack(const std_msgs::String::ConstPtr& msg);
    void ExecuteManualAdjust(const std_msgs::Float32MultiArray::ConstPtr& msg);

    // Robot interface
    ros::Publisher pub_init_conn_       = n_.advertise<std_msgs::String>(
        "/RobInterface/Connection", 2);
    ros::Publisher pub_disconn_         = n_.advertise<std_msgs::String>(
        "/RobInterface/Connection", 2);
    ros::ServiceClient clt_jnt_         = n_.serviceClient<rotms_ros_msgs::GetJnts>(
        "/RobInterface/GetJntsPos");
    ros::ServiceClient clt_eff_         = n_.serviceClient<rotms_ros_msgs::GetEFF>(
        "/RobInterface/GetEFFPose");
    ros::Publisher pub_robeffmove_      = n_.advertise<geometry_msgs::Pose>(
        "/RobInterface/MoveEFF", 2);
    ros::Publisher pub_robjntmove_      = n_.advertise<std_msgs::Float32MultiArray>(
        "/RobInterface/MoveJnt", 2);

    // Temp data cache (volatile)
    struct TempDataCache datacache_;
    void ResetVolatileDataCacheLandmarks();
    void ResetVolatileDataCacheToolPose();

    // Utility
    void StateTransitionCheck(int new_state, std::string s);
    void ToolPoseTargetToXR();

    ros::Publisher pub_xr_ = n_.advertise<std_msgs::String>(
        "/XRComm/msg_to_send_hi_f", 2);

};
