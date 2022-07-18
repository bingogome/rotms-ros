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
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include "rotms_ros_msgs/PoseValid.h"


struct OpsVolatileTempDataCache 
{
    int landmark_total = -1;
    std::vector<std::vector<double>> landmarkdig;
};

class TMSOperations
{
public:

    TMSOperations(ros::NodeHandle& n);

    // Cruicial operations
    void OperationPlanLandmarks();
    void OperationDigitization();
    void OperationPlanToolPose();
    void OperationRegistration();

    void OperationResetRegistration();
    void OperationResetToolPose();
    void OperationUsePreRegistration();

    // Secondary and intermediate operations
    // void Operation();
    // void Operation();
    // void Operation();

private:

    ros::NodeHandle& n_;
    ros::Publisher pub_registration_ = 
        n_.advertise<rotms_ros_msgs::PoseValid>("/Kinematics/TR_bodyref_body", 1, true);
    ros::Publisher pub_toolpose_ = 
        n_.advertise<rotms_ros_msgs::PoseValid>("/Kinematics/TR_body_cntct", 1, true);
    ros::Publisher pub_run_opttracker_tr_bodyref_ptrtip_ = 
        n_.advertise<std_msgs::String>("/Kinematics/Flag_bodyref_ptrtip", 2);

    struct OpsVolatileTempDataCache datacache_;
    void ResetOpsVolatileDataCache();

};

void SaveLandmarkDigData(struct OpsVolatileTempDataCache datacache, std::string f);