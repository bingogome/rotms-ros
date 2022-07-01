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
#include <vector>
#include "state_machine.hpp"

struct VolatileTempDataCache 
{
    int landmark_total = -1;
    std::vector<std::vector<double>> landmark_coords;
};

class Dispatcher
{

public:

    Dispatcher(ros::NodeHandle& n, const std::vector<WorkState*>& states);

private:

    ros::NodeHandle& n_;
    const std::vector<WorkState*>& states_;
    int activated_state_;

    // Dispatch signals
    ros::Subscriber sub_medimg_landmarkplanmeta_ = n_.subscribe(
        "/MedImg/LandmarkPlanMeta", 10, &Dispatcher::LandmarkPlanMetaCallBack, this);
    ros::Subscriber sub_medimg_landmarkplanfids_ = n_.subscribe(
        "/MedImg/LandmarkPlanFids", 10, &Dispatcher::LandmarkPlanFidsCallBack, this);
    ros::Subscriber sub_medimg_autodigitization_ = n_.subscribe(
        "/MedImg/StartAct", 10, &Dispatcher::AutodigitizationCallBack, this);
    ros::Subscriber sub_medimg_registration_ = n_.subscribe(
        "/MedImg/StartAct", 10, &Dispatcher::RegistrationCallBack, this);

    // Cruicial operations
    void LandmarkPlanMetaCallBack(const std_msgs::Int16::ConstPtr& msg);
    void AutodigitizationCallBack(const std_msgs::String::ConstPtr& msg);
    void RegistrationCallBack(const std_msgs::String::ConstPtr& msg);
    void RegistrationUsePrevCallBack(const std_msgs::String::ConstPtr& msg);

    // Secondary and intermediate operations
    void LandmarkPlanFidsCallBack(const std_msgs::Float32MultiArray::ConstPtr& msg);

    // Temp data cache (volatile)
    struct VolatileTempDataCache datacache_;
    void ResetVolatileDataCacheLandmarks();

};

void SaveLandmarkPlanData(struct VolatileTempDataCache datacache, std::string f);
std::string FormatDouble2String(double a, int dec);
std::string GetTimeString();
