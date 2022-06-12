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

#include <map>
#include <string>
#include <ros/ros.h>
#include "function_map_medimg.hpp"
#include "decode_node.hpp"

/**
* This maps the functions to the received cmd.
*/

CommDecoderMedImg::CommDecoderMedImg(
    ros::NodeHandle& n, 
    const std::string modulesuffix,
    FuncMap opsdict) 
    : 
    CommDecoder(n, modulesuffix, opsdict) 
{
    pubs_.push_back(
        n_.advertise<std_msgs::String>("/MedImg/StartAct", 5));
    pubs_.push_back(
        n_.advertise<std_msgs::String>("/MedImg/TargetPlan", 5));
    pubs_.push_back(
        n_.advertise<std_msgs::String>("/MedImg/FiducialPlan", 5));
}

FuncMap GetFuncMapMedImg()
{
    FuncMap fm;

    fm["START_AUTO_DIGITIZE"] = StartAutoDigitize;
    fm["START_REGISTRATION"] = StartRegistration;
    fm["START_USE_PREV_REGISTRATION"] = StartUsePrevRegistration;

    fm["FIDUCIAL_CURRENT_ON_IMG"] = FiducialCurrentOnImg;
    fm["FIDUCIAL_NUM_OF_ON_IMG"] = FiducialNumOnImg;

    fm["TARGET_POSE_ORIENTATION"] = TargetPoseOrientation;
    fm["TARGET_POSE_TRANSLATION"] = TargetPoseTranslation;

    return fm;
}

void StartAutoDigitize(std::string& ss, PublisherVec& pubs)
{

}

void StartRegistration(std::string& ss, PublisherVec& pubs)
{
    
}

void StartUsePrevRegistration(std::string& ss, PublisherVec& pubs)
{
    
}

void FiducialCurrentOnImg(std::string& ss, PublisherVec& pubs)
{
    
}

void FiducialNumOnImg(std::string& ss, PublisherVec& pubs)
{
    
}

void TargetPoseOrientation(std::string& ss, PublisherVec& pubs)
{
    
}

void TargetPoseTranslation(std::string& ss, PublisherVec& pubs)
{
    
}