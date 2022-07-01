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
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

#include "function_map_medimg.hpp"
#include "decode_node.hpp"

std::vector<double> SubStringTokenize2Double(std::string s, std::string del = "_")
{   
    std::vector<double> ans;
    int start = 0;
    int end = s.find(del);
    while (end != -1) {
        ans.push_back(
            std::stod(s.substr(start, end - start)));
        start = end + del.size();
        end = s.find(del, start);
    }
    ans.push_back(
        std::stod(s.substr(start, end - start)));
    return ans;
}

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
        n_.advertise<std_msgs::String>("/MedImg/StartAct", 2));
    pubs_.push_back(
        n_.advertise<std_msgs::Int16>("/MedImg/LandmarkPlanMeta", 2));
    pubs_.push_back(
        n_.advertise<std_msgs::Float32MultiArray>("/MedImg/LandmarkPlanFids", 10));
    pubs_.push_back(
        n_.advertise<geometry_msgs::Quaternion>("/MedImg/ToolPlanOrient", 2));
    pubs_.push_back(
        n_.advertise<geometry_msgs::Point>("/MedImg/ToolPlanTrans", 2));
}

FuncMap GetFuncMapMedImg()
{
    FuncMap fm;

    fm["START_AUTO_DIGITIZE"] = StartAutoDigitize;
    fm["START_REGISTRATION"] = StartRegistration;
    fm["START_USE_PREV_REGISTRATION"] = StartUsePrevRegistration;

    fm["LANDMARK_CURRENT_ON_IMG"] = LandmarkCurrentOnImg;
    fm["LANDMARK_NUM_OF_ON_IMG"] = LandmarkNumOnImg;
    fm["LANDMARK_LAST_RECEIVED"] = LandmarkLastReceived;

    fm["TARGET_POSE_ORIENTATION"] = TargetPoseOrientation;
    fm["TARGET_POSE_TRANSLATION"] = TargetPoseTranslation;

    return fm;
}

void StartAutoDigitize(std::string& ss, PublisherVec& pubs)
{
    // pubs[0] is the publisher /MedImg/StartAct
    std_msgs::String msg;
    msg.data = "_autodigitize__";
    pubs[0].publish(msg);
}

void StartRegistration(std::string& ss, PublisherVec& pubs)
{
    // pubs[0] is the publisher /MedImg/StartAct
    std_msgs::String msg;
    msg.data = "_register__";
    pubs[0].publish(msg);
}

void StartUsePrevRegistration(std::string& ss, PublisherVec& pubs)
{
    // pubs[0] is the publisher /MedImg/StartAct
    std_msgs::String msg;
    msg.data = "_prevregister__";
    pubs[0].publish(msg);
}

/* Landmark related functions:
The decoder receives 3 commands
1. Get the total number of landmarks
2. Get each landmark cooredinate
3. Notice the last landmark has been received
*/

void LandmarkCurrentOnImg(std::string& ss, PublisherVec& pubs)
{
    // format: {current_index, x, y, z}
    std::vector<double> fid_vec = SubStringTokenize2Double(ss, "_");

    std_msgs::Float32MultiArray msg;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = fid_vec.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "format__num_x_y_z";
    msg.data.clear();
    msg.data.insert(msg.data.end(), fid_vec.begin(), fid_vec.end());
    // pubs[2] is the publisher /MedImg/LandmarkPlanFids (fiducial)
    // Publish the current index of landmark, and the coordinate
    pubs[2].publish(msg);
}

void LandmarkNumOnImg(std::string& ss, PublisherVec& pubs)
{
    std_msgs::Int16 msg_test;
    msg_test.data = std::stoi(ss);
    // pubs[1] is the publisher /MedImg/LandmarkPlanMeta (meta data)
    // Publish number of landmarks (landmarks). (If it is not -99)
    pubs[1].publish(msg_test);
}

void LandmarkLastReceived(std::string& ss, PublisherVec& pubs)
{
    std_msgs::Int16 msg_test;
    msg_test.data = -99;
    // pubs[1] is the publisher /MedImg/LandmarkPlanMeta (meta data)
    // Publish -99 to the topic indicating the receiving of landmarks is complete
    pubs[1].publish(msg_test);
}

void TargetPoseOrientation(std::string& ss, PublisherVec& pubs)
{
    geometry_msgs::Quaternion quat;

    // format: { x, y, z, w }
    std::vector<double> quat_vec = SubStringTokenize2Double(ss, "_");

    quat.x = quat_vec[0];
    quat.y = quat_vec[1];
    quat.z = quat_vec[2];
    quat.w = quat_vec[3];

    // pubs[3] is the publisher /MedImg/ToolPlanOrient
    pubs[3].publish(quat);
}

void TargetPoseTranslation(std::string& ss, PublisherVec& pubs)
{
    geometry_msgs::Point p;

    // format: { x, y, z }
    std::vector<double> p_vec = SubStringTokenize2Double(ss, "_");

    p.x = p_vec[0];
    p.y = p_vec[1];
    p.z = p_vec[2];

    // pubs[4] is the publisher /MedImg/ToolPlanTrans
    pubs[4].publish(p);
}
