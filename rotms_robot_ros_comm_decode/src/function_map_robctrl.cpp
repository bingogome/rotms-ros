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
#include "function_map_robctrl.hpp"
#include "decode_node.hpp"

/**
* This maps the functions to the received cmd.
*/

CommDecoderRobCtrl::CommDecoderRobCtrl(
    ros::NodeHandle& n, 
    const std::string modulesuffix,
    FuncMap opsdict) 
    : 
    CommDecoder(n, modulesuffix, opsdict) 
{
    pubs_.push_back(
        n_.advertise<std_msgs::String>("/RobCtrl/GetInfo", 5));
    pubs_.push_back(
        n_.advertise<std_msgs::String>("/RobCtrl/Motion", 5));
    pubs_.push_back(
        n_.advertise<std_msgs::String>("/RobCtrl/Adjust", 5));
    pubs_.push_back(
        n_.advertise<std_msgs::String>("/RobCtrl/Session", 5));
}

FuncMap GetFuncMapRobCtrl()
{
    FuncMap fm;

    fm["GET_JNT_ANGS"] = GetJntAngs;
    fm["GET_EFF_POSE"] = GetEffPose;

    fm["EXECUTE_MOTION"] = ExecuteMotion;
    fm["EXECUTE_MOVE_CONFIRM"] = ExecuteMoveConfirm;

    fm["SESSION_END"] = SessionEnd;

    fm["MAN_ADJUST_BACKWARD"] = ManualAdjustBackwards;
    fm["MAN_ADJUST_CLOSER"] = ManualAdjustApproach;
    fm["MAN_ADJUST_FARTHER"] = ManualAdjustAway;
    fm["MAN_ADJUST_FORWARD"] = ManualAdjustForward;
    fm["MAN_ADJUST_LEFT"] = ManualAdjustLeft;
    fm["MAN_ADJUST_PITCH"] = ManualAdjustPitch;
    fm["MAN_ADJUST_RIGHT"] = ManualAdjustRight;
    fm["MAN_ADJUST_ROLL"] = ManualAdjustRoll;
    fm["MAN_ADJUST_YAW"] = ManualAdjustYaw;
    return fm;
}

void GetJntAngs(std::string& ss, PublisherVec& pubs)
{
    std_msgs::String msg_test;
    msg_test.data = "JNTS";
    // pubs[0] is the publisher /RobCtrl/GetInfo
    pubs[0].publish(msg_test);
}

void GetEffPose(std::string& ss, PublisherVec& pubs)
{
    std_msgs::String msg_test;
    msg_test.data = "EFF";
    // pubs[0] is the publisher /RobCtrl/GetInfo
    pubs[0].publish(msg_test);
}

void ExecuteMotion(std::string& ss, PublisherVec& pubs)
{

}

void ExecuteMoveConfirm(std::string& ss, PublisherVec& pubs)
{

}

void SessionEnd(std::string& ss, PublisherVec& pubs)
{

}

void ManualAdjustBackwards(std::string& ss, PublisherVec& pubs)
{

}

void ManualAdjustApproach(std::string& ss, PublisherVec& pubs)
{

}

void ManualAdjustAway(std::string& ss, PublisherVec& pubs)
{

}

void ManualAdjustForward(std::string& ss, PublisherVec& pubs)
{

}

void ManualAdjustLeft(std::string& ss, PublisherVec& pubs)
{

}

void ManualAdjustPitch(std::string& ss, PublisherVec& pubs)
{

}

void ManualAdjustRight(std::string& ss, PublisherVec& pubs)
{

}

void ManualAdjustRoll(std::string& ss, PublisherVec& pubs)
{

}

void ManualAdjustYaw(std::string& ss, PublisherVec& pubs)
{

}