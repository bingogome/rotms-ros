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
#include <std_msgs/Float32MultiArray.h>
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
        n_.advertise<std_msgs::String>("/RobCtrl/GetInfo", 2));
    pubs_.push_back(
        n_.advertise<std_msgs::String>("/RobCtrl/Motion", 2));
    pubs_.push_back(
        n_.advertise<std_msgs::Float32MultiArray>("/RobCtrl/MotionAdjust", 2));
    pubs_.push_back(
        n_.advertise<std_msgs::String>("/RobCtrl/Session", 2));
    pubs_.push_back(
        n_.advertise<std_msgs::String>("/RobCtrl/RobConnection", 2));
}

FuncMap GetFuncMapRobCtrl()
{
    FuncMap fm;

    fm["GET_JNT_ANGS"] = GetJntsAngs;
    fm["GET_EFF_POSE"] = GetEFFPose;
    fm["GET_JNT_ANGS_TOINIT"] = SetCurJntsAsInit;

    fm["EXECUTE_MOTION"] = ExecuteMotion;
    fm["EXECUTE_MOVE_CONFIRM"] = ExecuteMoveConfirm;
    fm["EXECUTE_ENDBACK"] = ExecuteEndAndBack;
    fm["EXECUTE_BACKINIT"] = ExecuteBackInit;
    fm["EXECUTE_BACKOFFSET"] = ExecuteBackOffset;
    fm["EXECUTE_ROB_HOMING"] = ExecuteRobotHoming;

    fm["SESSION_REINIT"] = SessionReinit;

    fm["MAN_ADJUST_T"] = ManualAdjustT;
    fm["MAN_ADJUST_R"] = ManualAdjustR;

    fm["ROB_CONN_ON"] = ConnectRobot;
    fm["ROB_CONN_OFF"] = DisconnectRobot;

    return fm;
}

// Parse received messages into double vector
// Input format has to be num1_num2_num3_ (cannot be num1_num2_num3)
std::vector<double> ParseString2DoubleVec(std::string s)
{
	std::vector<double> vec;
	std::string delimiter = "_";
	size_t pos = 0;
	while ((pos = s.find(delimiter)) != std::string::npos) 
	{
	    vec.push_back(std::stod(s.substr(0, pos)));
	    s.erase(0, pos + delimiter.length());
	}
	return vec;
}

void GetJntsAngs(std::string& ss, PublisherVec& pubs)
{
    std_msgs::String msg_test;
    msg_test.data = "_jnts__";
    // pubs[0] is the publisher /RobCtrl/GetInfo
    pubs[0].publish(msg_test);
}

void GetEFFPose(std::string& ss, PublisherVec& pubs)
{
    std_msgs::String msg_test;
    msg_test.data = "_eff__";
    // pubs[0] is the publisher /RobCtrl/GetInfo
    pubs[0].publish(msg_test);
}

void SetCurJntsAsInit(std::string& ss, PublisherVec& pubs)
{
    std_msgs::String msg_test;
    msg_test.data = "_initcur__";
    // pubs[0] is the publisher /RobCtrl/GetInfo
    pubs[0].publish(msg_test);
}

void ExecuteMotion(std::string& ss, PublisherVec& pubs)
{
    std_msgs::String msg_test;
    msg_test.data = "_execute__";
    // pubs[1] is the publisher /RobCtrl/Motion
    pubs[1].publish(msg_test);
}

void ExecuteMoveConfirm(std::string& ss, PublisherVec& pubs)
{
    std_msgs::String msg_test;
    msg_test.data = "_confirm__";
    // pubs[1] is the publisher /RobCtrl/Motion
    pubs[1].publish(msg_test);
}

void ExecuteEndAndBack(std::string& ss, PublisherVec& pubs)
{
    SessionReinit(ss, pubs);
    ExecuteBackInit(ss, pubs);
}

void ExecuteBackInit(std::string& ss, PublisherVec& pubs)
{
    std_msgs::String msg_test;
    msg_test.data = "_backinit__";
    // pubs[1] is the publisher /RobCtrl/Motion
    pubs[1].publish(msg_test);
}

void ExecuteBackOffset(std::string& ss, PublisherVec& pubs)
{
    std_msgs::String msg_test;
    msg_test.data = "_backoffset__";
    // pubs[1] is the publisher /RobCtrl/Motion
    pubs[1].publish(msg_test);
}

void ExecuteRobotHoming(std::string& ss, PublisherVec& pubs)
{
    std_msgs::String msg_test;
    msg_test.data = "_robothoming__";
    // pubs[1] is the publisher /RobCtrl/Motion
    pubs[1].publish(msg_test);
}

void SessionReinit(std::string& ss, PublisherVec& pubs)
{
    std_msgs::String msg_test;
    msg_test.data = "_reinit__";
    // pubs[3] is the publisher /RobCtrl/Session
    pubs[3].publish(msg_test);
}

void ManualAdjustT(std::string& ss, PublisherVec& pubs)
{
    std::vector<double>  vec = ParseString2DoubleVec(ss+"_0.0_0.0_0.0_");
    std_msgs::Float32MultiArray msg;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = 6;
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "format__x_y_z_rz_ry_rx"; // euler angles
    msg.data.clear();
    msg.data.insert(msg.data.end(), vec.begin(), vec.end());
    // pubs[2] is the publisher /RobCtrl/MotionAdjust
    pubs[2].publish(msg);
}

void ManualAdjustR(std::string& ss, PublisherVec& pubs)
{
    std::vector<double>  vec = ParseString2DoubleVec("0.0_0.0_0.0_"+ss+"_");
    std_msgs::Float32MultiArray msg;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = 6;
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "format__x_y_z_rz_ry_rx"; // euler angles
    msg.data.clear();
    msg.data.insert(msg.data.end(), vec.begin(), vec.end());
    // pubs[2] is the publisher /RobCtrl/MotionAdjust
    pubs[2].publish(msg);
}

void ConnectRobot(std::string& ss, PublisherVec& pubs)
{
    std_msgs::String msg_test;
    msg_test.data = "_connect__";
    // pubs[4] is the publisher /RobCtrl/RobConnection
    pubs[4].publish(msg_test);
}

void DisconnectRobot(std::string& ss, PublisherVec& pubs)
{
    std_msgs::String msg_test;
    msg_test.data = "_disconnect__";
    // pubs[4] is the publisher /RobCtrl/RobConnection
    pubs[4].publish(msg_test);
}
