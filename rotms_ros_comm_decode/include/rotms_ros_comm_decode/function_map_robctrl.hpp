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
#include <map>
#include "decode_node.hpp"



class CommDecoderRobCtrl : public CommDecoder
{

public:

    CommDecoderRobCtrl(
        ros::NodeHandle& n, 
        const std::string modulesuffix,
        FuncMap opsdict
        );

};

FuncMap GetFuncMapRobCtrl();

// Parse received messages into double vector
// Input format has to be num1_num2_num3_ (cannot be num1_num2_num3)
std::vector<double> ParseString2DoubleVec(std::string s);

void GetJntsAngs(std::string& ss, PublisherVec& pubs);
void GetEFFPose(std::string& ss, PublisherVec& pubs);
void SetCurJntsAsInit(std::string& ss, PublisherVec& pubs);

void ExecuteMotion(std::string& ss, PublisherVec& pubs);
void ExecuteMoveConfirm(std::string& ss, PublisherVec& pubs);
void ExecuteEndAndBack(std::string& ss, PublisherVec& pubs);
void ExecuteBackInit(std::string& ss, PublisherVec& pubs);
void ExecuteBackOffset(std::string& ss, PublisherVec& pubs);
void ExecuteRobotHoming(std::string& ss, PublisherVec& pubs);

void SessionReinit(std::string& ss, PublisherVec& pubs);

void ManualAdjustT(std::string& ss, PublisherVec& pubs);
void ManualAdjustR(std::string& ss, PublisherVec& pubs);

void ConnectRobot(std::string& ss, PublisherVec& pubs);
void DisconnectRobot(std::string& ss, PublisherVec& pubs);
