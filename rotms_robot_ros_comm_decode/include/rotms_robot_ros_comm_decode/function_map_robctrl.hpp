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

class CommDecoderPubsRobCtrl : public CommDecoderPubs
{
public:
    CommDecoderPubsRobCtrl();
    int a = 1;
};

const std::map<std::string, void(*)(CommDecoderPubs&, std::string&)> GetFuncMapRobCtrl();

void GetJntAngs(CommDecoderPubs& pubs, std::string& ss);

void GetEffPose(CommDecoderPubs& pubs, std::string& ss);

void ExecuteMotion(CommDecoderPubs& pubs, std::string& ss);

void ExecuteMoveConfirm(CommDecoderPubs& pubs, std::string& ss);

void SessionEnd(CommDecoderPubs& pubs, std::string& ss);

void ManualAdjustBackwards(CommDecoderPubs& pubs, std::string& ss);

void ManualAdjustApproach(CommDecoderPubs& pubs, std::string& ss);

void ManualAdjustAway(CommDecoderPubs& pubs, std::string& ss);

void ManualAdjustForward(CommDecoderPubs& pubs, std::string& ss);

void ManualAdjustLeft(CommDecoderPubs& pubs, std::string& ss);

void ManualAdjustPitch(CommDecoderPubs& pubs, std::string& ss);

void ManualAdjustRight(CommDecoderPubs& pubs, std::string& ss);

void ManualAdjustRoll(CommDecoderPubs& pubs, std::string& ss);

void ManualAdjustYaw(CommDecoderPubs& pubs, std::string& ss);