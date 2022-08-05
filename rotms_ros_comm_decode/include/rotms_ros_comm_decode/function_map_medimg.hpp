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

class CommDecoderMedImg : public CommDecoder
{

public:

    CommDecoderMedImg(
        ros::NodeHandle& n, 
        const std::string modulesuffix,
        FuncMap opsdict
        );

};

FuncMap GetFuncMapMedImg();

void StartAutoDigitize(std::string& ss, PublisherVec& pubs);

void StartRegistration(std::string& ss, PublisherVec& pubs);

void StartUsePrevRegistration(std::string& ss, PublisherVec& pubs);

void StartTRECalculation(std::string& ss, PublisherVec& pubs);

void StopTRECalculation(std::string& ss, PublisherVec& pubs);

void LandmarkDigitizeIndividual(std::string& ss, PublisherVec& pubs);

void LandmarkCurrentOnImg(std::string& ss, PublisherVec& pubs);

void LandmarkNumOnImg(std::string& ss, PublisherVec& pubs);

void LandmarkLastReceived(std::string& ss, PublisherVec& pubs);

void TargetPoseOrientation(std::string& ss, PublisherVec& pubs);

void TargetPoseTranslation(std::string& ss, PublisherVec& pubs);