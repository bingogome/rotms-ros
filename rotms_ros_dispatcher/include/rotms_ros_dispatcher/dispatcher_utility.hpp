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
#include <string>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <tf2/LinearMath/Transform.h>

struct TempDataCache 
{
    int landmark_total = -1;
    std::vector<std::vector<double>> landmark_coords;

    bool toolpose_t_recvd = false;
    bool toolpose_r_recvd = false;
    std::vector<double> toolpose_t;
    std::vector<double> toolpose_r;
};

void SaveLandmarkPlanData(struct TempDataCache datacache, std::string f, std::string time_stamp);
void SaveToolPoseData(struct TempDataCache datacache, std::string f);
void SaveCurrentJntsAsInit(std_msgs::Float32MultiArray jnts, std::string f);
std::vector<double> ReadJntsFromConfig(std::string f);
std::string FormatDouble2String(double a, int dec);
std::string GetTimeString();
std::vector<tf2::Vector3> ReadPointCloudFromYAML(std::string f, std::string pnt);
tf2::Transform ReadTransformFromYAML(std::string f);
double GetPairPointResidual(
    tf2::Transform tr, std::vector<tf2::Vector3> A, std::vector<tf2::Vector3> B);
std::vector<double> quat2eul(std::vector<double> q /*x,y,z,w*/);
std::vector<double> eul2quat(std::vector<double> eul);
