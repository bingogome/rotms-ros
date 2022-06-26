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

class TMSOperations
{
public:

    TMSOperations(ros::NodeHandle& n);

private:

    ros::NodeHandle& n_;
    ros::Publisher pub_registration_ = 
        n_.advertise<geometry_msgs::Pose>("/Rotms/DataCache/Update/Registration", 5);
    ros::Publisher pub_toolpose_ = 
        n_.advertise<geometry_msgs::Pose>("/Rotms/DataCache/Update/ToolPose", 5);

    // Cruicial operations
    void OperationPlanLandmarks();
    void OperationDigitization();
    void OperationPlanToolPose();
    void OperationRegistration();

    // Secondary and intermediate operations
    // void Operation();
    // void Operation();
    // void Operation();
}