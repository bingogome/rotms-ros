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

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include "operations.hpp"
#include "operations_toolplan.hpp"

OperationsToolplan::OperationsToolplan(ros::NodeHandle& n) : OperationsBase(n)
{}

void OperationsToolplan::OperationPlanToolPose()
{
    // The operation has been done by dispatcher and cached to /share/config
    // Only need to publish
    std::string packpath = ros::package::getPath("rotms_ros_operations");
    YAML::Node f = YAML::LoadFile(packpath + "/share/config/toolpose.yaml");
    YAML::Node ff1 = f["TRANSLATION"];
    YAML::Node ff2 = f["ROTATION"];
    geometry_msgs::Pose tr;
    tr.position.x = ff1["x"].as<double>();
    tr.position.y = ff1["y"].as<double>();
    tr.position.z = ff1["z"].as<double>();
    tr.orientation.x = ff2["x"].as<double>();
    tr.orientation.y = ff2["y"].as<double>();
    tr.orientation.z = ff2["z"].as<double>();
    tr.orientation.w = ff2["w"].as<double>();
    rotms_ros_msgs::PoseValid pv;
    pv.valid = true;
    pv.pose = tr;
    pub_toolpose_.publish(pv);
}

void OperationsToolplan::OperationResetToolPose()
{
    rotms_ros_msgs::PoseValid pv;
    pv.valid = false;
    pub_toolpose_.publish(pv);
}