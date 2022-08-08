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
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>

#include <tuple>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <cmath>

#include "operations_utility.hpp"
#include "registration_funcs.hpp"
#include "operations_registration.hpp"
#include "rotms_ros_msgs/PoseValid.h"
#include "ros_print_color.hpp"

OperationsRegistration::OperationsRegistration(ros::NodeHandle& n) : OperationsBase(n)
{}

void OperationsRegistration::OperationPlanLandmarks()
{
    // The operation of caching planned landmarks has been done by dispatcher and cached to /share/cache/landmarkplan.yaml

    // Only need to initialize /share/cache/landmarkdig.yaml
    // Note: have to be called after the landmarkplan.yaml is cached!

    std::string packpath = ros::package::getPath("rotms_ros_operations");
    YAML::Node f = YAML::LoadFile(packpath + "/share/cache/landmarkplan.yaml");
    std::string time_stamp = f["TIMESTAMP"].as<std::string>();
    std::string now_time = GetTimeString();
    double diff_t = GetTimeDiff(time_stamp, now_time);
    if ( !(diff_t>0 && diff_t<1) )
    {
        ROS_YELLOW_STREAM("[ROTMS WARNING] Time stamp of planned landmarks is not recent! Failed to initialize landmarkdig!");
        return;
    }

    // Init landmarkdig.yaml
    int num_of_landmarks = f["NUM"].as<int>();

    YAML::Node f2 = YAML::LoadFile(packpath + "/share/cache/landmarkdig.yaml");
    int num_of_landmarks2 = f2["NUM"].as<int>();

    if (num_of_landmarks!=num_of_landmarks2)
    {
        std::ofstream filesave(packpath + "/share/cache/landmarkdig.yaml");
        if(filesave.is_open())
        {
            filesave << "NUM: " << num_of_landmarks << "\n";
            filesave << "\n";
            filesave << "DIGITIZED: # in m\n";
            filesave << "\n";
            filesave << "  {\n";
            for(int i=0;i<num_of_landmarks;i++)
            {
                filesave << "    p" << i << ": ";
                filesave << "{";
                filesave << "x: " << nan("1") << ", ";
                filesave << "y: " << nan("1") << ", ";
                filesave << "z: " << nan("1");
                if (i==num_of_landmarks-1)
                    filesave << "}\n";
                else
                    filesave << "},\n";
            }
            filesave << "  }\n";
            filesave << " \n";
            filesave << "TIMESTAMP: " << GetTimeString() << "\n";
            filesave.close();
        }
    }
}

void OperationsRegistration::OperationRegistration()
{
    std::string packpath = ros::package::getPath("rotms_ros_operations");
        
    // Read the points-pair from cache file
    YAML::Node f1 = YAML::LoadFile(packpath + "/share/cache/landmarkplan.yaml");
    YAML::Node f2 = YAML::LoadFile(packpath + "/share/cache/landmarkdig.yaml");
    YAML::Node ff1 = f1["PLANNED"];
    YAML::Node ff2 = f2["DIGITIZED"];
    std::vector<std::vector<double>> cloudpln;
    std::vector<std::vector<double>> clouddig;
    
    for(YAML::const_iterator it=ff1.begin(); it!=ff1.end(); ++it)
    {
        YAML::Node value = it->second;
        std::vector<double> temppnt = {
            value["x"].as<double>(),
            value["y"].as<double>(),
            value["z"].as<double>()
        };
        cloudpln.push_back(temppnt);
    }

    for(YAML::const_iterator it=ff2.begin(); it!=ff2.end(); ++it)
    {
        YAML::Node value = it->second;
        std::vector<double> temppnt = {
            value["x"].as<double>(),
            value["y"].as<double>(),
            value["z"].as<double>()
        };
        clouddig.push_back(temppnt);
    }
    ROS_GREEN_STREAM("[ROTMS INFO] Planned landmark size: " + std::to_string(cloudpln.size()));
    ROS_GREEN_STREAM("[ROTMS INFO] Digitized landmark size: " + std::to_string(clouddig.size()));
    if(cloudpln.size()!=clouddig.size())
        throw std::runtime_error(
            "Planned point cloud and the digitized point cloud have different size! [Read from cache]");

    // Perform registration algorithm
    std::tuple<std::vector<std::vector<double>>, std::vector<double>> reg = 
        getRegistrationResult(cloudpln,clouddig);
    std::vector<std::vector<double>> R = std::get<0>(reg);
    std::vector<double> p = std::get<1>(reg);

    // Convert to quaternion
    geometry_msgs::Pose res;
    std::vector<double> quat = rotm2quat(R);

    // Publish the registration result to rot
    res.position.x = p[0]; res.position.y = p[1]; res.position.z = p[2];
    res.orientation.x = quat[0]; 
    res.orientation.y = quat[1]; 
    res.orientation.z = quat[2]; 
    res.orientation.w = quat[3]; 
    rotms_ros_msgs::PoseValid pv;
    pv.valid = true;
    pv.pose = res;
    pub_registration_.publish(pv);

    // Write the registration result to cache files
    SaveRegistrationData(quat, p, packpath + "/share/data/reg_" + GetTimeString() + ".yaml"); // record
    SaveRegistrationData(quat, p, packpath + "/share/config/reg" + ".yaml"); // use

}

void OperationsRegistration::OperationResetRegistration()
{
    rotms_ros_msgs::PoseValid pv;
    pv.valid = false;
    pub_registration_.publish(pv);
}

void OperationsRegistration::OperationUsePreRegistration()
{
    std::string packpath = ros::package::getPath("rotms_ros_operations");
        
    // Read the registration from cache file
    YAML::Node f = YAML::LoadFile(packpath + "/share/config/reg.yaml");
    YAML::Node ft = f["TRANSLATION"];
    YAML::Node fr = f["ROTATION"];

    rotms_ros_msgs::PoseValid pv;
    pv.valid = true;
    pv.pose.position.x = ft["x"].as<double>();
    pv.pose.position.y = ft["y"].as<double>();
    pv.pose.position.z = ft["z"].as<double>();
    pv.pose.orientation.x = fr["x"].as<double>();
    pv.pose.orientation.y = fr["y"].as<double>();
    pv.pose.orientation.z = fr["z"].as<double>();
    pv.pose.orientation.w = fr["w"].as<double>();
    pub_registration_.publish(pv);
}
