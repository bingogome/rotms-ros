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
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <tuple>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include "registration_funcs.hpp"
#include "operations_tms.hpp"
#include "rotms_ros_msgs/PoseValid.h"
#include "ros_print_color.hpp"

OperationsTMS::OperationsTMS(ros::NodeHandle& n) : OperationsBase(n)
{}

void OperationsTMS::OperationPlanLandmarks()
{
    // The operation has been done by dispatcher and cached to /share/cache
    // no need to call this anymore.
    // Perhaps future change 
}

void OperationsTMS::OperationDigitization()
{
    // Get meta data of planned landmarks
    std::string packpath = ros::package::getPath("rotms_ros_operations");
    YAML::Node f = YAML::LoadFile(packpath + "/share/cache/landmarkplan.yaml");
    int num_of_landmarks = f["NUM"].as<int>();

    ROS_INFO_STREAM(num_of_landmarks);
    datacache_.landmark_total = num_of_landmarks;

    // Poke opttracker_tr_bodyref_ptrtip node /Kinematics/Flag_bodyref_ptrtip
    std_msgs::String flag_start;
    flag_start.data = "_start__";
    pub_run_opttracker_tr_bodyref_ptrtip_.publish(flag_start);

    // Beep the Opttracker 3 times to indicate get prepared for digitization
    ros::Publisher pub_beep = n_.advertise<std_msgs::Int32>("/NDI/beep", 10);
    ros::Duration(3).sleep();
    std_msgs::Int32 beep_num;
    beep_num.data = 3;
    pub_beep.publish(beep_num); 
    ros::spinOnce();

    // Wait 7 seconds to get prepared
    ros::Duration(7).sleep();

    // Start digitization
    for(int i=0;i<num_of_landmarks;i++)
    {
        // Beep 2 times for each landmark
        beep_num.data = 2;
        ros::Duration(7).sleep();
        pub_beep.publish(beep_num); ros::spinOnce();
        // Wait for a 
        geometry_msgs::PointConstPtr curdigPtr = ros::topic::waitForMessage<geometry_msgs::Point>("/Kinematics/T_bodyref_ptrtip");
        std::vector<double> curlandmark{curdigPtr->x, curdigPtr->y, curdigPtr->z};
        datacache_.landmarkdig.push_back(curlandmark);
        ROS_GREEN_STREAM("[ROTMS INFO] User digitized one point " + std::to_string(i));
    }

    // Poke opttracker_tr_bodyref_ptrtip node /Kinematics/Flag_bodyref_ptrtip
    flag_start.data = "_end__";
    pub_run_opttracker_tr_bodyref_ptrtip_.publish(flag_start);

    // Check validity and save
    if (datacache_.landmarkdig.size()!=num_of_landmarks)
    {
        ResetOpsVolatileDataCache();
        throw std::runtime_error(
            "Number of the digitized landmarks does not match!");
    }
    else
    {
        SaveLandmarkDigData(datacache_, 
            packpath + "/share/data/landmarkdig_"+ GetTimeString() + ".yaml");
        SaveLandmarkDigData(datacache_, 
            packpath + "/share/cache/landmarkdig" + ".yaml");
    }

}

void OperationsTMS::OperationRegistration()
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

void OperationsTMS::OperationResetRegistration()
{
    rotms_ros_msgs::PoseValid pv;
    pv.valid = false;
    pub_registration_.publish(pv);
}

void OperationsTMS::OperationUsePreRegistration()
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

void OperationsTMS::ResetOpsVolatileDataCache()
{
    datacache_.landmark_total = -1;
    datacache_.landmarkdig.clear();
}

void SaveLandmarkDigData(struct TempDataCacheOps datacache, std::string f)
{
    std::ofstream filesave(f);
    if(filesave.is_open())
    {
        filesave << "NUM: " << datacache.landmark_total << "\n";
        filesave << "\n";
        filesave << "DIGITIZED: # in m\n";
        filesave << "\n";
        filesave << "  {\n";
        for(int i=0;i<datacache.landmark_total;i++)
        {
            std::vector<std::vector<double>> c = datacache.landmarkdig;
            filesave << "    d" << i << ": ";
            filesave << "{";
            filesave << "x: " << FormatDouble2String(c[i][0], 16) << ", ";
            filesave << "y: " << FormatDouble2String(c[i][1], 16) << ", ";
            filesave << "z: " << FormatDouble2String(c[i][2], 16);
            if (i==datacache.landmark_total-1)
                filesave << "}\n";
            else
                filesave << "},\n";
        }
        filesave << "  }\n";
        filesave.close();
    }
}