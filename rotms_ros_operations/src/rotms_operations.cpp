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

#include "registration_funcs.hpp"
#include "rotms_operations.hpp"

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>

#include <tuple>
#include <vector>
#include <string>

#include <stdexcept>
#include <yaml-cpp/yaml.h>

TMSOperations::TMSOperations(ros::NodeHandle& n)
    : n_(n)
{}

void TMSOperations::OperationPlanLandmarks()
{
    // The operation has been done by dispatcher and cached to /share/cache
    // no need to call this anymore.
    // Perhaps future change 
}

void TMSOperations::OperationDigitization()
{
    // Get meta data of planned landmarks
    std::string packpath = ros::package::getPath("rotms_ros_operation");
    YAML::Node f = YAML::LoadFile(packpath + "/share/cache/landmarkplan.yaml");
    int num_of_landmarks = f["NUM"].as<int>();

    ROS_INFO_STREAM(num_of_landmarks);

    // // Beep the Polaris 3 times to indicate get prepared for digitization
    // ros::Publisher pub_beep = n_.advertise<std_msgs::Int32>("/NDI/beep", 10);
    // ros::Duration(3).sleep();
    // std_msgs::Int32 beep_num;
    // beep_num.data = 3;
    // pub_beep.publish(beep_num); 
    // ros::spinOnce();

    // // Wait 7 seconds to get prepared
    // ros::Duration(7).sleep();

    // // Start digitization
    // for(int i=0;i<digPntCld_.points.size();i++)
    // {
    //     beep_num.data = 2;
    //     ros::Duration(7).sleep();
    //     pub_beep.publish(beep_num); ros::spinOnce();
    //     geometry_msgs::TransformStampedConstPtr curdigPtr = ros::topic::waitForMessage<geometry_msgs::TransformStamped>("/TMSKuka/PtrtipWRTHeadRef");
    //     digPntCld_.points[i].x = curdigPtr->transform.translation.x;
    //     digPntCld_.points[i].y = curdigPtr->transform.translation.y;
    //     digPntCld_.points[i].z = curdigPtr->transform.translation.z;
    //     flag_dig_recvd_[i] = true;
    //     ROS_INFO("User digitized one point (#%d)", i);
    // }

}

void TMSOperations::OperationPlanToolPose()
{

}

void TMSOperations::OperationRegistration()
{
    std::string packpath = ros::package::getPath("rotms_ros_operation");
        
    // Read the points-pair from cache file
    YAML::Node f = YAML::LoadFile(packpath + "/share/cache/pointpair.yaml");
    YAML::Node ff1 = f["PLANNED"];
    YAML::Node ff2 = f["DIGITIZED"];
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
    
    if(cloudpln.size()!=clouddig.size())
        throw std::runtime_error(
            "Planned point cloud and the digitized point cloud have different size!");

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
    pub_registration_.publish(res);

    // Write the registration result to cache files
    SaveRegistrationData(quat, p, packpath + "/share/data/reg_" + GetTimeString() + ".yaml"); // record
    SaveRegistrationData(quat, p, packpath + "/share/config/reg" + ".yaml"); // use

}