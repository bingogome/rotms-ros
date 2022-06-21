#include "rotms_dispatcher.hpp"
#include "registration_funcs.hpp"

#include <tuple>
#include <vector>
#include <string>

#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

Dispatcher::Dispatcher(ros::NodeHandle& n) : n_(n){}

// void Dispatcher::RegistrationCallBack(const std_msgs::String::ConstPtr& msg)
void Dispatcher::RegistrationCallBack()
{
    // if (msg->data.compare("_end_robot_connection_")==0)
    if (true)
    {
        std::string packpath = ros::package::getPath("rotms_ros_dispatcher");
        
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
    
}