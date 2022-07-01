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

#include "rotms_dispatcher.hpp"
#include "state_machine.hpp"
#include "state_machine_states.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <iostream>

/*
Dispatcher takes decoded messages from comm_decode and preprocesses
volatile data that have "receiving bursts" (eg. The operation plan 
landmarks will receive NUM_LANDMARKS messages at one request. These 
messages will be preprocessed by dispatcher and cached in /share/cache 
for state/flag/operation for postprocessing.
On the other hand, state/flag/operation will postprocess any data 
that have been preprocessed and cached.
*/
Dispatcher::Dispatcher(ros::NodeHandle& n, const std::vector<WorkState*>& states) 
    : n_(n), states_(states)
{
    bool integ = CheckFlagIntegrity(states_);
    ROS_INFO_STREAM("Flag integrity check: " + std::to_string(integ));
}

void Dispatcher::LandmarkPlanMetaCallBack(const std_msgs::Int16::ConstPtr& msg)
{
    if (msg->data==-99) // signals all landmarks plan have been received.
    {
        std::string packpath = ros::package::getPath("rotms_ros_operations");

        if (datacache_.landmark_coords.size() != datacache_.landmark_total)
        {
            ROS_INFO("The number of landmarks received does not match planned!");
            ROS_INFO_STREAM("Planned: " + std::to_string(datacache_.landmark_total));
            ROS_INFO_STREAM("Received: " + std::to_string(datacache_.landmark_coords.size()));
            ROS_INFO("Try one more time!");
            Dispatcher::ResetVolatileDataCacheLandmarks();
            return;
        }

        SaveLandmarkPlanData(datacache_, packpath + "/share/cache/landmarkplan.yaml");
        SaveLandmarkPlanData(datacache_, packpath + "/share/data/landmarkplan_" + GetTimeString() + ".yaml");

        ROS_INFO("Landmarks cached.");

        Dispatcher::ResetVolatileDataCacheLandmarks(); // reset

        int new_state = states_[activated_state_]->LandmarksPlanned();
        ROS_INFO_STREAM("Old state: " + std::to_string(activated_state_));
        ROS_INFO_STREAM("Attempt new state: " + std::to_string(new_state));
        if (new_state != -1)
        {
            activated_state_ = new_state;
            ROS_INFO_STREAM("Transitioned to new state: " + 
                std::to_string(activated_state_));
        }
        else
        {
            // Failed operation
            ROS_INFO("State transition not possible.");
            ROS_INFO("Make sure the operation dependencies are met.");
        }
    }
    else
    {
        datacache_.landmark_total = msg->data;
    }
}

void Dispatcher::LandmarkPlanFidsCallBack(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data[0]==datacache_.landmark_coords.size())
    {
        std::vector<double> cur_fid{
            msg->data[1],msg->data[2],msg->data[3],};
        datacache_.landmark_coords.push_back(cur_fid);
    }
    else
    {
        ROS_INFO("Current index does not match the waiting index!");
        ROS_INFO_STREAM("Current index: " + std::to_string(msg->data[0]));
        ROS_INFO_STREAM("Waiting index: " + std::to_string(datacache_.landmark_coords.size()));
        ROS_INFO("Try one more time!");
        Dispatcher::ResetVolatileDataCacheLandmarks();
    }
}

void Dispatcher::ResetVolatileDataCacheLandmarks()
{
    datacache_.landmark_total = -1;
    datacache_.landmark_coords.clear();
}

void Dispatcher::AutodigitizationCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data.compare("_autodigitize__")==0)
    {
        int new_state = states_[activated_state_]->LandmarksDigitized();
        ROS_INFO_STREAM("Old state: " + std::to_string(activated_state_));
        ROS_INFO_STREAM("Attempt new state: " + std::to_string(new_state));
        if (new_state != -1)
        {
            activated_state_ = new_state;
            ROS_INFO_STREAM("Transitioned to new state: " + 
                std::to_string(activated_state_));
        }
        else
        {
            // Failed operation
            ROS_INFO("State transition not possible.");
            ROS_INFO("Make sure the operation dependencies are met.");
        }
    }
}

void Dispatcher::RegistrationCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data.compare("_register__")==0)
    {
        int new_state = states_[activated_state_]->Registered();
        ROS_INFO_STREAM("Old state: " + std::to_string(activated_state_));
        ROS_INFO_STREAM("Attempt new state: " + std::to_string(new_state));
        if (new_state != -1)
        {
            activated_state_ = new_state;
            ROS_INFO_STREAM("Transitioned to new state: " + 
                std::to_string(activated_state_));
        }
        else
        {
            // Failed operation
            ROS_INFO("State transition not possible.");
            ROS_INFO("Make sure the operation dependencies are met.");
        }
    }
    
}

void Dispatcher::ToolPoseOrientCallBack(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    std::vector<double> toolpose_r{
        msg->x, msg->y, msg->z, msg->w
    };
    datacache_.toolpose_r = toolpose_r;
    datacache_.toolpose_r_recvd = true;
    if (datacache_.toolpose_t_recvd == true)
    {
        
        std::string packpath = ros::package::getPath("rotms_ros_operations");
        SaveToolPoseData(datacache_, packpath + "/share/config/toolpose.yaml");
        SaveToolPoseData(datacache_, packpath + "/share/data/toolpose_" + GetTimeString() + ".yaml");

        ROS_INFO("Toolpose cached.");
        Dispatcher::ResetVolatileDataCacheToolPose();

        int new_state = states_[activated_state_]->ToolPosePlanned();
        ROS_INFO_STREAM("Old state: " + std::to_string(activated_state_));
        ROS_INFO_STREAM("Attempt new state: " + std::to_string(new_state));
        if (new_state != -1)
        {
            activated_state_ = new_state;
            ROS_INFO_STREAM("Transitioned to new state: " + 
                std::to_string(activated_state_));
        }
        else
        {
            // Failed operation
            ROS_INFO("State transition not possible.");
            ROS_INFO("Make sure the operation dependencies are met.");
        }
    }
}

void Dispatcher::ToolPoseTransCallBack(const geometry_msgs::Point::ConstPtr& msg)
{
    std::vector<double> toolpose_t{
        msg->x, msg->y, msg->z
    };
    datacache_.toolpose_t = toolpose_t;
    datacache_.toolpose_t_recvd = true;
    if (datacache_.toolpose_r_recvd == true)
    {
        std::string packpath = ros::package::getPath("rotms_ros_operations");
        SaveToolPoseData(datacache_, packpath + "/share/config/toolpose.yaml");
        SaveToolPoseData(datacache_, packpath + "/share/data/toolpose_" + GetTimeString() + ".yaml");

        ROS_INFO("Toolpose cached.");
        Dispatcher::ResetVolatileDataCacheToolPose();

        int new_state = states_[activated_state_]->ToolPosePlanned();
        ROS_INFO_STREAM("Old state: " + std::to_string(activated_state_));
        ROS_INFO_STREAM("Attempt new state: " + std::to_string(new_state));
        if (new_state != -1)
        {
            activated_state_ = new_state;
            ROS_INFO_STREAM("Transitioned to new state: " + 
                std::to_string(activated_state_));
        }
        else
        {
            // Failed operation
            ROS_INFO("State transition not possible.");
            ROS_INFO("Make sure the operation dependencies are met.");
        }
    }
}

void Dispatcher::ResetVolatileDataCacheToolPose()
{
    datacache_.toolpose_t_recvd = false;
    datacache_.toolpose_r_recvd = false;
    datacache_.toolpose_t.clear();
    datacache_.toolpose_r.clear();
}

void SaveLandmarkPlanData(struct VolatileTempDataCache datacache, std::string f)
{
    std::ofstream filesave(f);
    if(filesave.is_open())
    {
        filesave << "NUM: " << datacache.landmark_total << "\n";
        filesave << "\n";
        filesave << "PLANNED: # in m\n";
        filesave << "\n";
        filesave << "  {\n";
        for(int i=0;i<datacache.landmark_total;i++)
        {
            std::vector<std::vector<double>> c = datacache.landmark_coords;
            filesave << "    p" << i << ": ";
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

void SaveToolPoseData(struct VolatileTempDataCache datacache, std::string f)
{
    std::ofstream filesave(f);
    if(filesave.is_open())
    {
        filesave << "TRANSLATION: # translation: x,y,z\n";
        filesave << "\n";
        filesave << "  {\n";
		filesave << "    x: " << FormatDouble2String(datacache.toolpose_t[0], 16) << ",\n";
		filesave << "    y: " << FormatDouble2String(datacache.toolpose_t[1], 16) << ",\n";
		filesave << "    z: " << FormatDouble2String(datacache.toolpose_t[2], 16) << "\n";
		filesave << "  }\n";
        filesave << "\n";
		filesave << "ROTATION: # quat: x,y,z,w\n";
        filesave << "\n";
        filesave << "  {\n";
		filesave << "    x: " << FormatDouble2String(datacache.toolpose_r[0], 16) << ",\n";
		filesave << "    y: " << FormatDouble2String(datacache.toolpose_r[1], 16) << ",\n";
		filesave << "    z: " << FormatDouble2String(datacache.toolpose_r[2], 16) << ",\n";
        filesave << "    w: " << FormatDouble2String(datacache.toolpose_r[3], 16) << "\n";
        filesave << "  }\n";
		filesave.close();
    }
}

std::string FormatDouble2String(double a, int dec)
{
	std::stringstream stream;
    stream << std::fixed << std::setprecision(dec) << a;
    std::string s = stream.str();
    return s;
}

std::string GetTimeString()
{
	time_t rawtime;
	struct tm * timeinfo;
	char buffer [80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime (buffer,80,"%Y%m%d_%I%M%p",timeinfo);

	return buffer;
}