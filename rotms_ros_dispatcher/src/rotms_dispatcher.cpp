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
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <iostream>


Dispatcher::Dispatcher(ros::NodeHandle& n, std::vector<WorkState>& states) 
    : n_(n), states_(states)
{

}

void Dispatcher::LandmarkPlanMetaCallBack(const std_msgs::Int16::ConstPtr& msg)
{
    if (msg->data==-99) // signals all landmarks plan have been received.
    {
        std::string packpath = ros::package::getPath("rotms_ros_operations");

        SaveLandmarkPlanData(datacache_, packpath + "/share/cache/landmarkplan.yaml");
        SaveLandmarkPlanData(datacache_, packpath + "/share/data/landmarkplan_" + GetTimeString() + ".yaml");

        ROS_INFO("Landmarks cached.");

        Dispatcher::ResetVolatileDataCache(); // reset
    }
    else
    {
        datacache_.landmark_total = msg->data;
    }
}

void Dispatcher::AutodigitizationCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data.compare("_autodigitize__")==0)
    {
        int new_state = states_[activated_state_].LandmarksDigitized();
        if (new_state != -1)
        {
            activated_state_ = new_state;
        }
        else
        {
            // Failed operation
            // TODO
        }
    }
}

void Dispatcher::RegistrationCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data.compare("_register__")==0)
    {
        int new_state = states_[activated_state_].Registered();
        if (new_state != -1)
        {
            activated_state_ = new_state;
        }
        else
        {
            // Failed operation
            // TODO
        }
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
        throw std::runtime_error(
            "Current index does not match the waiting index!");
    }
}

void Dispatcher::ResetVolatileDataCache()
{
    datacache_.landmark_total = -1;
    datacache_.landmark_coords.clear();
}

void SaveLandmarkPlanData(struct VolatileTempDataCache datacache, std::string f)
{
    std::ofstream filesave(f);
    if(filesave.is_open())
    {
        for(int i=0;i<datacache.landmark_total;i++)
        {
            std::vector<std::vector<double>> c = datacache.landmark_coords;
            filesave << "p" << i << ":\n";
            filesave << "\n";
            filesave << "  {\n";
            filesave << "    x: " << FormatDouble2String(c[i][0], 16) << ",\n";
            filesave << "    y: " << FormatDouble2String(c[i][1], 16) << ",\n";
            filesave << "    z: " << FormatDouble2String(c[i][2], 16) << "\n";
            filesave << "  }\n";
            filesave << "\n";
        }
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