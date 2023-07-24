/***
MIT License

Copyright (c) 2023 Yihao Liu, Johns Hopkins University

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
#include <iostream>
#include <fstream>
#include <ctime>
#include <time.h>

#include "ros_print_color.hpp"

std::string GetTimeString()
{
	time_t rawtime;
	struct tm * timeinfo;
	char buffer [80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime (buffer,80,"%Y%m%d_%I%M%S%p",timeinfo);

	return buffer;
}

class Mngr
{
// Manages the flag of running the calculation of the transform
public:
    
    Mngr(ros::NodeHandle& n) : n_(n){}
    bool run_flag = false;

    geometry_msgs::Pose msg_out_pose;
    geometry_msgs::Pose msg_out_pose_body;
    geometry_msgs::Pose msg_out_pose_tool;

    std::vector<geometry_msgs::Pose> v_pose;
    std::vector<geometry_msgs::Pose> v_pose_body;
    std::vector<geometry_msgs::Pose> v_pose_tool;

    void SaveAcquiredData()
    {
        std::ofstream f;
        std::string packpath = ros::package::getPath("rotms_ros");
        std::string filename = packpath + "/saveddata/poses_" + GetTimeString();

        f.open(filename + ".csv");
        for (int i=0; i<v_pose.size(); i++)
        {
            f << "body_tool"  << "," << 
                std::to_string(v_pose[i].position.x) << "," << 
                std::to_string(v_pose[i].position.y) << "," << 
                std::to_string(v_pose[i].position.z) << "," << 
                std::to_string(v_pose[i].orientation.x) << "," << 
                std::to_string(v_pose[i].orientation.y) << "," << 
                std::to_string(v_pose[i].orientation.z) << "," << 
                std::to_string(v_pose[i].orientation.w) << "\n";
            f << "pol_body"  << "," << 
                std::to_string(v_pose_body[i].position.x) << "," << 
                std::to_string(v_pose_body[i].position.y) << "," << 
                std::to_string(v_pose_body[i].position.z) << "," << 
                std::to_string(v_pose_body[i].orientation.x) << "," << 
                std::to_string(v_pose_body[i].orientation.y) << "," << 
                std::to_string(v_pose_body[i].orientation.z) << "," << 
                std::to_string(v_pose_body[i].orientation.w) << "\n";
            f << "pol_tool"  << "," << 
                std::to_string(v_pose_tool[i].position.x) << "," << 
                std::to_string(v_pose_tool[i].position.y) << "," << 
                std::to_string(v_pose_tool[i].position.z) << "," << 
                std::to_string(v_pose_tool[i].orientation.x) << "," << 
                std::to_string(v_pose_tool[i].orientation.y) << "," << 
                std::to_string(v_pose_tool[i].orientation.z) << "," << 
                std::to_string(v_pose_tool[i].orientation.w) << "\n";
        }
        f.close();
    }

private:

    ros::NodeHandle& n_;
    ros::Subscriber sub_run_ = n_.subscribe(
        "/Misc/Mep/DataRecord", 2, &Mngr::FlagCallBack, this);
    ros::Subscriber sub_tr_body_tool_ = n_.subscribe(
        "/Misc/body_tool", 2, &Mngr::BodyToolCallBack, this);
    ros::Subscriber sub_tr_pol_body_ = n_.subscribe(
        "/Misc/pol_body", 2, &Mngr::PolBodyCallBack, this);
    ros::Subscriber sub_tr_pol_tool_ = n_.subscribe(
        "/Misc/pol_tool", 2, &Mngr::PolToolCallBack, this);

    void FlagCallBack(const std_msgs::String::ConstPtr& msg)
    {
        if(msg->data.compare("_start__")==0) run_flag = true;
        if(msg->data.compare("_end__")==0) run_flag = false;
    }

    void BodyToolCallBack(const geometry_msgs::Pose::ConstPtr& msg)
    {
        msg_out_pose.position = msg->position;
        msg_out_pose.orientation = msg->orientation;
    }

    void PolBodyCallBack(const geometry_msgs::Pose::ConstPtr& msg)
    {
        msg_out_pose_body.position = msg->position;
        msg_out_pose_body.orientation = msg->orientation;
    }

    void PolToolCallBack(const geometry_msgs::Pose::ConstPtr& msg)
    {
        msg_out_pose_tool.position = msg->position;
        msg_out_pose_tool.orientation = msg->orientation;
    }
};

int main(int argc, char **argv)
{
    // ROS stuff
    ros::init(argc, argv, "NodeMepRequestSavePose");
    ros::NodeHandle nh;
    double rateFreq = 20.0;
    ros::Rate rate(rateFreq);

    // Instantiate the flag manager
    Mngr mngr1(nh);

    // Go in the loop
    int counter = 0;
    while (nh.ok())
    {
        if (mngr1.run_flag && counter<=(2*rateFreq))
        {
            mngr1.v_pose.push_back(mngr1.msg_out_pose);
            mngr1.v_pose_body.push_back(mngr1.msg_out_pose_body);
            mngr1.v_pose_tool.push_back(mngr1.msg_out_pose_tool);
            counter+=1;
        }
        else if (mngr1.run_flag && counter>(2*rateFreq))
        {
            // save
            mngr1.SaveAcquiredData();

            // echo
            ROS_GREEN_STREAM("[ROTMS INFO] Data saved.");

            // reset
            mngr1.run_flag = false;
            counter = 0;
            mngr1.v_pose.clear();
            mngr1.v_pose_body.clear();
            mngr1.v_pose_tool.clear();
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}