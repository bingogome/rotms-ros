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
#include <std_msgs/String.h>
#include <boost/asio.hpp>
#include <signal.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

#include "kst_servoing.hpp"
#include "rotms_robot_ros_interface.hpp"

void endSigintHandler(int sig)
{
    std::string packpath = ros::package::getPath("rotms_robot_ros_interface");
	YAML::Node f = YAML::LoadFile(packpath + "/config.yaml");
    YAML::Node ff = f["ROBOT_NODE_TERMINATE"];

    double handshake_rate = ff["END_HANDSHAKE_EFFORTS_RATE"].as<double>();
    int handshake_num = ff["END_HANDSHAKE_EFFORTS_NUM"].as<int>();

    // Enter a random "handshake" effort
    // Hope for a successful handshake at the end of node
    ros::Duration duration(1.0/handshake_rate);
    for(int i=0;i<handshake_num;i++)
    {// This is not an elegent solution but works (at best effort)
        duration.sleep();
        ros::spinOnce();
    }
    ros::shutdown();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "NodeRobot", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, endSigintHandler);

    boost::asio::io_context io_context;
    
    std::string packpath = ros::package::getPath("rotms_robot_ros_interface");
	YAML::Node f = YAML::LoadFile(packpath + "/config.yaml");
    YAML::Node ff = f["ROBOT"];

    KstServoing kst = KstServoing(ff["IP_ADDRESS"].as<std::string>(), io_context);
    RobotROSInterface ri = RobotROSInterface(kst, nh);

    ros::spin();
    return 0;
}