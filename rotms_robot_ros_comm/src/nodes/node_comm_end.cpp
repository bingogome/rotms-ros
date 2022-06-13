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
#include <signal.h>
#include <ros/package.h>

#include <boost/array.hpp>
#include <boost/asio.hpp>

#include <yaml-cpp/yaml.h>

using boost::asio::ip::udp;

void endPort(int portnum)
{
    boost::asio::io_context io_context;
    udp::socket socket_(io_context);;
	udp::endpoint remote_endpoint_(udp::v4(), portnum);
    std::string ss_str_;

    socket_.open(udp::v4());
    ss_str_ = "_msg_end__";
	socket_.send_to(boost::asio::buffer(ss_str_),remote_endpoint_);
    ros::spinOnce();

}

void endSigintHandler(int sig)
{

    std::string packpath = ros::package::getPath("rotms_robot_ros_comm");
	YAML::Node f = YAML::LoadFile(packpath + "/config_comm.yaml");

    std::vector<int> portvec;
	portvec.push_back(f["PORT_IN_MEDIMG"].as<int>());
    portvec.push_back(f["PORT_IN_ROBCTRL"].as<int>());
    portvec.push_back(f["PORT_IN_TARGETVIZ"].as<int>());

    for(int i=0;i<portvec.size();i++)
    {
        endPort(portvec[i]);
    }

    ros::shutdown();
}

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "CommEnd", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    signal(SIGINT, endSigintHandler);
    ros::spin();
    return 0;
}