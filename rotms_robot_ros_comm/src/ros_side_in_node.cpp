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

#include <ros_side_in.hpp>
#include <ros_side_in_node.hpp>
#include <ros/package.h>

#include <yaml-cpp/yaml.h>
#include <boost/asio.hpp>

void CommNodeInIniter(ros::NodeHandle& n, std::string modulesuffix)
{
    boost::asio::io_context io_context;

	std::string packpath = ros::package::getPath("rotms_robot_ros_comm");
	YAML::Node f = YAML::LoadFile(packpath + "/config_comm.yaml");

	struct ROSSideInConfig cfg;
	cfg.port_in = f["PORT_IN_"+modulesuffix].as<int>();
	cfg.msg_size = f["MSG_SIZE_"+modulesuffix].as<int>();
	cfg.end_msg = f["MSG_END_"+modulesuffix].as<std::string>();
	cfg.publisher_name = f["PUBLISHER_NAME_"+modulesuffix].as<std::string>();
	cfg.verbose = f["VERBOSE_"+modulesuffix].as<int>();

	try
	{
		ROSSideIn server(n, io_context, cfg);
		io_context.run(); 
		// For my own note: 
		// use io_context.poll() if do not want it run indefinitely
	}
	catch (std::exception& e)
	{
		io_context.stop();
		ROS_INFO("Error happened, or user interrupted");
        throw;
	}
}