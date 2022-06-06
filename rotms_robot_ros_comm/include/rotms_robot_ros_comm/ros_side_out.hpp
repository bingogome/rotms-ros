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

#pragma once

#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

// Configuration parameters struct. Will be initialized by 
// data from config_ros.yaml
struct ROSSideOutConfig
{
	int port_out;
	std::string subscriber_name;
	int verbose;
	int msg_size;
};

class ROSSideOut
{

public:

	ROSSideOut(ros::NodeHandle& n, boost::asio::io_context& io_context, 
		struct ROSSideOutConfig cfg);

private:

	void SubCallBack(const std_msgs::String::ConstPtr& msg);

	// ros related members
	ros::NodeHandle& n_;
	ros::Subscriber sub_;
	std_msgs::String msg_test_;

	// asio related members
	udp::socket socket_;
	udp::endpoint remote_endpoint_;

	struct ROSSideOutConfig cfg_;
	std::string ss_str_; // whole msg
};
