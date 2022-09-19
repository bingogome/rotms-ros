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

#include <ros_side_out.hpp>

#include <std_msgs/String.h>
#include <ros/ros.h>

#include <string>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

#include "ros_print_color.hpp"

using boost::asio::ip::udp;
using boost::asio::ip::address;

ROSSideOut::ROSSideOut(ros::NodeHandle& n, boost::asio::io_context& io_context, 
	struct ROSSideOutConfig cfg)
	: cfg_(cfg), n_(n), socket_(io_context), remote_endpoint_(address::from_string(cfg.ip_out), cfg.port_out)
{
	socket_.open(udp::v4());
	// msg to send
	sub_ = n_.subscribe(cfg_.subscriber_name, 10, &ROSSideOut::SubCallBack, this);
}

void ROSSideOut::SubCallBack(const std_msgs::String::ConstPtr& msg)
{
	ss_str_ = msg->data.c_str();
	socket_.send_to(boost::asio::buffer(ss_str_),remote_endpoint_);
	if (cfg_.verbose == 1)
	{
		ROS_GREEN_STREAM("[ROTMS INFO] Following message was sent out to an udp port:");
		ROS_GREEN_STREAM("[ROTMS INFO] Message content: "<<ss_str_);
	}
}

ROSSideOutAndAck::ROSSideOutAndAck(ros::NodeHandle& n, boost::asio::io_context& io_context, 
	struct ROSSideOutAndAckConfig cfg)
	: ROSSideOut(n, io_context, cfg)
{
	// acknowledge when receive from ros_side_in
	sub_ack_ = n_.subscribe(cfg.publisher_name, 10, &ROSSideOutAndAck::AckCallBack, this);
}

void ROSSideOutAndAck::AckCallBack(const std_msgs::String::ConstPtr& msg)
{
	socket_.send_to(boost::asio::buffer("ack"),remote_endpoint_);
	if (cfg_.verbose == 1)
	{
		ROS_GREEN_STREAM("[ROTMS INFO] Ack the received message from ros_side_in.");
	}
}