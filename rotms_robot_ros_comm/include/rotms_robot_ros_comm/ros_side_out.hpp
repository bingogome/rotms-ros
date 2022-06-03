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
