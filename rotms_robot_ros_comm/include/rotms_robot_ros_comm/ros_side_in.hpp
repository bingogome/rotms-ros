#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

// Configuration parameters struct. Will be initialized by 
// data from config_ros.yaml
struct ROSSideInConfig
{
	int port_in;
	std::string end_msg;
	std::string publisher_name;
	int verbose;
	int msg_size;
};

// Main class of the program. Will be called by the main function and constantly
// receives messages
class ROSSideIn
{

public: 
	// constructor and start receiving
	ROSSideIn(ros::NodeHandle& n, boost::asio::io_context& io_context, struct ROSSideInConfig cfg);

private:

	void StartReceive();
	void EndServerClean();
	void HandleReceive(const boost::system::error_code& error, std::size_t /*bytes_transferred*/);
	void HandleIncoming();

	// ros related members
	ros::NodeHandle& n_;
	ros::Publisher pub_test_; // can be action, service. Use topic as a demo
	std_msgs::String msg_test_;

	// asio related members
	udp::socket socket_;
	udp::endpoint remote_endpoint_;
	boost::array<char, 150> recv_buffer_; // TODO: make this configurable 55 chars in an encoded command
	// TODO: make socket array so that multiple channels are possible (or dynamic expansion)
	// TODO: make TCP optional

	struct ROSSideInConfig cfg_;
	std::string sscmd_str_; // command header of current messege
	std::string ss_str_; // whole msg


};