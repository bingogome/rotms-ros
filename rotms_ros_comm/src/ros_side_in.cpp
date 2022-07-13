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

#include <std_msgs/String.h>
#include <ros/ros.h>

#include <string>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

#include "ros_print_color.hpp"

using boost::asio::ip::udp;

ROSSideIn::ROSSideIn(ros::NodeHandle& n, boost::asio::io_context& io_context, 
	struct ROSSideInConfig cfg) :
	cfg_(cfg),
	n_(n), 
	socket_(io_context, udp::endpoint(udp::v4(), cfg.port_in))
{
	eom_ = cfg.eom;
	pub_test_ = n_.advertise<std_msgs::String>(cfg_.publisher_name, 50);
	ROSSideIn::StartReceive();
}

void ROSSideIn::StartReceive()
{
	if (cfg_.verbose == 1)
		ROS_GREEN_STREAM("[ROTMS INFO] Waiting for data transmission (in) ...");
	socket_.async_receive_from(
		boost::asio::buffer(recv_buffer_), 
		remote_endpoint_,
		boost::bind(
			&ROSSideIn::HandleReceive, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred
		)
	);
}

// End port gracefully
void ROSSideIn::EndServerClean()
{
	socket_.close();
}

// Handle asio communication. Start new receive once finished
void ROSSideIn::HandleReceive(const boost::system::error_code& error,
		std::size_t /*bytes_transferred*/)
{
	if (!error)
	{
		ROSSideIn::HandleIncoming();
		ros::spinOnce();
		ROSSideIn::StartReceive();
	}
}

// Handle msg processing.
void ROSSideIn::HandleIncoming()
{
	if (cfg_.verbose == 1)
		ROS_GREEN_STREAM("[ROTMS INFO] Handling received message ...");
	std::stringstream sscmd;
	std::stringstream ss;
	for(int i=0;i<10;i++) // end msg header length is 10
		sscmd << recv_buffer_[i];
	for(int i=0;i<cfg_.msg_size;i++) // chars in a msg
	{
		ss << recv_buffer_[i];
		if(recv_buffer_[i]==eom_)
			break;
	}
	sscmd_str_ = sscmd.str();
	ss_str_ = ss.str();

	// if (cfg_.verbose)
	// 	ROS_INFO_STREAM(ss_str_+"\n");

	if(sscmd_str_==cfg_.end_msg){
		if (cfg_.verbose == 1)
			ROS_GREEN_STREAM("[ROTMS INFO] Ending communication node ...");
		ROSSideIn::EndServerClean();
		throw;
	}
	else{
		// main msg handling body 
		msg_test_.data = ss_str_;
		pub_test_.publish(msg_test_);
	}
}