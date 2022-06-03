#include <ros_side_in.hpp>

#include <std_msgs/String.h>
#include <ros/ros.h>

#include <string>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

ROSSideIn::ROSSideIn(ros::NodeHandle& n, boost::asio::io_context& io_context, 
	struct ROSSideInConfig cfg) :
	cfg_(cfg),
	n_(n), 
	socket_(io_context, udp::endpoint(udp::v4(), cfg.port_in))
{
	pub_test_ = n_.advertise<std_msgs::String>(cfg_.publisher_name, 50);
	ROSSideIn::StartReceive();
}

void ROSSideIn::StartReceive()
{
	if (cfg_.verbose == 1)
		ROS_INFO("Waiting Transmission");
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
		ROS_INFO("Handling msg ...");
	std::stringstream sscmd;
	std::stringstream ss;
	for(int i=0;i<10;i++) // header length is 10
		sscmd << recv_buffer_[i];
	for(int i=0;i<cfg_.msg_size;i++) // chars in a msg
		ss << recv_buffer_[i];
	sscmd_str_ = sscmd.str();
	ss_str_ = ss.str();

	if(sscmd_str_==cfg_.end_msg){
		if (cfg_.verbose == 1)
			ROS_INFO("Ending communication node ...");
		ROSSideIn::EndServerClean();
		throw;
	}
	else{
		// main msg handling body 
		msg_test_.data = ss_str_;
		pub_test_.publish(msg_test_);
	}
}