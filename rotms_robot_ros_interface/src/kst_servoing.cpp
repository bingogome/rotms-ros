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

#include <kst_servoing.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <iomanip>
#include <ros/ros.h>

/*****
 *
 * Some utility functions
 * 
 ****/

// Format the double to string messages
std::string FormatedDouble2String(double a, int dec)
{
	std::stringstream stream;
    stream << std::fixed << std::setprecision(dec) << a;
    std::string s = stream.str();
    return s;
}

// Parse received messages into double vector
std::vector<double> ParseString2DoubleVec(std::string s)
{
	std::vector<double> vec;
	std::string delimiter = "_";
	size_t pos = 0;
	while ((pos = s.find(delimiter)) != std::string::npos) 
	{
	    vec.push_back(std::stod(s.substr(0, pos)));
	    s.erase(0, pos + delimiter.length());
	}
	return vec;
}

/*****
 * 
 * KstServoing implementations
 *
 ****/

// Constructor
KstServoing::KstServoing(
	std::string robot_ip, 
	boost::asio::io_context& io_context)
	: 
	tcp_sock_(io_context)
{
	ip_ = robot_ip;
}

// Send connection request to Sunrise Cabinet
void KstServoing::NetEstablishConnection()
{

    tcp::endpoint remote_endpoint = tcp::endpoint(boost::asio::ip::address_v4::from_string(ip_), 30001);
	tcp_sock_.connect(remote_endpoint);

	ros::Duration(1).sleep();

	boost::system::error_code error;

	// Msg is the tool transformation (may need to edit. leave it for now
	// currently just calculate from outside and send the command that only
	// regards to the end-effector of the robot)
	const std::string msg1 = "TFtrans_0.0_0.0_0.0_0.0_0.0_0.0\n";
	size_t lenmsgr1;

	// TODO: test if: successful connection when received 
	// May need to add a pause (or better, check if can do a handshake type of ack)
	// Previously using ros sleep

	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
	lenmsgr1 = tcp_sock_.read_some(boost::asio::buffer(buf_));
	
	ros::Duration(0.5).sleep();
	
}

// Send a point-to-point command, in joint space
void KstServoing::PTPJointSpace(std::vector<double> jpos , double relVel)
{
	try
	{
		boost::system::error_code error;

		const std::string msg1 = "jRelVel_" + 
			boost::lexical_cast<std::string>(relVel) + "_\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
		size_t lenmsgr1 = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		const std::string msgjs = "jp_" + 
			FormatedDouble2String(jpos[0], 5) + "_" +
			FormatedDouble2String(jpos[1], 5) + "_" +
			FormatedDouble2String(jpos[2], 5) + "_" +
			FormatedDouble2String(jpos[3], 5) + "_" +
			FormatedDouble2String(jpos[4], 5) + "_" +
			FormatedDouble2String(jpos[5], 5) + "_" +
			FormatedDouble2String(jpos[6], 5) + "_\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msgjs), error);
		size_t lenmsgrjs = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		const std::string msg2 = "doPTPinJS\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msg2), error);
		size_t lenmsgr2 = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		size_t lenmsgr3 = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

	}
	catch(std::exception& e)
	{
		throw;
	}
}

// Send a point-to-point command, in end-effector, and the linear trajectory
void KstServoing::PTPLineEFF(std::vector<double> epos, double vel)
{// vel: mm/sec
	try
	{
		boost::system::error_code error;

		const std::string msg1 = "jRelVel_" + 
			boost::lexical_cast<std::string>(vel) + "_\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
		size_t lenmsgr1 = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		const std::string msgjs = "cArtixanPosition_" +
			std::to_string(epos[0]) + "_" +
			std::to_string(epos[1]) + "_" +
			std::to_string(epos[2]) + "_" +
			std::to_string(epos[3]) + "_" +
			std::to_string(epos[4]) + "_" +
			std::to_string(epos[5]) + "_\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msgjs), error);
		size_t lenmsgrjs = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		const std::string msg2 = "doPTPinCS\n"; 
		boost::asio::write(tcp_sock_, boost::asio::buffer(msg2), error);
		size_t lenmsgr2 = tcp_sock_.read_some(boost::asio::buffer(buf_), error);

		size_t lenmsgr3 = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
	}
	catch(std::exception& e)
	{
		throw;
	}
}

// Send a command to Sunrise Cabinet to start Direct Servo, in EFF
void KstServoing::ServoDirectCartesianStart()
{
	boost::system::error_code error;
	const std::string msg1 = "stDcEEf_\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
	size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
}

// Send a command to Sunrise Cabinet to start Direct Servo, in joint space
void KstServoing::ServoDirectJointStart()
{
	boost::system::error_code error;
	const std::string msg1 = "startDirectServoJoints\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
	size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
}

// Send a command to Sunrise Cabinet to start Smart Servo, in EFF
void KstServoing::ServoSmartCartesianStart()
{
	boost::system::error_code error;
	const std::string msg1 = "stSmtSEEf_\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
	size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
}

// Stop Smart/Direct Servo
void KstServoing::ServoStop()
{
	boost::system::error_code error;
	const std::string msg1 = "stopDirectServoJoints\n"; // the same to stop direct servo based on KST
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
	size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
}

// In servo mode, send joint positions
void KstServoing::ServoSendJoints(std::vector<double> jp)
{
	boost::system::error_code error;
	const std::string msg1 = "jf_"+ 
		FormatedDouble2String(jp[0], 5) + "_" +
		FormatedDouble2String(jp[1], 5) + "_" +
		FormatedDouble2String(jp[2], 5) + "_" +
		FormatedDouble2String(jp[3], 5) + "_" +
		FormatedDouble2String(jp[4], 5) + "_" +
		FormatedDouble2String(jp[5], 5) + "_" +
		FormatedDouble2String(jp[6], 5) + "_\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
}


// In servo mode, send joint positions and get feedback
std::vector<double> KstServoing::ServoSendJointsGetFeedback(std::vector<double> jp)
{
	boost::system::error_code error;
	const std::string msg1 = "jpJP_"+ 
		FormatedDouble2String(jp[0], 5) + "_" +
		FormatedDouble2String(jp[1], 5) + "_" +
		FormatedDouble2String(jp[2], 5) + "_" +
		FormatedDouble2String(jp[3], 5) + "_" +
		FormatedDouble2String(jp[4], 5) + "_" +
		FormatedDouble2String(jp[5], 5) + "_" +
		FormatedDouble2String(jp[6], 5) + "_\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);

	size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
	std::stringstream ssmsgr;
	std::string strmsgr;
	for(int i=0;i<lenmsgr;i++)
		ssmsgr << buf_[i];
	strmsgr = ssmsgr.str();

	// Joint position feedback
	std::vector<double> jpbk = ParseString2DoubleVec(strmsgr);

	return jpbk;
}

// In servo mode, send EFF position
void KstServoing::ServoSendEFF(std::vector<double> eff) // x y z rz ry rx
{
	boost::system::error_code error;
	const std::string msg1 = "DcSeCar_"+ 
		FormatedDouble2String(eff[0], 5) + "_" +
		FormatedDouble2String(eff[1], 5) + "_" +
		FormatedDouble2String(eff[2], 5) + "_" +
		FormatedDouble2String(eff[3], 5) + "_" +
		FormatedDouble2String(eff[4], 5) + "_" +
		FormatedDouble2String(eff[5], 5) + "_\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
}

// In servo mode, send EFF position and get feedback
std::vector<double> KstServoing::ServoSendEFFGetFeedback(std::vector<double> eff)  // x y z rz ry rx
{
	boost::system::error_code error;
	const std::string msg1 = "DcSeCarEEfP_"+ 
		FormatedDouble2String(eff[0], 5) + "_" +
		FormatedDouble2String(eff[1], 5) + "_" +
		FormatedDouble2String(eff[2], 5) + "_" +
		FormatedDouble2String(eff[3], 5) + "_" +
		FormatedDouble2String(eff[4], 5) + "_" +
		FormatedDouble2String(eff[5], 5) + "_\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);

	size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
	std::stringstream ssmsgr;
	std::string strmsgr;
	for(int i=0;i<lenmsgr;i++)
		ssmsgr << buf_[i];
	strmsgr = ssmsgr.str();

	std::vector<double> effbk = ParseString2DoubleVec(strmsgr);

	return effbk;
}

// Get current joint position
std::vector<double> KstServoing::GetJointPosition()
{
	std::string strmsgr;
	try
	{
		boost::system::error_code error;
		const std::string msg = "getJointsPositions\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msg), error);
		size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
		std::stringstream ssmsgr;
		for(int i=0;i<lenmsgr;i++)
			ssmsgr << buf_[i];
		ssmsgr << "_"; // this is for getting jpos only, because the kuka java side miss a _
		strmsgr = ssmsgr.str();
	}
	catch(std::exception& e)
	{
		throw;
	}
	std::vector<double> vec = ParseString2DoubleVec(strmsgr);
	// vector format: a1 a2 a3 a4 a5 a6 a7
	
	return vec;
}

// Get current EFF pose
std::vector<double> KstServoing::GetEFFPosition()
{
	std::string strmsgr;
	try
	{
		boost::system::error_code error;
		const std::string msg = "Eef_pos\n";
		boost::asio::write(tcp_sock_, boost::asio::buffer(msg), error);
		size_t lenmsgr = tcp_sock_.read_some(boost::asio::buffer(buf_), error);
		std::stringstream ssmsgr;
		for(int i=0;i<lenmsgr;i++)
			ssmsgr << buf_[i];
		strmsgr = ssmsgr.str();
	}
	catch(std::exception& e)
	{
		throw;
	}
	std::vector<double> vec = ParseString2DoubleVec(strmsgr);
	// vector format: x y z rz ry rx

	return vec;
}

// Turn of the connection
void KstServoing::NetTurnoffServer()
{
	boost::system::error_code error;
	const std::string msg1 = "end\n";
	boost::asio::write(tcp_sock_, boost::asio::buffer(msg1), error);
	tcp_sock_.close();
}
