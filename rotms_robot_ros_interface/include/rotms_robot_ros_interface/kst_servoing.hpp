#pragma once
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <vector>
using boost::asio::ip::tcp;

class KstServoing{

private:

    // connection
    std::string ip_;
    tcp::socket tcp_sock_; // tcpip connection object
    boost::array<char, 128> buf_;

    // robot data
    std::string robot_type_;
    int flange_type_;

    // flags
    bool f_sft_real_time_ = false; // status of soft real time

public:

    // constructor
    KstServoing(
        std::string robot_ip, 
        boost::asio::io_context& io_context
        );
    
    // PTP motion
    void PTPJointSpace(std::vector<double> jpos , double relVel);
    void PTPLineEEF(std::vector<double> epos, double vel); // vel: mm/sec

    // Smart and direct servo methods
    void ServoDirectCartesianStart();
    void ServoDirectJointStart();
    void ServoSmartCartesianStart();
    void ServoSmartJointStart();
    void ServoStop();

    void ServoSendJoints(std::vector<double> jp);
    std::vector<double> ServoSendJointsGetFeedback(std::vector<double> jp);
    void ServoSendEEF(std::vector<double> eef); // x y z rz ry rx
    std::vector<double> ServoSendEEFGetFeedback(std::vector<double> eef); // x y z rz ry rx

    // getters
    std::vector<double> GetJointPosition();
    std::vector<double> GetEEFPosition();
        
    // networking
    void NetEstablishConnection();
    void NetTurnoffServer();

};