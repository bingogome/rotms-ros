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
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <vector>
using boost::asio::ip::tcp;

class KstServoing{

private:

    // connection
    std::string ip_;
    tcp::socket tcp_sock_; // tcpip connection object
    boost::array<char, 256> buf_;

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
    void PTPLineEFF(std::vector<double> epos, double vel); // vel: mm/sec
    // epos: x y z rz ry rx

    // Smart and direct servo methods
    void ServoDirectCartesianStart();
    void ServoDirectJointStart();
    void ServoSmartCartesianStart();
    void ServoSmartJointStart();
    void ServoStop();

    void ServoSendJoints(std::vector<double> jp);
    std::vector<double> ServoSendJointsGetFeedback(std::vector<double> jp);
    void ServoSendEFF(std::vector<double> eff); // x y z rz ry rx
    std::vector<double> ServoSendEFFGetFeedback(std::vector<double> eff); // x y z rz ry rx

    // getters
    std::vector<double> GetJointPosition();
    std::vector<double> GetEFFPosition();
        
    // networking
    void NetEstablishConnection();
    void NetTurnoffServer();

};