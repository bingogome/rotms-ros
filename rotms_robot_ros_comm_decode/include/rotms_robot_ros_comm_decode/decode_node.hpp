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
#include <map>
#include <ros/ros.h>
#include <std_msgs/String.h>

typedef std::vector<ros::Publisher> PublisherVec;
typedef void (*OperationFunc)(std::string&, PublisherVec&);
typedef std::map<std::string, OperationFunc> FuncMap;


class CommDecoder
{

public:

    CommDecoder(
        ros::NodeHandle& n, 
        const std::string modulesuffix,
        FuncMap opsdict
        );

protected:

    void SubCallBack(const std_msgs::String::ConstPtr& msg);
    virtual void CmdsProcess(std::string& lookupkey, std::string& msgcontent);
    ros::NodeHandle& n_;
	ros::Subscriber sub_;
    PublisherVec pubs_;

    char eom_; // End of message indicator
    int msglen_; // general msg length
    int msgheaderlen_ = 16; // cmd hear length

    const std::map<std::string, std::string> cmddict_; // lookup table for cmd->CMD
    const FuncMap opsdict_; // lookup table for CMD->operation
    
    std::string ss_str_; // whole msg
};

