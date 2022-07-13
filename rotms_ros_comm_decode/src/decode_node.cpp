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

#include "decode_node.hpp"

#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <map>

/**
* CommDecoder: The decoder for the msg received from ros_comm (ros_side_in)
*/

std::map<std::string, std::string> LoadCmdsConfig(const std::string modulesuffix)
{
    // Load config file and commands dictionary
    std::string packpath = ros::package::getPath("rotms_ros_comm_decode");
    YAML::Node f = YAML::LoadFile(packpath + "/config_comm_decode.yaml");
    YAML::Node ff = f[modulesuffix+"_CMDS"];
    std::map<std::string, std::string> cmddict;
    
    for(YAML::const_iterator it=ff.begin(); it!=ff.end(); ++it)
    {
        std::string key = it->first.as<std::string>();
        std::string value = it->second.as<std::string>();
        // switch key and value for the CommDecode to easily do function dispatch
        std::pair<std::string, std::string> p = std::make_pair(value, key);
        cmddict.insert(p);
    }
    return cmddict;
}

CommDecoder::CommDecoder(
    ros::NodeHandle& n, 
    const std::string modulesuffix,
    FuncMap opsdict
    ) 
    : 
    n_(n), 
    cmddict_(LoadCmdsConfig(modulesuffix)),
    opsdict_(opsdict)
{
    // Load ros_comm (ros_side_in) configuration
    std::string packpath_comm = ros::package::getPath("rotms_ros_comm");
    YAML::Node f_comm = YAML::LoadFile(packpath_comm + "/config_comm.yaml");
    // End of message character
    eom_ = f_comm["CMDS_EOM"].as<char>();
    // msg total length
    msglen_ = f_comm["MSG_SIZE_"+modulesuffix].as<int>();
    // Message received subscriber
    sub_ = n.subscribe(f_comm["PUBLISHER_NAME_"+modulesuffix].as<std::string>(), 
        10, &CommDecoder::SubCallBack, this);    
}

void CommDecoder::SubCallBack(const std_msgs::String::ConstPtr& msg)
{
    ss_str_ = msg->data.c_str();
    
    std::stringstream sscmd;
    std::stringstream ss;
    // msg header length is msgheaderlen_
    for(int i=0;i<msgheaderlen_;i++) sscmd << ss_str_[i];
    for(int i=msgheaderlen_; ss_str_[i]!=eom_ && i<msglen_; i++)
    {
        ss << ss_str_[i];
    }
    std::string lookupkey = cmddict_.at(sscmd.str());
    std::string msgcontent = ss.str();

    // Check if the content is only the eom symbol.
    std::string eom_string(1, eom_);
    if(ss.str()!=eom_string)
        msgcontent = msgcontent.erase(0,1);


    CmdsProcess(lookupkey, msgcontent);
}

void CommDecoder::CmdsProcess(std::string& lookupkey, std::string& msgcontent)
{
    OperationFunc operation = opsdict_.at(lookupkey);
    (*operation)(msgcontent, pubs_);
}

