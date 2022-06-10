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
    std::string packpath = ros::package::getPath("rotms_robot_ros_comm_decode");
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
    CommDecoderPubs& pubs,
    FuncMap opsdict
    ) 
    : 
    n_(n), pubs_(pubs),
    cmddict_(LoadCmdsConfig(modulesuffix)), 
    opsdict_(opsdict)
{
    // Load ros_comm (ros_side_in) configuration
    std::string packpath_comm = ros::package::getPath("rotms_robot_ros_comm");
    YAML::Node f_comm = YAML::LoadFile(packpath_comm + "/config_comm.yaml");
    sub_ = n.subscribe(f_comm["PUBLISHER_NAME_"+modulesuffix].as<std::string>(), 
        10, &CommDecoder::SubCallBack, this);    
}

void CommDecoder::SubCallBack(const std_msgs::String::ConstPtr& msg)
{
    ss_str_ = msg->data.c_str();
    CmdsProcess();
}

void CommDecoder::CmdsProcess()
{
    std::stringstream sscmd;
    std::stringstream ss;
    for(int i=0;i<16;i++) // msg header length is 16
        sscmd << ss_str_[i];
    for(int i=16;ss_str_[i]!='\n';i++)
        ss << ss_str_[i];
    opsdict_[cmddict_[sscmd]](pubs_, ss);
}

CommDecoderPubs::CommDecoderPubs()
{

}