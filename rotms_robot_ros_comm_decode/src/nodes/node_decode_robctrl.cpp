#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CommDecodeRobCtrl");
    ros::NodeHandle nh;

    std::string packpath = ros::package::getPath("rotms_robot_ros_comm_decode");
	YAML::Node f = YAML::LoadFile(packpath + "/config_comm_decode.yaml");
    YAML::Node ff = f["ROBCTRL_CMDS"];

    std::map<std::string, std::string> cmddict;

    for(YAML::const_iterator it=ff.begin(); it!=ff.end(); ++it)
    {
        std::string key = it->first.as<std::string>();
        std::string value = it->second.as<std::string>();
        std::pair<std::string, std::string> p = std::make_pair(key,value);
        cmddict.insert(p);
    }

    ROS_INFO_STREAM("Msg: "<<cmddict.size());
    
    return 0;
}