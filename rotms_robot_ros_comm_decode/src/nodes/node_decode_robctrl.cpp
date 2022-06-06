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
    std::string test = ff["GET_JNT_ANGS"].as<std::string>();
    ROS_INFO_STREAM("Msg: "<<test);
    
    return 0;
}