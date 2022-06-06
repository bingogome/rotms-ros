#include <yaml-cpp/yaml.h>

int main(int argc, char **argv)
{
    std::string packpath = ros::package::getPath("rotms_robot_ros_comm");
	YAML::Node f = YAML::LoadFile(packpath + "/config_comm_decode.yaml");
    YAML::Node ff = f["ROBCTRL_CMDS"];
    std::string test = ff["GET_JNT_ANGS"]
    ROS_INFO_STREAM("Msg: "<<test);
    return 0;
}