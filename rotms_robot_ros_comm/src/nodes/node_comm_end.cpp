#include <ros/ros.h>
#include <signal.h>
#include <ros/package.h>

#include <boost/array.hpp>
#include <boost/asio.hpp>

#include <yaml-cpp/yaml.h>

using boost::asio::ip::udp;

void endPort(int portnum)
{
    boost::asio::io_context io_context;
    udp::socket socket_(io_context);;
	udp::endpoint remote_endpoint_(udp::v4(), portnum);
    std::string ss_str_;

    socket_.open(udp::v4());
    ss_str_ = "_msg_end__";
	socket_.send_to(boost::asio::buffer(ss_str_),remote_endpoint_);

}

void endSigintHandler(int sig)
{

    std::string packpath = ros::package::getPath("rotms_robot_ros_comm");
	YAML::Node f = YAML::LoadFile(packpath + "/config_comm.yaml");

    std::vector<int> portvec;
	portvec.push_back(f["PORT_IN_MEDIMG"].as<int>());
    portvec.push_back(f["PORT_IN_ROBCTRL"].as<int>());
    portvec.push_back(f["PORT_IN_TARGETVIZ"].as<int>());

    for(int i=0;i<portvec.size();i++)
    {
        endPort(portvec[i]);
    }

    ros::shutdown();
}

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "CommEnd", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    signal(SIGINT, endSigintHandler);
    ros::spin();
    return 0;
}