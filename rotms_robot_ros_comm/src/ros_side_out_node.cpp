#include <ros_side_out.hpp>
#include <ros_side_out_node.hpp>
#include <ros/package.h>

#include <yaml-cpp/yaml.h>
#include <boost/asio.hpp>

ROSSideOut CommNodeOutIniter(ros::NodeHandle& n, std::string modulesuffix)
{
    boost::asio::io_context io_context;
    std::string packpath = ros::package::getPath("rotms_robot_ros_comm");
	YAML::Node f = YAML::LoadFile(packpath + "/config_comm.yaml");

    struct ROSSideOutConfig cfg;
	cfg.port_out = f["PORT_OUT_"+modulesuffix].as<int>();
	cfg.msg_size = f["MSG_SIZE_"+modulesuffix].as<int>();
	cfg.subscriber_name = f["SUBSCRIBER_NAME_"+modulesuffix].as<std::string>();
	cfg.verbose = f["VERBOSE_"+modulesuffix].as<int>();

	ROSSideOut server(n, io_context, cfg);

	return server;
}