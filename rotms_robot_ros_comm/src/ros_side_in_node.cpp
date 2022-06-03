#include <ros_side_in.hpp>
#include <ros_side_in_node.hpp>
#include <ros/package.h>

#include <yaml-cpp/yaml.h>
#include <boost/asio.hpp>

void CommNodeIniter(ros::NodeHandle& n, std::string modulesuffix)
{
    boost::asio::io_context io_context;

	std::string packpath = ros::package::getPath("rotms_robot_ros_comm");
	YAML::Node f = YAML::LoadFile(packpath + "/config_comm.yaml");

	struct ROSSideInConfig cfg;
	cfg.port_in = f["PORT_IN_"+modulesuffix].as<int>();
	cfg.msg_size = f["MSG_SIZE_"+modulesuffix].as<int>();
	cfg.end_msg = f["MSG_END_"+modulesuffix].as<std::string>();
	cfg.publisher_name = f["PUBLISHER_NAME_"+modulesuffix].as<std::string>();
	cfg.verbose = f["VERBOSE_"+modulesuffix].as<int>();

	try
	{
		ROSSideIn server(n, io_context, cfg);
		io_context.run(); 
		// For my own note: 
		// use io_context.poll() if do not want it run indefinitely
	}
	catch (std::exception& e)
	{
		io_context.stop();
		ROS_INFO("Error happened, or user interrupted");
        throw;
	}
}