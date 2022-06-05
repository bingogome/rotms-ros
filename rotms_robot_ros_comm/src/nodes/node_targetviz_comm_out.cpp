#include <ros/ros.h>
#include <ros_side_out_node.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "CommOutTargetViz");
    ros::NodeHandle n;

	CommNodeOutiter(n, "TARGETVIZ");

	ros::spin();
	return 0;
}