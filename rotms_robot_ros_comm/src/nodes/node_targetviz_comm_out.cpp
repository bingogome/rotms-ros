#include <ros/ros.h>
#include <ros_side_out_node.hpp>
#include <ros_side_out.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "CommOutTargetViz");
    ros::NodeHandle n;

	ROSSideOut server = CommNodeOutIniter(n, "TARGETVIZ");

	ros::spin();
	return 0;
}