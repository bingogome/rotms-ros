#include <ros/ros.h>
#include <ros_side_in_node.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "CommInTargetViz");
    ros::NodeHandle n;

    CommNodeInIniter(n, "TARGETVIZ");

	return 0;
}