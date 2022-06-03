#include <ros/ros.h>
#include <ros_side_in_node.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "CommInMedImg");
    ros::NodeHandle n;

    CommNodeIniter(n, "MEDIMG");

	return 0;
}