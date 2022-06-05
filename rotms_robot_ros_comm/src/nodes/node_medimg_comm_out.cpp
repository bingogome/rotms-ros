#include <ros/ros.h>
#include <ros_side_out_node.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "CommOutMedImg");
    ros::NodeHandle n;

	CommNodeOutiter(n, "MEDIMG");

	ros::spin();
	return 0;
}