#include "registration_funcs.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "DummyNode");

    ros::NodeHandle nh;
    ros::spin();
    return 0;
}