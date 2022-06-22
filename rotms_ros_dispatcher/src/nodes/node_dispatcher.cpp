#include "rotms_dispatcher.hpp"
#include <ros/ros.h>
#include <tuple>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "DummyNode");
    ros::NodeHandle nh;

    Dispatcher d = Dispatcher(nh);

    ros::spin();
    return 0;
}