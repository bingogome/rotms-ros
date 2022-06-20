#include <ros/ros.h>
#include "state_machine.hpp"
#include "state_machine_states.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DummyNode");
    ros::NodeHandle nh;
    std::vector<WorkState> vec = GetStatesVector();

    ros::spin();
    return 0;
}