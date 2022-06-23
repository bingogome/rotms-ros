#include <ros/ros.h>
#include "flag_machine.hpp"
#include "state_machine.hpp"
#include "state_machine_states.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DummyNode");
    ros::NodeHandle nh;
    
    FlagMachine f = FlagMachine();
    std::vector<WorkState> vec = GetStatesVector(f);

    ros::spin();
    return 0;
}