#include "rotms_dispatcher.hpp"
#include "flag_machine.hpp"
#include "state_machine.hpp"
#include "state_machine_states.hpp"
#include <ros/ros.h>
#include <tuple>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "DummyNode");
    ros::NodeHandle nh;

    Dispatcher d = Dispatcher(nh);
    FlagMachine f = FlagMachine();
    std::vector<WorkState> vec = GetStatesVector(f);
    bool temp = CheckFlagIntegrity(vec);

    ROS_INFO_STREAM(temp);

    ros::spin();
    return 0;
}