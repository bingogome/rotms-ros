#include <ros/ros.h>
#include "flag_machine.hpp"
#include "state_machine.hpp"
#include "state_machine_states.hpp"

// This node not needed in the final system. 
// The headers and definition files from this package will be 
// called by rotms_ros_dispatcher package
int main(int argc, char **argv)
{
    ros::init(argc, argv, "DummyNode");
    ros::NodeHandle nh;
    
    FlagMachine f = FlagMachine();
    std::vector<WorkState> vec = GetStatesVector(f);
    bool temp = CheckFlagIntegrity(vec);

    ROS_INFO_STREAM(temp);

    ros::spin();
    return 0;
}