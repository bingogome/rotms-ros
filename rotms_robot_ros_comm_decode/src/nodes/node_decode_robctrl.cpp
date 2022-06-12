#include <ros/ros.h>
#include "decode_node.hpp"
#include "function_map_robctrl.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "CommDecodeRobCtrl");
    ros::NodeHandle nh;
    FuncMap fm = GetFuncMapRobCtrl();
    CommDecoderRobCtrl dcdr = CommDecoderRobCtrl(nh, "ROBCTRL", fm);

    ros::spin();
    return 0;
}