#include <ros/ros.h>
#include "decode_node.hpp"
#include "function_map_robctrl.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "CommDecodeRobCtrl");
    ros::NodeHandle nh;

    std::map<std::string, std::string> cmddict;
    CommDecoderPubsRobCtrl dcdr_pubs = CommDecoderPubsRobCtrl();
    FuncMap funcs = GetFuncMapRobCtrl();
    CommDecoder dcdr = CommDecoder(nh, "ROBCTRL", dcdr_pubs, funcs);

    ros::spin();
    return 0;
}