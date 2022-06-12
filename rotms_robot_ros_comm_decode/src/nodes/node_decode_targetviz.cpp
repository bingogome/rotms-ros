#include <ros/ros.h>
#include "decode_node.hpp"
#include "function_map_targetviz.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CommDecodeTargetViz");
    ros::NodeHandle nh;
    FuncMap fm = GetFuncMapTargetViz();
    CommDecoderTargetViz dcdr = CommDecoderTargetViz(nh, "TARGETVIZ", fm);

    ros::spin();
    return 0;
}