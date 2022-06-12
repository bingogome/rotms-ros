#include <ros/ros.h>
#include "decode_node.hpp"
#include "function_map_medimg.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CommDecodeMedImg");
    ros::NodeHandle nh;
    FuncMap fm = GetFuncMapMedImg();
    CommDecoderMedImg dcdr = CommDecoderMedImg(nh, "MEDIMG", fm);

    ros::spin();
    return 0;
}