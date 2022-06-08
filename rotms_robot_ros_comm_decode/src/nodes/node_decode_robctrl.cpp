#include <ros/ros.h>
#include "decode_node.hpp"


class CommDecoderRobCtrl : public CommDecoder
{

public:

    CommDecoderRobCtrl(ros::NodeHandle& n) : CommDecoder(n,"ROBCTRL") {}

protected:

    // Modify this method to process the commands from ros_comm (ros_side_in)
    // according to the config file config_comm_decode.yaml
    void CmdsProcess() override 
    {
        
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CommDecodeRobCtrl");
    ros::NodeHandle nh;

    std::map<std::string, std::string> cmddict;
    CommDecoderRobCtrl dcdr = CommDecoderRobCtrl(nh);

    ros::spin();
    return 0;
}