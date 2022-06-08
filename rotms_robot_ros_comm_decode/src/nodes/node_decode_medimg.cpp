#include <ros/ros.h>
#include "decode_node.hpp"


class CommDecoderMedImg : public CommDecoder
{

public:

    CommDecoderMedImg(ros::NodeHandle& n) : CommDecoder(n,"MEDIMG") {}

protected:

    // Modify this method to process the commands from ros_comm (ros_side_in)
    void CmdsProcess() override 
    {
        
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CommDecodeMedImg");
    ros::NodeHandle nh;

    std::map<std::string, std::string> cmddict;
    CommDecoderMedImg dcdr = CommDecoderMedImg(nh);

    ros::spin();
    return 0;
}