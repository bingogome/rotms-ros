#include <ros/ros.h>
#include "decode_node.hpp"

class CommDecoderPubsMedImg : public CommDecoderPubs
{
public:
    CommDecoderPubsMedImg() : CommDecoderPubs()
    {

    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CommDecodeMedImg");
    ros::NodeHandle nh;

    std::map<std::string, std::string> cmddict;
    CommDecoderPubsMedImg dcdr_pubs = CommDecoderPubsMedImg();

    ros::spin();
    return 0;
}