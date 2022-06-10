#include <ros/ros.h>
#include "decode_node.hpp"

class CommDecoderPubsTargetViz : public CommDecoderPubs
{
public:
    CommDecoderPubsTargetViz() : CommDecoderPubs()
    {

    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CommDecodeTargetViz");
    ros::NodeHandle nh;

    std::map<std::string, std::string> cmddict;
    CommDecoderPubsTargetViz dcdr_pubs = CommDecoderPubsTargetViz();

    ros::spin();
    return 0;
}