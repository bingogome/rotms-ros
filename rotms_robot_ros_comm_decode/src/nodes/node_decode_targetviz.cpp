#include <ros/ros.h>
#include "decode_node.hpp"


class CommDecoderTargetViz : public CommDecoder
{

public:

    CommDecoderTargetViz(ros::NodeHandle& n) : CommDecoder(n,"TargetViz") {}

protected:

    void CmdsProcess() override 
    {
        
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CommDecodeTargetViz");
    ros::NodeHandle nh;

    std::map<std::string, std::string> cmddict;
    CommDecoderTargetViz dcdr = CommDecoderTargetViz(nh);

    ros::spin();
    return 0;
}