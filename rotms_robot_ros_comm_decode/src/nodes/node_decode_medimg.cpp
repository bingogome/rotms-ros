#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CommDecodeMedImg");
    ros::NodeHandle nh;

    ros::spin();
    return 0;
}