#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/asio.hpp>
#include <signal.h>

#include "kst_servoing.hpp"
#include "rotms_robot_ros_interface.hpp"

void endSigintHandler(int sig)
{
    ros::Duration duration(0.05);
    for(int i=0;i<10;i++)
    {// This is not an elegent solution but works (at best effort)
        duration.sleep();
        ros::spinOnce();
    }
    ros::shutdown();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "NodeRobot", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, endSigintHandler);

    boost::asio::io_context io_context;
    KstServoing kst = KstServoing("172.31.1.147", io_context);
    RobotROSInterface ri = RobotROSInterface(kst, nh);

    ros::spin();
    return 0;
}