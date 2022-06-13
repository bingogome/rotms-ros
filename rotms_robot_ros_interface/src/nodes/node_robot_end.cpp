#include <ros/ros.h>
#include <std_msgs/String.h>
#include <signal.h>

void endSigintHandler(int sig)
{

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/RobInterface/EndConnections", 10);
    std_msgs::String msg;
   
    // This wait is needed for successfully publish the ending msg
    ros::Duration duration(0.05);
    duration.sleep();

    std::stringstream ss;
    ss << "_end_robot_connection_";
    msg.data = ss.str();
    
    ROS_INFO_STREAM("Ending robot connetion "+msg.data); 
    for(int i=0;i<10;i++)
    {
        pub.publish(msg);
        ros::spinOnce();
        duration.sleep();
    }
    ROS_INFO("Ending connection efforts sent to ROS node");
    ros::shutdown();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "NodeRobotEnd", ros::init_options::NoSigintHandler);

    ros::NodeHandle nh;
    signal(SIGINT, endSigintHandler);

    ros::spin();
    return 0;
}