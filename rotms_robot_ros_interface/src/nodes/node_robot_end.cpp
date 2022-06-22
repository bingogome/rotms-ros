#include <ros/ros.h>
#include <std_msgs/String.h>
#include <signal.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

void endSigintHandler(int sig)
{

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/RobInterface/TerminateNode", 10);
    std_msgs::String msg;
   
    std::string packpath = ros::package::getPath("rotms_robot_ros_interface");
	YAML::Node f = YAML::LoadFile(packpath + "/config.yaml");
    YAML::Node ff = f["ROBOT_NODE_TERMINATE"];

    double handshake_rate = ff["END_HANDSHAKE_EFFORTS_RATE"].as<double>();
    int handshake_num = ff["END_HANDSHAKE_EFFORTS_NUM"].as<int>();

    // This wait is needed for successfully publish the ending msg
    ros::Duration duration(1.0/handshake_rate);
    duration.sleep();

    std::stringstream ss;
    ss << "_end_robot_connection_";
    msg.data = ss.str();
    
    // Enter a random "handshake" effort
    ROS_INFO_STREAM("Ending robot connetion "+msg.data); 
    for(int i=0;i<handshake_num;i++)
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