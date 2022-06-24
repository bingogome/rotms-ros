/***
MIT License

Copyright (c) 2022 Yihao Liu, Johns Hopkins University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
***/

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