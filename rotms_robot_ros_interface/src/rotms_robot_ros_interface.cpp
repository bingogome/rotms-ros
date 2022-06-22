#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <math.h>
#include "rotms_robot_ros_interface.hpp"

std::vector<double> quat2eul(std::vector<double> q /*x,y,z,w*/)
{
    double aSinInput = -2.0 * (q[0] * q[2] - q[3] * q[1]);
    if(aSinInput > 1.0)
        aSinInput = 1.0;
    if(aSinInput < -1.0)
        aSinInput = -1.0;
    
    std::vector<double> ans{
        atan2( 2.0 * (q[0] * q[1] + q[3] * q[2]), q[3] * q[3] + q[0] * q[0] - q[1] * q[1] - q[2] * q[2] ), 
        asin( aSinInput ), 
        atan2( 2.0 * (q[1] * q[2] + q[3] * q[0]), q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2] )
    };
    return ans;
}

std::vector<double> eul2quat(std::vector<double> eul)
{
	std::vector<double> eulhalf{eul[0]/2,eul[1]/2,eul[2]/2};
    eul = eulhalf;
    std::vector<double> ans{
        cos(eul[0]) * cos(eul[1]) * sin(eul[2]) - sin(eul[0]) * sin(eul[1]) * cos(eul[2]),
        cos(eul[0]) * sin(eul[1]) * cos(eul[2]) + sin(eul[0]) * cos(eul[1]) * sin(eul[2]),
        sin(eul[0]) * cos(eul[1]) * cos(eul[2]) - cos(eul[0]) * sin(eul[1]) * sin(eul[2]),
        cos(eul[0]) * cos(eul[1]) * cos(eul[2]) + sin(eul[0]) * sin(eul[1]) * sin(eul[2])
    };
    return ans; // x y z w
}

RobotROSInterface::RobotROSInterface(KstServoing& kst, ros::NodeHandle& nh) 
        : 
        kst_(kst), n_(nh) {}

// Start the cabinet connection
void RobotROSInterface::RobotInitConnectionCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data.compare("_start_robot_connection_")==0 && !flag_connected_)
    {
        try
        {
            kst_.NetEstablishConnection();
        }
        catch(...)
        {
            ROS_INFO("ERROR ERROR ERROR");
        }
        flag_connected_ = true;
    }   
}

// Move to receive EEF pose at a slow speed
void RobotROSInterface::RobotEEFMotionCallBack(const geometry_msgs::Pose::ConstPtr& msg)
{
    std::vector<double> quat{
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    };
    std::vector<double> eul = quat2eul(quat);
    std::vector<double> epos{
        msg->position.x,
        msg->position.y,
        msg->position.z,
        eul[0],
        eul[1],
        eul[2]
    };
    try
    {
        kst_.PTPLineEEF(epos, /*DON'T CHANGE!*/12.0); // second parameter is mm/sec
    }
    catch(...)
    {
        ROS_INFO("ERROR ERROR ERROR");
    }
    
}

// Should be only called when terminating node
void RobotROSInterface::RobotTerminateNodeCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (!flag_end_handshaked_) ROS_INFO("Robot connection ending signal received by ROS node");
    if (msg->data.compare("_end_robot_connection_")==0 && !flag_end_handshaked_)
    {
        flag_end_handshaked_ = true;
        try
        {
            kst_.NetTurnoffServer();
        }
        catch(...)
        {
            ROS_INFO("ERROR ERROR ERROR");
        }
        
        ROS_INFO("Ending connection signal sent to cabinet");
    }
}

// Get joint positions
void RobotROSInterface::RobotGetJntPosCallBack(const std_msgs::String::ConstPtr& msg)
{

}

// Get end-effector pose
void RobotROSInterface::RobotGetEEFPoseCallBack(const std_msgs::String::ConstPtr& msg)
{
    try
    {
        std::vector<double> vec = kst_.GetEEFPosition();
        geometry_msgs::Pose out;
        out.position.x = vec[0]; out.position.y = vec[1]; out.position.z = vec[2];
        std::vector<double> eul{vec[3],vec[4],vec[5]};
        std::vector<double> quat = eul2quat(eul);
        out.orientation.x = quat[0]; out.orientation.y = quat[1]; out.orientation.z = quat[2]; out.orientation.w = quat[3]; 
        pub_eef_.publish(out);
    }
    catch(...)
    {
        ROS_INFO("ERROR ERROR ERROR");
    }
}