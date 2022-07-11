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
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <rotms_ros_msgs/GetJnts.h>
#include <rotms_ros_msgs/GetEFF.h>
#include <vector>
#include <math.h>
#include "rotms_robot_ros_interface.hpp"
#include "ros_print_color.hpp"

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

// Publish the robot connection status
void RobotROSInterface::RobotConnectStatusQuery(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data.compare("_query_robot_connection_status__")==0)
    {
        std_msgs::Bool robconnstatus;
        robconnstatus.data = flag_connected_;
        pub_robconnstatus_.publish(robconnstatus);
        ROS_GREEN_STREAM("[ROTMS INFO] Robot cabinet connection status: " + std::to_string(flag_connected_));
    }
}

// Start the cabinet connection
void RobotROSInterface::RobotInitConnectionCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data.compare("_start_robot_connection_")==0 && !flag_connected_)
    {
        try
        {
            kst_.NetEstablishConnection();
            flag_connected_ = true;
            ROS_GREEN_STREAM("[ROTMS INFO] Robot cabinet connection initialized.");
            std_msgs::Bool robconnstatus;
            robconnstatus.data = true;
            pub_robconnstatus_.publish(robconnstatus);
        }
        catch(...)
        {
            ROS_RED_STREAM("[ROTMS ERROR] ERROR ERROR ERROR (1)");
        }
    }   
}

// End the cabinet connection
void RobotROSInterface::RobotDisconnectCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data.compare("_end_robot_connection_")==0 && flag_connected_)
    {
        try
        {
            kst_.NetTurnoffServer();
            flag_connected_ = false;
            ROS_GREEN_STREAM("[ROTMS INFO] Robot cabinet disconnected.");
            std_msgs::Bool robconnstatus;
            robconnstatus.data = false;
            pub_robconnstatus_.publish(robconnstatus);
        }
        catch(...)
        {
            ROS_RED_STREAM("[ROTMS ERROR] ERROR ERROR ERROR (2)");
        }
        
    }   
}

// Move to received EFF pose at a slow speed
void RobotROSInterface::RobotEFFMotionCallBack(const geometry_msgs::Pose::ConstPtr& msg)
{
    if(!flag_connected_)
    {
        ROS_YELLOW_STREAM("[ROTMS WARNING] Robot cabinet connection has not been established! (1)");
        return;
    }
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
        kst_.PTPLineEFF(epos, /*DON'T CHANGE!*/9.0); // second parameter is mm/sec
    }
    catch(...)
    {
        ROS_RED_STREAM("[ROTMS ERROR] ERROR ERROR ERROR (3.1)");
    }
}

void RobotROSInterface::RobotJntMotionCallBack(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(!flag_connected_)
    {
        ROS_YELLOW_STREAM("[ROTMS WARNING] Robot cabinet connection has not been established! (1)");
        return;
    }
    std::vector<double> jnts; 
    for(int i=0;i<msg->layout.dim[0].size;i++)
    {
        jnts.push_back(msg->data[i]);
    }
    try
    {
        kst_.PTPJointSpace(jnts, /*DON'T CHANGE!*/0.02); // relative to max speed
    }
    catch(...)
    {
        ROS_RED_STREAM("[ROTMS ERROR] ERROR ERROR ERROR (3.1)");
    }
}

// Should be only called when terminating node
void RobotROSInterface::RobotTerminateNodeCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (!flag_end_handshaked_) ROS_GREEN_STREAM("[ROTMS INFO] Robot connection ending signal received by ROS node.");
    if (msg->data.compare("_end_robot_connection_")==0 && !flag_end_handshaked_)
    {
        flag_end_handshaked_ = true;
        try
        {
            kst_.NetTurnoffServer();
            flag_connected_ = false;
            ROS_GREEN_STREAM("[ROTMS INFO] Ending connection signal sent to cabinet.");
            std_msgs::Bool robconnstatus;
            robconnstatus.data = false;
            pub_robconnstatus_.publish(robconnstatus);
        }
        catch(...)
        {
            ROS_RED_STREAM("[ROTMS ERROR] ERROR ERROR ERROR (4)");
        }
    }
}

// Get joint positions
bool RobotROSInterface::RobotGetJntsPosCallBack(
    rotms_ros_msgs::GetJnts::Request &req, rotms_ros_msgs::GetJnts::Response &res)
{
    std::vector<double> vec;
    if(!flag_connected_)
    {
        ROS_YELLOW_STREAM("[ROTMS WARNING] Robot cabinet connection has not been established! (2)");
        return false;
    }
    try
    {
        vec = kst_.GetJointPosition();
    }
    catch(...)
    {
        ROS_RED_STREAM("[ROTMS ERROR] ERROR ERROR ERROR (5)");
        return false;
    }   
    std_msgs::Float32MultiArray msg;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = 7; // Kuka iiwa has 7 joints
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "format__";
    msg.data.clear();
    msg.data.insert(msg.data.end(), vec.begin(), vec.end());
    pub_jnt_.publish(msg);
    res.jnt = msg;
    
    return true;
}

// Get end-effector pose
bool RobotROSInterface::RobotGetEFFPoseCallBack(
    rotms_ros_msgs::GetEFF::Request &req, rotms_ros_msgs::GetEFF::Response &res)
{
    if(!flag_connected_)
    {
        ROS_YELLOW_STREAM("[ROTMS WARNING] Robot cabinet connection has not been established! (3)");
        return false;
    }
    try
    {
        std::vector<double> vec = kst_.GetEFFPosition();
        geometry_msgs::Pose out;
        out.position.x = vec[0]; out.position.y = vec[1]; out.position.z = vec[2];
        std::vector<double> eul{vec[3],vec[4],vec[5]};
        std::vector<double> quat = eul2quat(eul);
        out.orientation.x = quat[0]; out.orientation.y = quat[1]; out.orientation.z = quat[2]; out.orientation.w = quat[3]; 
        pub_eff_.publish(out);
        res.eff = out;
    }
    catch(...)
    {
        ROS_RED_STREAM("[ROTMS ERROR] ERROR ERROR ERROR (6)");
        return false;
    }
    return true;
}
