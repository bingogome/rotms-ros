/***
MIT License

Copyright (c) 2023 Yihao Liu, Johns Hopkins University

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
#include <geometry_msgs/PoseArray.h>

std::string FormatDouble2String(double a, int dec)
{
	std::stringstream stream;
    stream << std::fixed << std::setprecision(dec) << a;
    std::string s = stream.str();
    return s;
}

class MngrCommRelay
{

public:

    MngrCommRelay(ros::NodeHandle& n) : n_(n){}

private:

    ros::NodeHandle& n_;
    ros::Subscriber sub_xr2slicer_planmsg = n_.subscribe(
        "/Relay/XR2Slicer/PlanMsg", 2, &MngrCommRelay::XR2SlicerPlanMsgCallBack, this);
    ros::Publiser pub_xr2slicer_planmsg = n_.advertise<std_msgs::String>(
        "/MedImgComm/msg_to_send_hi_f", 2);

    void XR2SlicerPlanMsgCallBack(const geometry_msgs::PoseArray::ConstPtr& msg)
    {
        // receive 2 points for tool pose plan. unit is m
        // send to slicer unit convert to mm
        std_msgs::String msg_out;
        msg_out.data = "__msg_toolpose_" + // convert m to mm
            FormatDouble2String(msg.poses[0].position.x * 1000.0, 5) + "_" +
            FormatDouble2String(msg.poses[0].position.y * 1000.0, 5) + "_" +
            FormatDouble2String(msg.poses[0].position.z * 1000.0, 5) + "_" +
            FormatDouble2String(msg.poses[1].position.x * 1000.0, 5) + "_" +
            FormatDouble2String(msg.poses[1].position.y * 1000.0, 5) + "_" +
            FormatDouble2String(msg.poses[1].position.z * 1000.0, 5);
        pub_xr2slicer_planmsg.publish(msg_out);
    }
};

int main(int argc, char **argv)
{
    // ROS stuff
    ros::init(argc, argv, "NodeCommRelay");
    ros::NodeHandle nh;
    ros::Rate rate(60.0);

    // Instantiate the manager
    MngrCommRelay mngr(nh);

    while (nh.ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}