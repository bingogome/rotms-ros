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

class TransformMngrHeadTool
{
// Manages the flag of running the calculation of the transform
public:
    
    TransformMngrHeadTool(ros::NodeHandle& n) : n_(n){}
    bool run_flag = false;

private:

    ros::NodeHandle& n_;
    ros::Subscriber sub_run_ = n_.subscribe(
        "/Kinematics/Flag_head_tool", 2, &TransformMngrHeadTool::FlagCallBack, this);
    void FlagCallBack(const std_msgs::String::ConstPtr& msg)
    {
        if(msg->data.compare("_start__")==0) run_flag = true;
        if(msg->data.compare("_end__")==0) run_flag = false;
    }
};


int main(int argc, char **argv)
{
    // ROS stuff
    ros::init(argc, argv, "NodeVizTrHeadTool");
    ros::NodeHandle nh;
    ros::Rate rate(30.0);

    // Instantiate the flag manager
    TransformMngrHeadTool mngr(nh);

    // Initialize the requred transforms

    // Initialize the tf2 intermediate variables

    // Initialize the result variable and its publisher
    ros::Publisher pub_encode_head_tool = nh.advertise<std_msgs::String>(
        "/TargetVizComm/msg_to_send", 5);
    std_msg::String msg_out;

    // Go in the loop, with the flag indicating wether do the calculation or not
    while (nh.ok())
    {
        if (mngr1.run_flag)
        {
            
            
            pub_encode_head_tool.publish(tr_bodyref_ptrtip.position);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}