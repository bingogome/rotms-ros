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

#include "rotms_dispatcher.hpp"

#include <ros/ros.h>

Dispatcher::Dispatcher(ros::NodeHandle& n, std::vector<WorkState>& states) 
    : n_(n), states_(state)
{

}

void Dispatcher::RegistrationCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data.compare("_register__")==0)
    {
        int new_state = states_[activated_state_].Registered();
        if (new_state != -1)
        {
            activated_state_ = new_state;
        }
        else
        {
            // Failed operation
            // TODO
        }
    }
    
}

void Dispatcher::DigitizationCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data.compare("_digitize__")==0)
    {
        int new_state = states_[activated_state_].LandmarksDigitized();
        if (new_state != -1)
        {
            activated_state_ = new_state;
        }
        else
        {
            // Failed operation
            // TODO
        }
    }
}