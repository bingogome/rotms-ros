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
#include "flag_machine.hpp"
#include "state_machine_states.hpp"
#include "rotms_operations.hpp"
#include "ros_print_color.hpp"

#include <ros/ros.h>
#include <tuple>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "NodeDispatcher");
    ros::NodeHandle nh;

    ROS_GREEN_STREAM("[ROTMS INFO] Dispatcher on.");

    // Initialize flags, states, operations and pass to dispatcher
    FlagMachine f = FlagMachine();
    TMSOperations ops = TMSOperations(nh);

    // WARNING: this function will return a vector of pointers
    // Remember to release memory !!
    // In this node, the memory is released by Dispatcher when 
    // destroying the Dispatcher object
    const std::vector<WorkState*> states = GetStatesVector(f, ops);

    // Initialize dispatcher
    Dispatcher d = Dispatcher(nh, states);

    ros::spin();

    return 0;
}