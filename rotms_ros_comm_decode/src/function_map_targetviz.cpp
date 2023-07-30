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

#include <map>
#include <string>
#include <ros/ros.h>
#include "function_map_targetviz.hpp"
#include "decode_node.hpp"

/**
* This maps the functions to the received cmd.
*/

CommDecoderTargetViz::CommDecoderTargetViz(
    ros::NodeHandle& n, 
    const std::string modulesuffix,
    FuncMap opsdict) 
    : 
    CommDecoder(n, modulesuffix, opsdict) 
{
    pubs_.push_back(
        n_.advertise<std_msgs::String>("/TargetViz/Visualize", 2));
    pubs_.push_back(
        n_.advertise<std_msgs::String>("/TargetViz/DataRecord", 2));
}

FuncMap GetFuncMapTargetViz()
{
    FuncMap fm;

    fm["VISUALIZE_START"] = VisualizeStart;
    fm["VISUALIZE_STOP"] = VisualizeStop;
    fm["VISUALIZE_SAVE_PLANANDREAL_POSE"] = VisualizeSavePlanAndRealPose;

    return fm;
}

void VisualizeStart(std::string& ss, PublisherVec& pubs)
{
    std_msgs::String msg_test;
    msg_test.data = "_start__";
    // pubs[0] is the publisher /TargetViz/Visualize
    pubs[0].publish(msg_test);
}

void VisualizeStop(std::string& ss, PublisherVec& pubs)
{
    std_msgs::String msg_test;
    msg_test.data = "_end__";
    // pubs[0] is the publisher /TargetViz/Visualize
    pubs[0].publish(msg_test);
}

void VisualizeSavePlanAndRealPose(std::string& ss, PublisherVec& pubs)
{
    std_msgs::String msg_test;
    msg_test.data = "_save_plan_real__";
    // pubs[1] is the publisher /TargetViz/DataRecord
    pubs[1].publish(msg_test);
}

void VisualizeSaveContinuousPose(std::string& ss, PublisherVec& pubs)
{
    std_msgs::String msg_test;
    msg_test.data = "_save_continuous_pose__";
    // pubs[1] is the publisher /TargetViz/DataRecord
    pubs[1].publish(msg_test);
}