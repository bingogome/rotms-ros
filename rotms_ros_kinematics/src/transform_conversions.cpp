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

#include "transform_conversions.hpp"
#include "rotms_ros_msgs/PoseValid.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>

tf2::Transform ConvertToTf2Transform(const geometry_msgs::PoseConstPtr tr)
{
    tf2::Vector3 v(
        tr->position.x, tr->position.y, tr->position.z);
    tf2::Quaternion q(
        tr->orientation.x, tr->orientation.y, tr->orientation.z,
        tr->orientation.w);

    tf2::Transform tr_(q, v);

    return tr_;
}

tf2::Transform ConvertToTf2Transform(const rotms_ros_msgs::PoseValidConstPtr tr)
{
    tf2::Vector3 v(
        tr->pose.position.x, tr->pose.position.y, tr->pose.position.z);
    tf2::Quaternion q(
        tr->pose.orientation.x, tr->pose.orientation.y, tr->pose.orientation.z,
        tr->pose.orientation.w);

    tf2::Transform tr_(q, v);

    return tr_;
}

tf2::Transform ConvertToTf2Transform(const geometry_msgs::TransformStampedConstPtr tr)
{
    tf2::Vector3 v(
        tr->transform.translation.x, tr->transform.translation.y, tr->transform.translation.z);
    tf2::Quaternion q(
        tr->transform.rotation.x, tr->transform.rotation.y, tr->transform.rotation.z,
        tr->transform.rotation.w);

    tf2::Transform tr_(q, v);
    return tr_;
}

tf2::Vector3 ConvertToTf2Vector3(const geometry_msgs::PointConstPtr t)
{
    tf2::Vector3 t_(t->x, t->y, t->z);
    return t_;
}

geometry_msgs::Pose ConvertToGeometryPose(const tf2::Transform tr)
{
    geometry_msgs::Pose tr_;
    tr_.position.x = tr.getOrigin().x();
    tr_.position.y = tr.getOrigin().y();
    tr_.position.z = tr.getOrigin().z();
    tr_.orientation.x = tr.getRotation().x();
    tr_.orientation.y = tr.getRotation().y();
    tr_.orientation.z = tr.getRotation().z();
    tr_.orientation.w = tr.getRotation().w();
    return tr_;
}