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

#include <ros/package.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <yaml-cpp/yaml.h>

void validateResult(Eigen::Matrix4d ots2base_mtx);
void setWaypoint(const geometry_msgs::Pose &waypoint);
void getOTSHeadRefCallback(const geometry_msgs::TransformStamped &msg);
void getOTSCoilRefCallback(const geometry_msgs::TransformStamped &msg);
void getOTSPointerCallback(const geometry_msgs::TransformStamped &msg);
void loadConstantHandEye(const std_msgs::Float64MultiArray &msg);
Eigen::Matrix4d loadConstantHandEye();
Eigen::Matrix4d getKUKABase2EEF(float x, float y, float z, float alpha, float beta, float gamma);

ros::Publisher pub_waypoint;
ros::Subscriber sub_ots_headref;
ros::Subscriber sub_ots_coilref;
ros::Subscriber sub_ots_pointer;

Eigen::Matrix4d ots2headref_mtx = Eigen::Matrix4d::Identity();
Eigen::Matrix4d ots2coilref_mtx = Eigen::Matrix4d::Identity();
Eigen::Matrix4d ots2pointer_mtx = Eigen::Matrix4d::Identity();
Eigen::Matrix4d ots2ptrtip_mtx = Eigen::Matrix4d::Identity();

Eigen::Matrix4d pointer2tip_mtx = Eigen::Matrix4d::Identity();
Eigen::Matrix4d base2eef = Eigen::Matrix4d::Identity();
Eigen::Matrix4d base2ots = Eigen::Matrix4d::Identity();


void setWaypoint(const geometry_msgs::Pose &msg)
{
    int x = 0;
}

void getOTSHeadRefCallback(const geometry_msgs::TransformStamped &msg)
{
    ots2headref_mtx(0, 3) = msg.transform.translation.x;
    ots2headref_mtx(1, 3) = msg.transform.translation.y;
    ots2headref_mtx(2, 3) = msg.transform.translation.z;
    Eigen::Quaterniond quat(msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z);
    ots2headref_mtx.block<3, 3>(0, 0) = quat.toRotationMatrix();
}

void getOTSCoilRefCallback(const geometry_msgs::TransformStamped &msg)
{
    ots2coilref_mtx(0, 3) = msg.transform.translation.x;
    ots2coilref_mtx(1, 3) = msg.transform.translation.y;
    ots2coilref_mtx(2, 3) = msg.transform.translation.z;
    Eigen::Quaterniond quat(msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z);
    ots2coilref_mtx.block<3, 3>(0, 0) = quat.toRotationMatrix();
}

void getOTSPointerCallback(const geometry_msgs::TransformStamped &msg)
{
    ots2pointer_mtx(0, 3) = msg.transform.translation.x;
    ots2pointer_mtx(1, 3) = msg.transform.translation.y;
    ots2pointer_mtx(2, 3) = msg.transform.translation.z;
    Eigen::Quaterniond quat(msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z);
    ots2pointer_mtx.block<3, 3>(0, 0) = quat.toRotationMatrix();

    ots2ptrtip_mtx = ots2pointer_mtx * pointer2tip_mtx;

    validateResult(base2ots);
}

Eigen::Matrix4d loadConstantHandEye()
{

    std::string packpath = ros::package::getPath("rotms_ros_kinematics");
    YAML::Node f = YAML::LoadFile(packpath + "/share/toolref_eff.yaml");

    Eigen::Matrix4d toolref2eef_mtx;
    toolref2eef_mtx << 1, 0, 0, f["x"].as<double>(),
        0, 1, 0, f["y"].as<double>(),
        0, 0, 1, f["z"].as<double>(),
        0, 0, 0, 1;

    Eigen::Quaterniond handeye_quat(f["rw"].as<double>(), f["rx"].as<double>(), f["ry"].as<double>(), f["rz"].as<double>());
    toolref2eef_mtx.block<3, 3>(0, 0) = handeye_quat.toRotationMatrix();
    return toolref2eef_mtx.inverse();
}

Eigen::Matrix4d getKUKABase2EEF(float x, float y, float z, float alpha, float beta, float gamma)
{

    // Convert angles to radians
    double a = M_PI * alpha / 180.0;
    double b = M_PI * beta / 180.0;
    double g = M_PI * gamma / 180.0;

    // Compute trigonometric values
    double ca = cos(a), sa = sin(a);
    double cb = cos(b), sb = sin(b);
    double cg = cos(g), sg = sin(g);

    // Compute rotation matrix R = R_z(a) * R_y(b) * R_x(g)
    Eigen::Matrix3d rotation_matrix_z;
    rotation_matrix_z << ca, -sa, 0,
        sa, ca, 0,
        0, 0, 1;
    Eigen::Matrix3d rotation_matrix_y;
    rotation_matrix_y << cb, 0, sb,
        0, 1, 0,
        -sb, 0, cb;
    Eigen::Matrix3d rotation_matrix_x;
    rotation_matrix_x << 1, 0, 0,
        0, cg, -sg,
        0, sg, cg;
    Eigen::Matrix3d rotation_matrix = rotation_matrix_z * rotation_matrix_y * rotation_matrix_x;
    // Construct the homogeneous transformation matrix
    base2eef.block<3, 3>(0, 0) = rotation_matrix;
    base2eef(0, 3) = x;
    base2eef(1, 3) = y;
    base2eef(2, 3) = z;

    Eigen::Quaterniond quat(base2eef.block<3, 3>(0, 0));
    std::cout << "====================================\n"
              << "base2eef rotation as quaternion (w, x, y, z)\n "
              << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z() 
              << "\n====================================\n";

    return base2eef;
}

void loadPointer2Tip(){
    std::string packpath = ros::package::getPath("rotms_ros_kinematics");
    YAML::Node f = YAML::LoadFile(packpath + "/share/ptr_ptrtip.yaml");

    pointer2tip_mtx << 1, 0, 0, f["x"].as<double>(),
        0, 1, 0, f["y"].as<double>(),
        0, 0, 1, f["z"].as<double>(),
        0, 0, 0, 1;
}

void validateResult(Eigen::Matrix4d base_to_ots)
{ 
    std::cout << "base2ots: " << std::endl
              << base_to_ots << std::endl;

    std::cout << "====================================" << std::endl;
    std::cout << "ots2base: " << std::endl
              << base_to_ots.inverse() << std::endl;

    std::cout << "====================================" << std::endl;
    Eigen::Matrix4d base2pointer = base_to_ots * ots2ptrtip_mtx;
    std::cout << "base2pointer: " << std::endl
              << base2pointer << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fri_trajectory_node");
    ros::NodeHandle nh;
    pub_waypoint = nh.advertise<geometry_msgs::Pose>("/waypoint", 1);
    sub_ots_headref = nh.subscribe("/NDI/HeadRef/measured_cp", 1, getOTSHeadRefCallback);
    sub_ots_coilref = nh.subscribe("/NDI/CoilRef/measured_cp", 1, getOTSCoilRefCallback);
    sub_ots_pointer = nh.subscribe("/NDI/PointerNew/measured_cp", 1, getOTSPointerCallback);

    float x, y, z, alpha, beta, gamma;

    // Taking user input from the command line
    std::cout << "Enter x, y, z (translation in mm) and alpha, beta, gamma (rotation in degrees): ";
    std::cin >> x >> y >> z >> alpha >> beta >> gamma;

    Eigen::Matrix4d base2eef = getKUKABase2EEF(x, y, z, alpha, beta, gamma);
    // print base2eef rotation as quaternion


    Eigen::Matrix4d eef2toolref = loadConstantHandEye();
    loadPointer2Tip();

    ros::spinOnce(); // Call the callback functions once to get the initial values

    base2ots = base2eef * eef2toolref * ots2coilref_mtx.inverse();

    validateResult(base2ots);

    ros::Rate rate(50);
    while (ros::ok())
    {
        base2ots = base2eef * eef2toolref * ots2coilref_mtx.inverse();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
