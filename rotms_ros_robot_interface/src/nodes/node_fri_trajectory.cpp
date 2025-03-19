#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>

void setWaypoint(const geometry_msgs::Pose& waypoint);
void getOTSHeadRefCallback(const geometry_msgs::PoseStamped& msg);
void getOTSCoilRefCallback(const geometry_msgs::PoseStamped& msg);
void getConstantHandEye(const std_msgs::Float64MultiArray& msg);
void update();
void reset();

ros::Publisher pub_waypoint;
ros::Subscriber sub_ots_headref;
ros::Subscriber sub_ots_coilref;
ros::Subscriber sub_constant_hand_eye;

geometry_msgs::Pose headref_pose;
geometry_msgs::Pose coilref_pose;
geometry_msgs::Pose handeye_pose;

Eigen::Matrix4d ots2coilref_mtx = Eigen::Matrix4d::Identity();
Eigen::Matrix4d eef2base = Eigen::Matrix4d::Identity();

void setWaypoint(const geometry_msgs::Pose& msg){
    geometry_msgs::PoseStamped waypoint;
    waypoint.header.frame_id = "world";
    waypoint.header.stamp = ros::Time::now();
    waypoint.pose = msg;
    pub_waypoint.publish(waypoint);
}

void getOTSHeadRefCallback(const geometry_msgs::PoseStamped& msg){
    geometry_msgs::Pose headref_pose = msg.pose;
}

void getOTSCoilRefCallback(const geometry_msgs::PoseStamped& msg){
    geometry_msgs::Pose coilref_pose = msg.pose;
}

void getConstantHandEye(const std_msgs::Float64MultiArray& msg){
    if(msg.data.size() != 7){
        ROS_ERROR("Invalid hand-eye calibration data received");
        return;
    }
    handeye_pose.position.x = msg.data[0];
    handeye_pose.position.y = msg.data[1];
    handeye_pose.position.z = msg.data[2];
    handeye_pose.orientation.x = msg.data[3];
    handeye_pose.orientation.y = msg.data[4];
    handeye_pose.orientation.z = msg.data[5];
    handeye_pose.orientation.w = msg.data[6];

    // calculate the robot end-effector to coil reference transformation
    Eigen::Matrix4d toolref2eef_mtx;
    toolref2eef_mtx << 1, 0, 0, handeye_pose.position.x,
                      0, 1, 0, handeye_pose.position.y,
                      0, 0, 1, handeye_pose.position.z,
                      0, 0, 0, 1;
    Eigen::Quaterniond handeye_quat(handeye_pose.orientation.w, handeye_pose.orientation.x, handeye_pose.orientation.y, handeye_pose.orientation.z);
    toolref2eef_mtx.block<3,3>(0,0) = handeye_quat.toRotationMatrix();

    // retrieve the coil reference to ots transformation
    ots2coilref_mtx(0, 3) = coilref_pose.position.x;
    ots2coilref_mtx(1, 3) = coilref_pose.position.y;
    ots2coilref_mtx(2, 3) = coilref_pose.position.z;

    Eigen::Quaterniond coilref_quat(coilref_pose.orientation.w, coilref_pose.orientation.x, coilref_pose.orientation.y, coilref_pose.orientation.z);
    ots2coilref_mtx.block<3,3>(0,0) = coilref_quat.toRotationMatrix();

    // calculate the ots to robot base transformation
    Eigen::Matrix4d ots2base_mtx = ots2coilref_mtx * toolref2eef_mtx.inverse();
    ots2base_mtx = ots2base_mtx * eef2base;
    
}

Eigen::Matrix4d getKUKAEEF2BASE(float x, float y, float z, float alpha, float beta, float gamma) {
    
    
    // Convert angles to radians
    double a = M_PI * alpha / 180.0;
    double b = M_PI * beta / 180.0;
    double g = M_PI * gamma / 180.0;

    // Compute trigonometric values
    double ca = cos(a), sa = sin(a);
    double cb = cos(b), sb = sin(b);
    double cg = cos(g), sg = sin(g);

    // Compute rotation matrix R = R_y(γ) * R_x(β) * R_z(α)
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << 
        ca * cg - sa * sb * sg, -sa * cb, ca * sg + sa * sb * cg,
        sa * cg + ca * sb * sg,  ca * cb, sa * sg - ca * sb * cg,
        -cb * sg,                 sb,     cb * cg;

    // Construct the homogeneous transformation matrix
    eef2base.block<3,3>(0,0) = rotation_matrix;
    eef2base(0,3) = x;
    eef2base(1,3) = y;
    eef2base(2,3) = z;

    return eef2base;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "fri_trajectory_node");
    ros::NodeHandle nh;
    pub_waypoint = nh.advertise<geometry_msgs::Pose>("/waypoint", 1);
    sub_ots_headref = nh.subscribe("/NDI/HeadRef/measured_cp", 1, getOTSHeadRefCallback);
    sub_ots_coilref = nh.subscribe("/NDI/CoilRef/measured_cp", 1, getOTSCoilRefCallback);
    sub_constant_hand_eye = nh.subscribe("/Kinematics/TR_toolref_eff", 1, getConstantHandEye);

    float x, y, z, alpha, beta, gamma;

    // Taking user input from the command line
    std::cout << "Enter x, y, z (translation in mm) and alpha, beta, gamma (rotation in degrees): ";
    std::cin >> x >> y >> z >> alpha >> beta >> gamma;

    Eigen::Matrix4d eef2base = getKUKAEEF2BASE(x, y, z, alpha, beta, gamma);

    ros::Rate rate(50);
    while(ros::ok()){
        update();
        rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}