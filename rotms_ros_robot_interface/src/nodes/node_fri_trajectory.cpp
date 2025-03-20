/**
 * MIT License
 * 
 * Copyright (c) 2022 Yihao Liu, Johns Hopkins University
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 #include <ros/ros.h>
 #include <ros/package.h>
 #include <geometry_msgs/Pose.h>
 #include <geometry_msgs/TransformStamped.h>
 #include <std_msgs/Float64MultiArray.h>
 #include <eigen3/Eigen/Eigen>
 #include <eigen3/Eigen/Dense>
 #include <yaml-cpp/yaml.h>
 #include <iostream>
 #include <cmath>
 
 class TrajectoryNode {
 public:
     // Constructor: initializes ROS publishers, subscribers, and transformation matrices.
     TrajectoryNode(ros::NodeHandle &nh)
     : ots2headref_mtx_(Eigen::Matrix4d::Identity()),
       ots2coilref_mtx_(Eigen::Matrix4d::Identity()),
       ots2pointer_mtx_(Eigen::Matrix4d::Identity()),
       ots2ptrtip_mtx_(Eigen::Matrix4d::Identity()),
       pointer2tip_mtx_(Eigen::Matrix4d::Identity()),
       base2eef_(Eigen::Matrix4d::Identity()),
       base2ots_(Eigen::Matrix4d::Identity()),
       base2headref_(Eigen::Matrix4d::Identity())
     {
         pub_waypoint_ = nh.advertise<geometry_msgs::Pose>("/waypoint", 1);
         sub_ots_headref_ = nh.subscribe("/NDI/HeadRef/measured_cp", 1, &TrajectoryNode::getOTSHeadRefCallback, this);
         sub_ots_coilref_ = nh.subscribe("/NDI/CoilRef/measured_cp", 1, &TrajectoryNode::getOTSCoilRefCallback, this);
         sub_ots_pointer_ = nh.subscribe("/NDI/PointerNew/measured_cp", 1, &TrajectoryNode::getOTSPointerCallback, this);
     }
 
     // Main loop: collects user input, initializes parameters, and runs the periodic loop.
     void run() {
         // Take user input for the robot's translation and rotation
         float x, y, z, alpha, beta, gamma;
         std::cout << "Enter x, y, z (translation in mm) and alpha, beta, gamma (rotation in degrees): ";
         std::cin >> x >> y >> z >> alpha >> beta >> gamma;
 
         // Compute base-to-end-effector transformation from input values
         base2eef_ = getKUKABase2EEF(x, y, z, alpha, beta, gamma);
 
         // Load constant hand-eye calibration and pointer-to-tip transformation
         Eigen::Matrix4d eef2toolref = loadConstantHandEye();
         loadPointer2Tip();
 
         // Process any initial callbacks to update transformation matrices
         ros::spinOnce();
 
         // Compute the initial base-to-OTS transformation
         base2ots_ = base2eef_ * eef2toolref * ots2coilref_mtx_.inverse();
         validateResult(base2ots_);
 
         ros::Rate rate(50);
         while (ros::ok()) {
             // Update transformation and publish waypoint
             base2ots_ = base2eef_ * eef2toolref * ots2coilref_mtx_.inverse();
             sendWayPoint();
             ros::spinOnce();
             rate.sleep();
         }
     }
 
 private:
     // ROS communication members
     ros::Publisher pub_waypoint_;
     ros::Subscriber sub_ots_headref_;
     ros::Subscriber sub_ots_coilref_;
     ros::Subscriber sub_ots_pointer_;
 
     // Transformation matrices (using Eigen)
     Eigen::Matrix4d ots2headref_mtx_;
     Eigen::Matrix4d ots2coilref_mtx_;
     Eigen::Matrix4d ots2pointer_mtx_;
     Eigen::Matrix4d ots2ptrtip_mtx_;
     Eigen::Matrix4d pointer2tip_mtx_;
     Eigen::Matrix4d base2eef_;
     Eigen::Matrix4d base2ots_;
     Eigen::Matrix4d base2headref_;
 
     // Waypoint pose message
     geometry_msgs::Pose waypoint_;
 
     // Publishes the computed waypoint based on the current base-to-OTS transformation.
     void sendWayPoint() {
         waypoint_.position.x = base2ots_(0, 3) - 0.1;
         waypoint_.position.y = base2ots_(1, 3);
         waypoint_.position.z = base2ots_(2, 3) + 0.2;
 
         Eigen::Quaterniond quat(base2ots_.block<3, 3>(0, 0));
         waypoint_.orientation.x = quat.x();
         waypoint_.orientation.y = quat.y();
         waypoint_.orientation.z = quat.z();
         waypoint_.orientation.w = quat.w();
 
         pub_waypoint_.publish(waypoint_);
     }
 
     // Callback: updates the transformation matrix for the HeadRef.
     void getOTSHeadRefCallback(const geometry_msgs::TransformStamped &msg) {
         ots2headref_mtx_(0, 3) = msg.transform.translation.x;
         ots2headref_mtx_(1, 3) = msg.transform.translation.y;
         ots2headref_mtx_(2, 3) = msg.transform.translation.z;
         Eigen::Quaterniond quat(msg.transform.rotation.w,
                                 msg.transform.rotation.x,
                                 msg.transform.rotation.y,
                                 msg.transform.rotation.z);
         ots2headref_mtx_.block<3, 3>(0, 0) = quat.toRotationMatrix();
     }
 
     // Callback: updates the transformation matrix for the CoilRef and computes base-to-head reference.
     void getOTSCoilRefCallback(const geometry_msgs::TransformStamped &msg) {
         ots2coilref_mtx_(0, 3) = msg.transform.translation.x;
         ots2coilref_mtx_(1, 3) = msg.transform.translation.y;
         ots2coilref_mtx_(2, 3) = msg.transform.translation.z;
         Eigen::Quaterniond quat(msg.transform.rotation.w,
                                 msg.transform.rotation.x,
                                 msg.transform.rotation.y,
                                 msg.transform.rotation.z);
         ots2coilref_mtx_.block<3, 3>(0, 0) = quat.toRotationMatrix();
 
         // Update base-to-headref transformation using the current base2ots matrix
         base2headref_ = base2ots_ * ots2headref_mtx_;
     }
 
     // Callback: updates the transformation matrix for the Pointer and computes the pointer tip transform.
     void getOTSPointerCallback(const geometry_msgs::TransformStamped &msg) {
         ots2pointer_mtx_(0, 3) = msg.transform.translation.x;
         ots2pointer_mtx_(1, 3) = msg.transform.translation.y;
         ots2pointer_mtx_(2, 3) = msg.transform.translation.z;
         Eigen::Quaterniond quat(msg.transform.rotation.w,
                                 msg.transform.rotation.x,
                                 msg.transform.rotation.y,
                                 msg.transform.rotation.z);
         ots2pointer_mtx_.block<3, 3>(0, 0) = quat.toRotationMatrix();
 
         // Compute pointer tip transformation based on the loaded pointer-to-tip matrix.
         ots2ptrtip_mtx_ = ots2pointer_mtx_ * pointer2tip_mtx_;
 
         validateResult(base2ots_);
     }
 
     // Loads the constant hand-eye calibration transformation from a YAML file.
     Eigen::Matrix4d loadConstantHandEye() {
         std::string packpath = ros::package::getPath("rotms_ros_kinematics");
         YAML::Node f = YAML::LoadFile(packpath + "/share/toolref_eff.yaml");
 
         Eigen::Matrix4d toolref2eef_mtx;
         toolref2eef_mtx << 1, 0, 0, f["x"].as<double>(),
                            0, 1, 0, f["y"].as<double>(),
                            0, 0, 1, f["z"].as<double>(),
                            0, 0, 0, 1;
 
         Eigen::Quaterniond handeye_quat(f["rw"].as<double>(),
                                         f["rx"].as<double>(),
                                         f["ry"].as<double>(),
                                         f["rz"].as<double>());
         toolref2eef_mtx.block<3, 3>(0, 0) = handeye_quat.toRotationMatrix();
         return toolref2eef_mtx.inverse();
     }
 
     // Loads the pointer-to-tip transformation from a YAML file.
     void loadPointer2Tip() {
         std::string packpath = ros::package::getPath("rotms_ros_kinematics");
         YAML::Node f = YAML::LoadFile(packpath + "/share/ptr_ptrtip.yaml");
 
         pointer2tip_mtx_ << 1, 0, 0, f["x"].as<double>(),
                             0, 1, 0, f["y"].as<double>(),
                             0, 0, 1, f["z"].as<double>(),
                             0, 0, 0, 1;
     }
 
     // Computes the KUKA base-to-end-effector transformation given translation (in mm) and rotation (in degrees).
     Eigen::Matrix4d getKUKABase2EEF(float x, float y, float z, float alpha, float beta, float gamma) {
         // Convert angles from degrees to radians
         double a = M_PI * alpha / 180.0;
         double b = M_PI * beta / 180.0;
         double g = M_PI * gamma / 180.0;
 
         // Compute trigonometric values for each rotation axis
         double ca = cos(a), sa = sin(a);
         double cb = cos(b), sb = sin(b);
         double cg = cos(g), sg = sin(g);
 
         // Compute individual rotation matrices
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
 
         // Compose the overall rotation matrix: R = R_z * R_y * R_x
         Eigen::Matrix3d rotation_matrix = rotation_matrix_z * rotation_matrix_y * rotation_matrix_x;
         
         // Construct the homogeneous transformation matrix
         Eigen::Matrix4d base2eef;
         base2eef.setIdentity();
         base2eef.block<3, 3>(0, 0) = rotation_matrix;
         base2eef(0, 3) = x;
         base2eef(1, 3) = y;
         base2eef(2, 3) = z;
 
         // Print the computed rotation as a quaternion for verification
         Eigen::Quaterniond quat(rotation_matrix);
         std::cout << "====================================\n"
                   << "base2eef rotation as quaternion (w, x, y, z):\n"
                   << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z() 
                   << "\n====================================\n";
 
         return base2eef;
     }
 
     // Validates and prints transformation results for debugging purposes.
     void validateResult(const Eigen::Matrix4d &base_to_ots) { 
         std::cout << "base2ots:" << std::endl << base_to_ots << std::endl;
         std::cout << "====================================" << std::endl;
         std::cout << "ots2base:" << std::endl << base_to_ots.inverse() << std::endl;
         std::cout << "====================================" << std::endl;
         Eigen::Matrix4d base2pointer = base_to_ots * ots2ptrtip_mtx_;
         std::cout << "base2pointer:" << std::endl << base2pointer << std::endl;
     }
 };
 
 int main(int argc, char **argv) {
     ros::init(argc, argv, "fri_trajectory_node");
     ros::NodeHandle nh;
 
     // Instantiate and run the trajectory node
     TrajectoryNode node(nh);
     node.run();
 
     return 0;
 }
 