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

#include "dispatcher_utility.hpp"
#include "rotms_dispatcher.hpp"
#include "state_machine_toolplan_states.hpp"
#include "state_machine_registration_states.hpp"
#include "state_machine_digitization_states.hpp"
#include "state_machine_robot_states.hpp"
#include "ros_print_color.hpp"

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2/LinearMath/Transform.h>
#include <rotms_ros_msgs/GetJnts.h>
#include <rotms_ros_msgs/GetEFF.h>
#include <rotms_ros_msgs/PoseValid.h>

/*
Dispatcher takes decoded messages from comm_decode and preprocesses
volatile data that have "receiving bursts" (eg. The operation plan 
landmarks will receive NUM_LANDMARKS messages at one request. These 
messages will be preprocessed by dispatcher and cached in /share/cache 
for state/flag/operation for postprocessing.
On the other hand, state/flag/operation will postprocess any data 
that have been preprocessed and cached.
*/
Dispatcher::Dispatcher(ros::NodeHandle& n, struct StateSet& states_set) 
    : n_(n), states_set_(states_set)
{
    bool integ;

    integ = CheckFlagIntegrityRegistration(states_set_.state_registration);
    ROS_GREEN_STREAM("[ROTMS INFO] Flag integrity check (Registration): " + std::to_string(integ));
    if (!integ) ROS_RED_STREAM("[ROTMS ERROR] Flag integrity check (Registration) failed! ");

    integ = CheckFlagIntegrityDigitization(states_set_.state_digitization);
    ROS_GREEN_STREAM("[ROTMS INFO] Flag integrity check (Digitization): " + std::to_string(integ));
    if (!integ) ROS_RED_STREAM("[ROTMS ERROR] Flag integrity check (Digitization) failed! ");

    integ = CheckFlagIntegrityToolplan(states_set_.state_toolplan);
    ROS_GREEN_STREAM("[ROTMS INFO] Flag integrity check (Tool plan): " + std::to_string(integ));
    if (!integ) ROS_RED_STREAM("[ROTMS ERROR] Flag integrity check (Tool plan) failed! ");

    integ = CheckFlagIntegrityRobot(states_set_.state_robot);
    ROS_GREEN_STREAM("[ROTMS INFO] Flag integrity check (Robot): " + std::to_string(integ));
    if (!integ) ROS_RED_STREAM("[ROTMS ERROR] Flag integrity check (Robot) failed! ");
}

void Dispatcher::LandmarkPlanMetaCallBack(const std_msgs::Int16::ConstPtr& msg)
{
    if (msg->data==-99) // signals all landmarks plan have been received.
    {
        std::string packpath = ros::package::getPath("rotms_ros_operations");

        if (datacache_.landmark_coords.size() != datacache_.landmark_total)
        {
            ROS_YELLOW_STREAM("[ROTMS WARNING] The number of landmarks received does not match planned!");
            ROS_YELLOW_STREAM("[ROTMS WARNING] Planned: " + std::to_string(datacache_.landmark_total));
            ROS_YELLOW_STREAM("[ROTMS WARNING] Received: " + std::to_string(datacache_.landmark_coords.size()));
            ROS_YELLOW_STREAM("[ROTMS WARNING] Try one more time!");
            Dispatcher::ResetVolatileDataCacheLandmarks();
            return;
        }

        SaveLandmarkPlanData(datacache_, packpath + "/share/cache/landmarkplan.yaml", GetTimeString());
        SaveLandmarkPlanData(datacache_, packpath + "/share/data/landmarkplan_" + GetTimeString() + ".yaml", GetTimeString());

        ROS_GREEN_STREAM("[ROTMS INFO] Landmarks cached.");

        Dispatcher::ResetVolatileDataCacheLandmarks(); // reset

        // First plan landmarks, then init digitization state machine
        int new_state_registration = states_set_.state_registration[activated_state_["REGISTRATION"]]->LandmarksPlanned();
        Dispatcher::StateTransitionCheck(new_state_registration, "REGISTRATION");
        int new_state_digitization = states_set_.state_digitization[activated_state_["DIGITIZATION"]]->ReinitState();
        Dispatcher::StateTransitionCheck(new_state_digitization, "DIGITIZATION");
    }
    else
    {
        datacache_.landmark_total = msg->data;
    }
}

void Dispatcher::LandmarkPlanLandmarksCallBack(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data[0]==datacache_.landmark_coords.size())
    {
        std::vector<double> cur_fid{
            msg->data[1],msg->data[2],msg->data[3],};
        datacache_.landmark_coords.push_back(cur_fid);
    }
    else
    {
        ROS_YELLOW_STREAM("[ROTMS WARNING] Current index does not match the waiting index!");
        ROS_YELLOW_STREAM("[ROTMS WARNING] Current index: " + std::to_string(msg->data[0]));
        ROS_YELLOW_STREAM("[ROTMS WARNING] Waiting index: " + std::to_string(datacache_.landmark_coords.size()));
        ROS_YELLOW_STREAM("[ROTMS WARNING] Try one more time!");
        Dispatcher::ResetVolatileDataCacheLandmarks();
    }
}

void Dispatcher::SessionReinitCallBack(const std_msgs::String::ConstPtr& msg)
{
    if(!msg->data.compare("_reinit__")==0) return; 

    // Reset state
    int new_state_registration = states_set_.state_registration[activated_state_["REGISTRATION"]]->ReinitState();
    Dispatcher::StateTransitionCheck(new_state_registration, "REGISTRATION");
    int new_state_toolplan = states_set_.state_toolplan[activated_state_["TOOLPLAN"]]->ReinitState();
    Dispatcher::StateTransitionCheck(new_state_toolplan, "TOOLPLAN");
    int new_state_digitization = states_set_.state_digitization[activated_state_["DIGITIZATION"]]->ReinitState();
    Dispatcher::StateTransitionCheck(new_state_digitization, "DIGITIZATION");

    // Reinit calibration data
    std_msgs::String msg_out;
    msg_out.data = "_reinit__";
    pub_reinitcaldata_.publish(msg_out);

    // Verbo
    ROS_GREEN_STREAM("[ROTMS INFO] Session reinited");
}

void Dispatcher::ResetVolatileDataCacheLandmarks()
{
    datacache_.landmark_total = -1;
    datacache_.landmark_coords.clear();
}

void Dispatcher::DigitizationCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data.compare("_autodigitize__")==0)
    {
        int new_state_digitization = states_set_.state_digitization[activated_state_["DIGITIZATION"]]->DigitizeAllLandmarks();
        Dispatcher::StateTransitionCheck(new_state_digitization, "DIGITIZATION");
    }
    else
    {
        if(msg->data.rfind("digitize_one_",0)==0)
        {
            int dig_idx = std::stoi(msg->data.substr(13));
            ROS_GREEN_STREAM("[ROTMS INFO] Digitize individual landmark: " + std::to_string(dig_idx));
            int new_state_digitization = states_set_.state_digitization[activated_state_["DIGITIZATION"]]->RedigitizeOneLandmark(dig_idx);
            Dispatcher::StateTransitionCheck(new_state_digitization, "DIGITIZATION");
        }
        if(msg->data.rfind("use_prev_digitize_digitize_one_",0)==0)
        {
            std::string command_msgdata = "use_prev_digitize_digitize_one_";
            int dig_idx = std::stoi(msg->data.substr(command_msgdata.length()));
            ROS_GREEN_STREAM("[ROTMS INFO] Digitize individual landmark: " + std::to_string(dig_idx));
            int new_state_digitization = states_set_.state_digitization[activated_state_["DIGITIZATION"]]->UsePrevDigAndRedigOneLandmark(dig_idx);
            Dispatcher::StateTransitionCheck(new_state_digitization, "DIGITIZATION");
        }
        else if (msg->data.compare("use_prev_digitize")==0)
        {
            int new_state_digitization = states_set_.state_digitization[activated_state_["DIGITIZATION"]]->ConfirmAllDigitized();
            Dispatcher::StateTransitionCheck(new_state_digitization, "DIGITIZATION");
        }
        
    }
    
}

void Dispatcher::RegistrationCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data.compare("_prevregister__")==0)
    {
        int new_state = 
            states_set_.state_registration[activated_state_["REGISTRATION"]]->UsePrevRegister();
        Dispatcher::StateTransitionCheck(new_state, "REGISTRATION");
        Dispatcher::RegistrationResidualCheck();
        Dispatcher::RegistrationToXR();
    }
    if (msg->data.compare("_register__")==0)
    {
        int new_state = 
            states_set_.state_registration[activated_state_["REGISTRATION"]]->Registered();
        Dispatcher::StateTransitionCheck(new_state, "REGISTRATION");
        Dispatcher::RegistrationResidualCheck();
        Dispatcher::RegistrationToXR();
    }
    
}

void Dispatcher::RegistrationResidualCheck()
{
    std::string packpath = ros::package::getPath("rotms_ros_operations");

    std::vector<tf2::Vector3> cloudpln = 
        ReadPointCloudFromYAML(packpath + "/share/cache/landmarkplan.yaml", "PLANNED");
    std::vector<tf2::Vector3> clouddig =
        ReadPointCloudFromYAML(packpath + "/share/cache/landmarkdig.yaml", "DIGITIZED");
    
    tf2::Transform reg = ReadTransformFromYAML(packpath + "/share/config/reg.yaml");

    double resid = GetPairPointResidual(reg, cloudpln, clouddig);

    std_msgs::String resid_msg;
    resid_msg.data = std::to_string(resid * 1000.0); // convert to mm
    pub_medplancomm_.publish(resid_msg);
}

void Dispatcher::TRECalculationCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data.compare("_starttre__")==0)
    {
        if(activated_state_["REGISTRATION"] != 0b111)
        {
            ROS_YELLOW_STREAM("Current state: " + std::to_string(activated_state_["REGISTRATION"]));
            ROS_YELLOW_STREAM("[ROTMS WARNING] Prerequisite is not met!");
            return;
        }
        // Poke opttracker_tr_bodyref_ptrtip node /Kinematics/Flag_bodyref_ptrtip
        std_msgs::String flag_opttracker;
        flag_opttracker.data = "_start__";
        pub_run_opttracker_tr_bodyref_ptrtip_.publish(flag_opttracker);
        // Poke node_TRE_t_body_ptrtip
        std_msgs::String msg_out;
        msg_out.data = "_start__";
        pub_flag_t_body_ptrtip_.publish(msg_out);
    }
    if (msg->data.compare("_stoptre__")==0)
    {
        // Poke node_TRE_t_body_ptrtip
        std_msgs::String msg_out;
        msg_out.data = "_end__";
        pub_flag_t_body_ptrtip_.publish(msg_out);
        // Poke opttracker_tr_bodyref_ptrtip node /Kinematics/Flag_bodyref_ptrtip
        std_msgs::String flag_opttracker;
        flag_opttracker.data = "_end__";
        pub_run_opttracker_tr_bodyref_ptrtip_.publish(flag_opttracker);
    }
}

void Dispatcher::ICPCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data.compare("icp_digitize")==0)
    {
        std_msgs::String msg_out;
        msg_out.data = "_start__";
        pub_icp_dig_.publish(msg_out);
    }
    if (msg->data.compare("icp_clear_prev")==0)
    {
        // Get the number of poins in a cloud
        std::string packpath_utility    = ros::package::getPath("rotms_ros_utility");
        YAML::Node node_icpconfig       = YAML::LoadFile(packpath_utility + "/icp_config.yaml");
        int num_pnts_acloud             = node_icpconfig["NUM_PNTS_IN_ACLOUD"].as<int>();

        // Get the number of clouds
        std::string packpath_operations = ros::package::getPath("rotms_ros_operations");
        YAML::Node node_icpdig          = YAML::LoadFile(packpath_operations + "/share/config/icpdig.yaml");
        int cloudctr = 0;
        for(YAML::const_iterator it=node_icpdig.begin(); it!=node_icpdig.end(); ++it) 
        {
            cloudctr++;
        }
        
        // Overwrite the file with only the clouds except for the last one 
        // (effectively "clear the previously digitized cloud")
        std::ofstream fstream;
        fstream.open(packpath_operations + "/share/config/icpdig.yaml");
        for(int i = 0; i<cloudctr-1; i++)
        {
            fstream << "points" + std::to_string(i) + ": \"";
            fstream << node_icpdig["points"+std::to_string(i)].as<std::string>();
            fstream << "\"\n";
        }
        fstream.close();
    }
    if (msg->data.compare("icp_clear_all")==0)
    {
        std::ofstream f;
        std::string packpath = ros::package::getPath("rotms_ros_operations");
        std::string filename = packpath + "/share/config/icpdig.yaml";
        f.open(filename);
        f.close();
    }
    if (msg->data.rfind("icp_register_",0)==0)
    {
        std::string command_msgdata = "icp_register_";
        std::string meshpath = msg->data.substr(command_msgdata.length());
        ROS_GREEN_STREAM("[ROTMS INFO] Received meshpath: " + meshpath);
        std_msgs::String msg_out;
        msg_out.data = meshpath;
        pub_icp_doicp_.publish(msg_out);
    }
}

void Dispatcher::ToolPoseOrientCallBack(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    std::vector<double> toolpose_r{
        msg->x, msg->y, msg->z, msg->w
    };
    datacache_.toolpose_r = toolpose_r;
    datacache_.toolpose_r_recvd = true;
    if (datacache_.toolpose_t_recvd == true)
    {
        
        std::string packpath = ros::package::getPath("rotms_ros_operations");
        SaveToolPoseData(datacache_, packpath + "/share/config/toolpose.yaml");
        SaveToolPoseData(datacache_, packpath + "/share/data/toolpose_" + GetTimeString() + ".yaml");

        ROS_GREEN_STREAM("[ROTMS INFO] Toolpose cached.");
        Dispatcher::ResetVolatileDataCacheToolPose();

        int new_state = states_set_.state_toolplan[activated_state_["TOOLPLAN"]]->ToolPosePlanned();
        Dispatcher::StateTransitionCheck(new_state, "TOOLPLAN");

        Dispatcher::ToolPoseTargetToXR();
    }
}

void Dispatcher::ToolPoseTransCallBack(const geometry_msgs::Point::ConstPtr& msg)
{
    std::vector<double> toolpose_t{
        msg->x, msg->y, msg->z
    };
    datacache_.toolpose_t = toolpose_t;
    datacache_.toolpose_t_recvd = true;
    if (datacache_.toolpose_r_recvd == true)
    {
        std::string packpath = ros::package::getPath("rotms_ros_operations");
        SaveToolPoseData(datacache_, packpath + "/share/config/toolpose.yaml");
        SaveToolPoseData(datacache_, packpath + "/share/data/toolpose_" + GetTimeString() + ".yaml");

        ROS_GREEN_STREAM("[ROTMS INFO] Toolpose cached.");
        Dispatcher::ResetVolatileDataCacheToolPose();

        int new_state = states_set_.state_toolplan[activated_state_["TOOLPLAN"]]->ToolPosePlanned();
        Dispatcher::StateTransitionCheck(new_state, "TOOLPLAN");

        Dispatcher::ToolPoseTargetToXR();
    }
}

void Dispatcher::ToolPoseTargetToXR()
{
    std::string packpath    = ros::package::getPath("rotms_ros_operations");
    YAML::Node f            = YAML::LoadFile(packpath + "/share/config/toolpose.yaml");
    YAML::Node ff1          = f["TRANSLATION"];
    YAML::Node ff2          = f["ROTATION"];

    std_msgs::String msg;

    msg.data = "trgt_" +
        FormatDouble2String(ff1["x"].as<double>(), 7) + "_" +
        FormatDouble2String(ff1["y"].as<double>(), 7) + "_" +
        FormatDouble2String(ff1["z"].as<double>(), 7) + "_" + 
        FormatDouble2String(ff2["x"].as<double>(), 7) + "_" + 
        FormatDouble2String(ff2["y"].as<double>(), 7) + "_" + 
        FormatDouble2String(ff2["z"].as<double>(), 7) + "_" +
        FormatDouble2String(ff2["w"].as<double>(), 7);
    
    pub_xr_.publish(msg);
}

void Dispatcher::RegistrationToXR()
{
    std::string packpath    = ros::package::getPath("rotms_ros_operations");
    YAML::Node f            = YAML::LoadFile(packpath + "/share/config/reg.yaml");
    YAML::Node ff1          = f["TRANSLATION"];
    YAML::Node ff2          = f["ROTATION"];

    std_msgs::String msg;

    msg.data = "regi_" +
        FormatDouble2String(ff1["x"].as<double>(), 7) + "_" +
        FormatDouble2String(ff1["y"].as<double>(), 7) + "_" +
        FormatDouble2String(ff1["z"].as<double>(), 7) + "_" + 
        FormatDouble2String(ff2["x"].as<double>(), 7) + "_" + 
        FormatDouble2String(ff2["y"].as<double>(), 7) + "_" + 
        FormatDouble2String(ff2["z"].as<double>(), 7) + "_" +
        FormatDouble2String(ff2["w"].as<double>(), 7);
    
    pub_xr_.publish(msg);
}

void Dispatcher::UpdateRobotConnFlagCallBack(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data) states_set_.state_robot[activated_state_["ROBOT"]]->flags_.ConnectRobot();
    else states_set_.state_robot[activated_state_["ROBOT"]]->flags_.DisconnectRobot();
    bool integ = CheckFlagIntegrityRobot(states_set_.state_robot);
    ROS_GREEN_STREAM("[ROTMS INFO] Flag integrity check (Robot): " + std::to_string(integ));
    if (!integ) ROS_RED_STREAM("[ROTMS ERROR] Flag integrity check (Robot) failed! ");
}

void Dispatcher::RobConnectCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data.compare("_connect__")==0)
    {
        std_msgs::String msg_test;
        msg_test.data = "_start_robot_connection_";
        pub_init_conn_.publish(msg_test);
        int new_state = states_set_.state_robot[activated_state_["ROBOT"]]->ConnectRobot();
        Dispatcher::StateTransitionCheck(new_state, "ROBOT");
    }
}

void Dispatcher::RobDisconnectCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data.compare("_disconnect__")==0)
    {
        std_msgs::String msg_test;
        msg_test.data = "_end_robot_connection_";
        pub_init_conn_.publish(msg_test);
        int new_state = states_set_.state_robot[activated_state_["ROBOT"]]->DisconnectRobot();
        Dispatcher::StateTransitionCheck(new_state, "ROBOT");
    }
}

void Dispatcher::GetJntsCallBack(const std_msgs::String::ConstPtr& msg)
{
    if(! (msg->data.compare("_jnts__")==0 
        || 
        msg->data.compare("_initcur__")==0) ) return;
    
    std_msgs::String msg_out;
    if(!states_set_.state_robot[activated_state_["ROBOT"]]->flags_.GetFlagRobotConnStatus())
    {
        ROS_YELLOW_STREAM("[ROTMS WARNING] Robot cabinet connection has not been established!");
        msg_out.data = "no_data_returned;";
        pub_robctrlcomm_.publish(msg_out);
        return;
    }
    rotms_ros_msgs::GetJnts srv;
    if(clt_jnt_.call(srv))
    {
        std_msgs::Float32MultiArray jnts = srv.response.jnt;
        std::stringstream str;
        for(int i=0;i<jnts.layout.dim[0].size;i++)
        {
            str << std::to_string(jnts.data[i]) << "_";
        }
        msg_out.data = str.str();
        pub_robctrlcomm_.publish(msg_out);

        if(msg->data.compare("_initcur__")==0) 
        {
            ROS_GREEN_STREAM("[ROTMS INFO] Setting current joints as init.");
            std::string packpath = ros::package::getPath("rotms_ros_operations");
            SaveCurrentJntsAsInit(jnts, packpath + "/share/config/initjnts.yaml");
            SaveCurrentJntsAsInit(jnts, packpath + "/share/data/initjnts_" + GetTimeString() + ".yaml");
        }
    }
    
}

void Dispatcher::GetEFFCallBack(const std_msgs::String::ConstPtr& msg)
{
    if(!msg->data.compare("_eff__")==0) return;
    std_msgs::String msg_out;
    if(!states_set_.state_robot[activated_state_["ROBOT"]]->flags_.GetFlagRobotConnStatus())
    {
        ROS_YELLOW_STREAM("[ROTMS WARNING] Robot cabinet connection has not been established!");
        msg_out.data = "no_data_returned;";
        pub_robctrlcomm_.publish(msg_out);
        return;
    }
    rotms_ros_msgs::GetEFF srv;
    if(clt_eff_.call(srv))
    {
        geometry_msgs::Pose eff = srv.response.eff;
        std::stringstream str;
        str << std::to_string(eff.position.x) << "_";
        str << std::to_string(eff.position.y) << "_";
        str << std::to_string(eff.position.z) << "_";
        str << std::to_string(eff.orientation.x) << "_";
        str << std::to_string(eff.orientation.y) << "_";
        str << std::to_string(eff.orientation.z) << "_";
        str << std::to_string(eff.orientation.w) << "_";
        msg_out.data = str.str();
        pub_robctrlcomm_.publish(msg_out);
        pub_expiredeffold_.publish(eff);
    }
}

void Dispatcher::ResetVolatileDataCacheToolPose()
{
    datacache_.toolpose_t_recvd = false;
    datacache_.toolpose_r_recvd = false;
    datacache_.toolpose_t.clear();
    datacache_.toolpose_r.clear();
}

void Dispatcher::ExecuteMotionToOffsetCallBack(const std_msgs::String::ConstPtr& msg)
{
    // Status check
    if(!msg->data.compare("_execute__")==0) return;
    if(
        activated_state_["TOOLPLAN"]!=0b1 || 
        activated_state_["ROBOT"]!=0b1 ||
        activated_state_["REGISTRATION"]!=0b111)
    {
        ROS_YELLOW_STREAM("[ROTMS WARNING] The prerequisites are not met. Check before robot motion. (code 1)");
        return;
    }
    std_msgs::String msg_reinit;
    msg_reinit.data = "_reinitoffset__";
    pub_reinitoffset_.publish(msg_reinit);
    ros::spinOnce();
    Dispatcher::ExecuteMotionToTargetEFFPose();
}

void Dispatcher::ExecuteConfirmMotionCallBack(const std_msgs::String::ConstPtr& msg)
{
    // Status check
    if(!msg->data.compare("_confirm__")==0) return;
    if(
        activated_state_["TOOLPLAN"]!=0b1 || 
        activated_state_["ROBOT"]!=0b1 ||
        activated_state_["REGISTRATION"]!=0b111)
    {
        ROS_YELLOW_STREAM("[ROTMS WARNING] The prerequisites are not met. Check before robot motion. (code 2)");
        return;
    }
    geometry_msgs::Pose changeoffset;
    changeoffset.position.x = 0.0; changeoffset.position.y = 0.0; changeoffset.position.z = 0.010; // 1cm offset to make sure safety 
    changeoffset.orientation.x = 0.0; changeoffset.orientation.y = 0.0; changeoffset.orientation.z = 0.0;
    changeoffset.orientation.w = 1.0;
    pub_changeoffset_.publish(changeoffset);
    ros::spinOnce();
    Dispatcher::ExecuteMotionToTargetEFFPose();
}

void Dispatcher::ExecuteManualAdjust(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(
        activated_state_["TOOLPLAN"]!=0b1 || 
        activated_state_["ROBOT"]!=0b1 ||
        activated_state_["REGISTRATION"]!=0b111)
    {
        ROS_YELLOW_STREAM("[ROTMS WARNING] The prerequisites are not met. Check before robot motion. (code 2)");
        return;
    }
    geometry_msgs::PoseConstPtr tr_curoffset = ros::topic::waitForMessage<geometry_msgs::Pose>(
        "/Kinematics/TR_cntct_offset");
    std::vector<double> q_temp{
        tr_curoffset->orientation.x,tr_curoffset->orientation.y,
        tr_curoffset->orientation.z,tr_curoffset->orientation.w
    };
    std::vector<double> eul_temp = quat2eul(q_temp);
    std::vector<double> eul{
        eul_temp[0] + msg->data[3],
        eul_temp[1] + msg->data[4],
        eul_temp[2] + msg->data[5]
    };
    std::vector<double> quat = eul2quat(eul);
    geometry_msgs::Pose changeoffset;
    changeoffset.position.x = tr_curoffset->position.x + msg->data[0]; 
    changeoffset.position.y = tr_curoffset->position.y + msg->data[1]; 
    changeoffset.position.z = tr_curoffset->position.z + msg->data[2];
    changeoffset.orientation.x = quat[0]; 
    changeoffset.orientation.y = quat[1]; 
    changeoffset.orientation.z = quat[2];
    changeoffset.orientation.w = quat[3];
    pub_changeoffset_.publish(changeoffset);
    ros::spinOnce();
    Dispatcher::ExecuteMotionToTargetEFFPose();
}

void Dispatcher::ExecuteMotionToTargetEFFPose()
{   
    if(!states_set_.state_robot[activated_state_["ROBOT"]]->flags_.GetFlagRobotConnStatus())
    {
        ROS_YELLOW_STREAM("[ROTMS WARNING] Robot cabinet connection has not been established!");
        return;
    }
    if(
        activated_state_["TOOLPLAN"]!=0b1 || 
        activated_state_["ROBOT"]!=0b1 ||
        activated_state_["REGISTRATION"]!=0b111)
    {
        ROS_YELLOW_STREAM("[ROTMS WARNING] The prerequisites are not met. Check before robot motion. (code 0)");
        return;
    }

    // Query for current EFF pose and publish (latch)
    rotms_ros_msgs::GetEFF srv;
    if(!clt_eff_.call(srv))
    {
        ROS_YELLOW_STREAM("[ROTMS WARNING] Could not request current EFF from robot cabinet!");
        return;
    }
    geometry_msgs::Pose effold = srv.response.eff;
    rotms_ros_msgs::PoseValid pv_old;
    pv_old.valid = true;
    pv_old.pose = effold;
    pub_effold_.publish(pv_old);

    ROS_GREEN_STREAM("[ROTMS INFO] Current EFF pose received.");

    // Query for target EFF pose
    std_msgs::String queryeff;
    queryeff.data = "_gettargeteff__";
    pub_gettargeteff_.publish(queryeff);
    rotms_ros_msgs::PoseValidConstPtr tr_targeteff = ros::topic::waitForMessage<rotms_ros_msgs::PoseValid>(
        "/Kinematics/TR_derivedeff");

    ROS_GREEN_STREAM("[ROTMS INFO] Target pose received, test validaty......");

    ros::Rate check_valid_rate(10);
    while(!tr_targeteff->valid)
    {
        tr_targeteff = ros::topic::waitForMessage<rotms_ros_msgs::PoseValid>(
            "/Kinematics/TR_derivedeff");
        check_valid_rate.sleep();
    }
    
    ROS_GREEN_STREAM("[ROTMS INFO] Target pose valid. ");

    geometry_msgs::Pose tr_targeteff_;
    tr_targeteff_.position.x = tr_targeteff->pose.position.x;
    tr_targeteff_.position.y = tr_targeteff->pose.position.y;
    tr_targeteff_.position.z = tr_targeteff->pose.position.z;
    tr_targeteff_.orientation.x = tr_targeteff->pose.orientation.x;
    tr_targeteff_.orientation.y = tr_targeteff->pose.orientation.y;
    tr_targeteff_.orientation.z = tr_targeteff->pose.orientation.z;
    tr_targeteff_.orientation.w = tr_targeteff->pose.orientation.w;

    // Send to robot interface and move
    pub_robeffmove_.publish(tr_targeteff_);

    ROS_GREEN_STREAM("[ROTMS INFO] Send to robot interface. ");

    // Stop old EFF pose publisher latch
    rotms_ros_msgs::PoseValid pv;
    pv.valid = false;
    pub_effold_.publish(pv);

    ROS_GREEN_STREAM("[ROTMS INFO] End of robot motion call. ");
}

void Dispatcher::ExecuteBackToCallBack(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data.compare("_backoffset__")==0)
    {
        if(
            activated_state_["TOOLPLAN"]!=0b1 || 
            activated_state_["ROBOT"]!=0b1 ||
            activated_state_["REGISTRATION"]!=0b111)
        {
            ROS_YELLOW_STREAM("[ROTMS WARNING] The prerequisites are not met. Check before robot motion. (code 3)");
            return;
        }
        geometry_msgs::Pose changeoffset;
        std::string packpath = ros::package::getPath("rotms_ros_kinematics");
        YAML::Node f = YAML::LoadFile(packpath + "/share/cntct_offset.yaml");
        changeoffset.position.x = f["x"].as<double>();
        changeoffset.position.y = f["y"].as<double>();
        changeoffset.position.z = f["z"].as<double>();
        changeoffset.orientation.x = f["rx"].as<double>();
        changeoffset.orientation.y = f["ry"].as<double>();
        changeoffset.orientation.z = f["rz"].as<double>();
        changeoffset.orientation.w = f["rw"].as<double>();
        pub_changeoffset_.publish(changeoffset);
        Dispatcher::ExecuteMotionToTargetEFFPose();
    }
    if(msg->data.compare("_backinit__")==0)
    {
        std::string packpath = ros::package::getPath("rotms_ros_operations");
        std::vector<double> vec = ReadJntsFromConfig(packpath + "/share/config/initjnts.yaml");
        std_msgs::Float32MultiArray msg_out;
        msg_out.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg_out.layout.dim[0].size = vec.size(); 
        msg_out.layout.dim[0].stride = 1;
        msg_out.layout.dim[0].label = "format__a1_a2_a3_a4_a5_a6_a7_";
        msg_out.data.clear();
        msg_out.data.insert(msg_out.data.end(), vec.begin(), vec.end());
        pub_robjntmove_.publish(msg_out);
    }
    if(msg->data.compare("_robothoming__")==0) 
    {
        std::vector<double> vec{
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std_msgs::Float32MultiArray msg_out;
        msg_out.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg_out.layout.dim[0].size = vec.size(); 
        msg_out.layout.dim[0].stride = 1;
        msg_out.layout.dim[0].label = "format__a1_a2_a3_a4_a5_a6_a7_";
        msg_out.data.clear();
        msg_out.data.insert(msg_out.data.end(), vec.begin(), vec.end());
        pub_robjntmove_.publish(msg_out);
    }
}

void Dispatcher::TargetVizCallBack(const std_msgs::String::ConstPtr& msg)
{
    std_msgs::String msg_out;
    if(msg->data.compare("_start__")==0) 
    {
        if(activated_state_["REGISTRATION"]!=0b111)
        {
            ROS_YELLOW_STREAM("Current state: " + std::to_string(activated_state_["REGISTRATION"]));
            ROS_YELLOW_STREAM("[ROTMS WARNING] Prerequisite is not met!");
            return;
        }
        // Poke node_viz_tr_body_tool
        msg_out.data = "_start__";
        pub_flag_bodytoolviz_.publish(msg_out);
    }
    else if(msg->data.compare("_end__")==0) 
    {
        // Poke node_viz_tr_body_tool
        msg_out.data = "_end__";
        pub_flag_bodytoolviz_.publish(msg_out);
    }
}

void Dispatcher::TargetVizSavePlanAndRealPoseCallBack(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data.compare("_save_plan_real__")==0)
    {
        ROS_GREEN_STREAM("[ROTMS INFO] Saving the planned pose and the real pose. ");

        // Get the current pose from the visualization module interface
        std_msgs::StringConstPtr tr_measure = ros::topic::waitForMessage<std_msgs::String>(
            "/TargetVizComm/msg_to_send_hi_f"); 
        std::string str_ = tr_measure->data.substr(11);
        std::vector<double> str_arr = SubStringTokenize2Double(str_);

        geometry_msgs::Pose tr_measure_;
        tr_measure_.position.x = str_arr[0] / 1000.0;
        tr_measure_.position.y = str_arr[1] / 1000.0;
        tr_measure_.position.z = str_arr[2] / 1000.0;
        tr_measure_.orientation.x = str_arr[3] ;
        tr_measure_.orientation.y = str_arr[4] ;
        tr_measure_.orientation.z = str_arr[5] ;
        tr_measure_.orientation.w = str_arr[6] ;

        std::string packpath = ros::package::getPath("rotms_ros");
        SaveToolPosePlannedAndMeasured(packpath + "/saveddata/toolpose_" + GetTimeString() + ".yaml", tr_measure_);

        ROS_GREEN_STREAM("[ROTMS INFO] Saved the planned pose and the real pose. ");
    }
    else if(msg->data.compare("_save_continuous_pose__")==0)
    {
        ROS_GREEN_STREAM("[ROTMS INFO] Sent request saving the planned pose and the real pose. ");
        ROS_GREEN_STREAM("[ROTMS INFO] Saving the planned pose and the real pose, in continuous time. ");
        std_msgs::String msg_out;
        msg_out.data = "_start__";
        pub_save_continuous_pose_.publish(msg_out);
    }
}

void Dispatcher::MepDataRecordCallBack(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data.compare("_save_plan_real__")==0)
    {
        ROS_GREEN_STREAM("[ROTMS INFO] Sent request saving the planned pose and the real pose. ");
        ROS_GREEN_STREAM("[ROTMS INFO] Saving the planned pose and the real pose, in continuous time. ");
        std_msgs::String msg_out;
        msg_out.data = "_start__";
        pub_save_continuous_pose_.publish(msg_out);
    }
}

void Dispatcher::StateTransitionCheck(int new_state, std::string s)
{
    ROS_GREEN_STREAM("[ROTMS INFO] State transition - " + s);
    ROS_GREEN_STREAM("[ROTMS INFO] Old state: " + std::to_string(activated_state_[s]));
    ROS_GREEN_STREAM("[ROTMS INFO] Attempt new state: " + std::to_string(new_state));
    if (new_state != -1)
    {
        activated_state_[s] = new_state;
        ROS_GREEN_STREAM("[ROTMS INFO] Transitioned to new state: " + 
            std::to_string(activated_state_[s]));
    }
    else
    {
        // Failed operation
        ROS_YELLOW_STREAM("[ROTMS WARNING] State transition not possible.");
        ROS_YELLOW_STREAM("[ROTMS WARNING] Make sure the operation dependencies are met.");
    }

    // Low level state branch change will need to change dispatcher activated state record of the higher level branch.
    if(s.compare("DIGITIZATION")==0)
    {
        int activated_state_registration = StateRegistration::GetActivatedState(states_set_.state_registration);
        activated_state_["REGISTRATION"] = activated_state_registration;
    }

    ROS_GREEN_STREAM("[ROTMS INFO] Current states: Registration, Digitization, Toolplan, Robot");
    ROS_GREEN_STREAM("[ROTMS INFO] " + 
        std::to_string(activated_state_["REGISTRATION"]) + ", "+ 
        std::to_string(activated_state_["DIGITIZATION"]) + ", "+ 
        std::to_string(activated_state_["TOOLPLAN"]) + ", "+ 
        std::to_string(activated_state_["ROBOT"])
        );
}
