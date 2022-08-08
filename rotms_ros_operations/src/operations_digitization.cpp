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

#include "operations_digitization.hpp"
#include <std_msgs/Int32.h>

void OperationsDigitization::OperationDigitizationAll()
{
    // Get meta data of planned landmarks
    std::string packpath = ros::package::getPath("rotms_ros_operations");
    YAML::Node f = YAML::LoadFile(packpath + "/share/cache/landmarkplan.yaml");
    int num_of_landmarks = f["NUM"].as<int>();

    ROS_INFO_STREAM(num_of_landmarks);
    datacache_.landmark_total = num_of_landmarks;

    // Poke opttracker_tr_bodyref_ptrtip node /Kinematics/Flag_bodyref_ptrtip
    std_msgs::String flag_start;
    flag_start.data = "_start__";
    pub_run_opttracker_tr_bodyref_ptrtip_.publish(flag_start);

    // Beep the Opttracker 3 times to indicate get prepared for digitization
    ros::Publisher pub_beep = n_.advertise<std_msgs::Int32>("/NDI/beep", 10);
    ros::Duration(3).sleep();
    std_msgs::Int32 beep_num;
    beep_num.data = 3;
    pub_beep.publish(beep_num); 
    ros::spinOnce();

    // Wait 7 seconds to get prepared
    ros::Duration(7).sleep();

    // Start digitization
    for(int i=0;i<num_of_landmarks;i++)
    {
        // Beep 2 times for each landmark
        beep_num.data = 2;
        ros::Duration(7).sleep();
        pub_beep.publish(beep_num); ros::spinOnce();
        // Wait for a 
        geometry_msgs::PointConstPtr curdigPtr = ros::topic::waitForMessage<geometry_msgs::Point>("/Kinematics/T_bodyref_ptrtip");
        std::vector<double> curlandmark{curdigPtr->x, curdigPtr->y, curdigPtr->z};
        datacache_.landmarkdig.push_back(curlandmark);
        ROS_GREEN_STREAM("[ROTMS INFO] User digitized one point " + std::to_string(i));
    }

    // Poke opttracker_tr_bodyref_ptrtip node /Kinematics/Flag_bodyref_ptrtip
    flag_start.data = "_end__";
    pub_run_opttracker_tr_bodyref_ptrtip_.publish(flag_start);

    // Check validity and save
    if (datacache_.landmarkdig.size()!=num_of_landmarks)
    {
        ResetOpsVolatileDataCache();
        throw std::runtime_error(
            "Number of the digitized landmarks does not match!");
    }
    else
    {
        SaveLandmarkDigData(datacache_, 
            packpath + "/share/data/landmarkdig_"+ GetTimeString() + ".yaml");
        SaveLandmarkDigData(datacache_, 
            packpath + "/share/cache/landmarkdig" + ".yaml");
    }

}

void OperationsDigitization::ResetOpsVolatileDataCache()
{
    datacache_.landmark_total = -1;
    datacache_.landmarkdig.clear();
}

void OperationsDigitization::OperationDigitizeOne()
{
    if(temp_dig_idx_==-1)
    {
        ROS_RED_STREAM("[ROTMS ERROR] Temp digitization index is not set!");
        return;
    }

    // Read YAML file in
    std::string packpath = ros::package::getPath("rotms_ros_operations");
    YAML::Node f2 = YAML::LoadFile(packpath + "/share/cache/landmarkdig.yaml");
    YAML::Node ff2 = f2["DIGITIZED"];
    std::vector<std::vector<double>> clouddig;

    for(YAML::const_iterator it=ff2.begin(); it!=ff2.end(); ++it)
    {
        YAML::Node value = it->second;
        std::vector<double> temppnt = {
            value["x"].as<double>(),
            value["y"].as<double>(),
            value["z"].as<double>()
        };
        clouddig.push_back(temppnt);
    }
    ROS_GREEN_STREAM("[ROTMS INFO] Digitized landmark size: " + std::to_string(clouddig.size()));
    
    // Digitize one from sensor
    // Beep 2 times for each landmark
    ros::Publisher pub_beep = n_.advertise<std_msgs::Int32>("/NDI/beep", 10);
    std_msgs::Int32 beep_num; beep_num.data = 2;
    pub_beep.publish(beep_num); ros::spinOnce();
    // Wait for a 
    geometry_msgs::PointConstPtr curdigPtr = ros::topic::waitForMessage<geometry_msgs::Point>("/Kinematics/T_bodyref_ptrtip");
    std::vector<double> curlandmark{curdigPtr->x, curdigPtr->y, curdigPtr->z};
    clouddig[temp_dig_idx_] = curlandmark;
    ROS_GREEN_STREAM("[ROTMS INFO] User digitized one point " + std::to_string(temp_dig_idx_));

    datacache_.landmark_total = clouddig.size();
    datacache_.landmarkdig = clouddig;

    // Cache
    SaveLandmarkDigData(datacache_, packpath + "/share/cache/landmarkdig.yaml");

    // Reset temp data
    OperationsDigitization::ResetOpsVolatileDataCache();
    OperationsDigitization::ClearTempDigitizationIdx();
}

void OperationsDigitization::SetTempDigitizationIdx(int idx)
{
    temp_dig_idx_ = idx;
}

void OperationsDigitization::ClearTempDigitizationIdx()
{
    temp_dig_idx_ = -1;
}

void SaveLandmarkDigData(struct TempDataCacheOps datacache, std::string f)
{
    std::ofstream filesave(f);
    if(filesave.is_open())
    {
        filesave << "NUM: " << datacache.landmark_total << "\n";
        filesave << "\n";
        filesave << "DIGITIZED: # in m\n";
        filesave << "\n";
        filesave << "  {\n";
        for(int i=0;i<datacache.landmark_total;i++)
        {
            std::vector<std::vector<double>> c = datacache.landmarkdig;
            filesave << "    d" << i << ": ";
            filesave << "{";
            filesave << "x: " << FormatDouble2String(c[i][0], 16) << ", ";
            filesave << "y: " << FormatDouble2String(c[i][1], 16) << ", ";
            filesave << "z: " << FormatDouble2String(c[i][2], 16);
            if (i==datacache.landmark_total-1)
                filesave << "}\n";
            else
                filesave << "},\n";
        }
        filesave << "  }\n";
        filesave.close();
    }
}