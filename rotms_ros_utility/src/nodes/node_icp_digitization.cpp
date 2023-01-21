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

#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <fstream>

class Mngr
{
public:

    Mngr(ros::NodeHandle& n) : n_(n)
    {
        std::string packpath = ros::package::getPath("rotms_ros_utility");
        YAML::Node f = YAML::LoadFile(packpath + "/icp_config.yaml");
        num_pnts_acloud = f["NUM_PNTS_IN_ACLOUD"].as<int>();
    }
    bool run_flag = false;
    std::vector<std::vector<double>> current_cloud;
    geometry_msgs::Point current_bodyref_ptrtip;

    static int num_pnts_acloud;
    static float dig_freq = 10;

private:

    ros::NodeHandle& n_;
    ros::Subscriber sub_run_ = n_.subscribe(
        "/ICP/digitization", 2, &Mngr::FlagCallBack, this);
    ros::Subscriber sub_bodyref_ptrtip_ = n_.subscribe(
        "/Kinematics/T_bodyref_ptrtip", 2, &Mngr::BodyrefPtrtipCallBack, this);
    ros::Publisher pub_run_opttracker_tr_bodyref_ptrtip_ = 
        n_.advertise<std_msgs::String>("/Kinematics/Flag_bodyref_ptrtip", 2);
    ros::Publisher pub_beep_ = 
        n_.advertise<std_msgs::Int32>("/NDI/beep", 2);

    void FlagCallBack(const std_msgs::String::ConstPtr& msg)
    {
        if(msg->data.compare("_start__")==0)
        {
            // Poke opttracker_tr_bodyref_ptrtip node /Kinematics/Flag_bodyref_ptrtip
            std_msgs::String flag_start;
            flag_start.data = "_start__";
            pub_run_opttracker_tr_bodyref_ptrtip_.publish(flag_start);
            ros::spinOnce();

            ros::Duration(5).sleep();
            ros::spinOnce();

            // Beep 2 times 
            std_msgs::Int32 beep_num; beep_num.data = 2;
            pub_beep_.publish(beep_num); 
            ros::spinOnce();
            ros::Duration(0.5).sleep();

            // Start to receive points
            run_flag = true;
        } 
    }

    void BodyrefPtrtipCallBack(const geometry_msgs::Point::ConstPtr& msg)
    {
        current_bodyref_ptrtip.x = msg->x;
        current_bodyref_ptrtip.y = msg->y;
        current_bodyref_ptrtip.z = msg->z;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "NodeICPDigitization");
    ros::NodeHandle nh;
    ros::Rate rate(dig_freq);
    Mngr mngr1(nh);

    while (nh.ok())
    {
        if (mngr1.run_flag)
        {
            if (mngr1.current_cloud.size() >= num_pnts_acloud)
            {
                mngr1.run_flag = false;
                
                // Poke opttracker_tr_bodyref_ptrtip node /Kinematics/Flag_bodyref_ptrtip
                std_msgs::String flag_start;
                flag_start.data = "_end__";
                mngr1.pub_run_opttracker_tr_bodyref_ptrtip_.publish(flag_start);
                ros::spinOnce();

                // Save the cloud and clear the vector
                std::string packpath = ros::package::getPath("rotms_ros_operations");
                std::string filename = packpath + "/share/config/icpdig.yaml";
                YAML::Node f2 = YAML::LoadFile(filename);

                int cloudctr = 0;
                for(YAML::const_iterator it=f2.begin(); it!=f2.end(); ++it) cloudctr++;

                std::ofstream f;
                f.open(filename, std::ios_base::app); // appending
                f << "points" + std::to_string(cloudctr) + ": \"";
                for (int i=0; i<mngr1.current_cloud.size(); i++)
                {
                    f << 
                        std::to_string(mngr1.current_cloud[i][0]) << "," << 
                        std::to_string(mngr1.current_cloud[i][1]) << "," << 
                        std::to_string(mngr1.current_cloud[i][2]) << ",";
                }
                f << "\"\n";
                f.close();
                mngr1.current_cloud.clear();
            }
            else
            {
                vector<int> vect{ 
                    mngr1.current_bodyref_ptrtip.x, 
                    mngr1.current_bodyref_ptrtip.y, 
                    mngr1.current_bodyref_ptrtip.z 
                };
                mngr1.current_cloud.push_back(vect);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}