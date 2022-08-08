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

#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <time.h>
#include <math.h> 
#include <yaml-cpp/yaml.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2/LinearMath/Transform.h>


void SaveLandmarkPlanData(struct TempDataCache datacache, std::string f, std::string time_stamp)
{
    std::ofstream filesave(f);
    if(filesave.is_open())
    {
        filesave << "NUM: " << datacache.landmark_total << "\n";
        filesave << "\n";
        filesave << "PLANNED: # in m\n";
        filesave << "\n";
        filesave << "  {\n";
        for(int i=0;i<datacache.landmark_total;i++)
        {
            std::vector<std::vector<double>> c = datacache.landmark_coords;
            filesave << "    p" << i << ": ";
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
        filesave << " \n";
        filesave << "TIMESTAMP: " << time_stamp << "\n";
        filesave.close();
    }
}

void SaveToolPoseData(struct TempDataCache datacache, std::string f)
{
    std::ofstream filesave(f);
    if(filesave.is_open())
    {
        filesave << "TRANSLATION: # translation: x,y,z\n";
        filesave << "\n";
        filesave << "  {\n";
		filesave << "    x: " << FormatDouble2String(datacache.toolpose_t[0], 16) << ",\n";
		filesave << "    y: " << FormatDouble2String(datacache.toolpose_t[1], 16) << ",\n";
		filesave << "    z: " << FormatDouble2String(datacache.toolpose_t[2], 16) << "\n";
		filesave << "  }\n";
        filesave << "\n";
		filesave << "ROTATION: # quat: x,y,z,w\n";
        filesave << "\n";
        filesave << "  {\n";
		filesave << "    x: " << FormatDouble2String(datacache.toolpose_r[0], 16) << ",\n";
		filesave << "    y: " << FormatDouble2String(datacache.toolpose_r[1], 16) << ",\n";
		filesave << "    z: " << FormatDouble2String(datacache.toolpose_r[2], 16) << ",\n";
        filesave << "    w: " << FormatDouble2String(datacache.toolpose_r[3], 16) << "\n";
        filesave << "  }\n";
		filesave.close();
    }
}

void SaveCurrentJntsAsInit(std_msgs::Float32MultiArray jnts, std::string f)
{
    std::ofstream filesave(f);
    if(filesave.is_open())
    {
        for(int i=0;i<jnts.layout.dim[0].size;i++)
        {
            filesave << "a" << i << ": " << FormatDouble2String(jnts.data[i], 16) << "\n";
        }
        filesave.close();
    }
    
}

std::vector<double> ReadJntsFromConfig(std::string f)
{
    YAML::Node nd = YAML::LoadFile(f);
    int num = 0;
    std::vector<double> jnt;
    for(YAML::const_iterator it=nd.begin(); it!=nd.end(); ++it)
    {
        jnt.push_back(nd["a"+std::to_string(num)].as<double>());
        num++;
    }
    return jnt;
}

std::string FormatDouble2String(double a, int dec)
{
	std::stringstream stream;
    stream << std::fixed << std::setprecision(dec) << a;
    std::string s = stream.str();
    return s;
}

std::string GetTimeString()
{
	time_t rawtime;
	struct tm * timeinfo;
	char buffer [80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime (buffer,80,"%Y%m%d_%I%M%S%p",timeinfo);

	return buffer;
}

std::vector<tf2::Vector3> ReadPointCloudFromYAML(std::string f, std::string pnt)
{
    std::vector<tf2::Vector3> cloud;
    YAML::Node f1 = YAML::LoadFile(f);
    YAML::Node ff1 = f1[pnt];
    for(YAML::const_iterator it=ff1.begin(); it!=ff1.end(); ++it)
    {
        YAML::Node value = it->second;
        tf2::Vector3 temppnt{
            value["x"].as<double>(),
            value["y"].as<double>(),
            value["z"].as<double>()
        };
        cloud.push_back(temppnt);
    }
    return cloud;
}

tf2::Transform ReadTransformFromYAML(std::string f)
{
    YAML::Node reg = YAML::LoadFile(f);
    YAML::Node t_ = reg["TRANSLATION"];
    YAML::Node r_ = reg["ROTATION"];
    tf2::Vector3 t{
        t_["x"].as<double>(),
        t_["y"].as<double>(),
        t_["z"].as<double>()
    };
    tf2::Quaternion r{
        r_["x"].as<double>(),
        r_["y"].as<double>(),
        r_["z"].as<double>(),
        r_["w"].as<double>()
    };
    tf2::Transform out{
        r, t
    };
    return out;
}

double GetPairPointResidual(
    tf2::Transform tr, std::vector<tf2::Vector3> A, std::vector<tf2::Vector3> B)
{
    double out = 0;
    for(int i=0;i<A.size();i++)
    {
        tf2::Vector3 cur_err = tr * A[i];
        cur_err -= B[i];
        double x = cur_err.getX();
        double y = cur_err.getY();
        double z = cur_err.getZ();
        out += sqrt(x*x+y*y+z*z);
    }
    out /= A.size();
    return out;
}

std::vector<double> quat2eul(std::vector<double> q /*x,y,z,w*/)
{
    double aSinInput = -2.0 * (q[0] * q[2] - q[3] * q[1]);
    if(aSinInput > 1.0)
        aSinInput = 1.0;
    if(aSinInput < -1.0)
        aSinInput = -1.0;
    
    std::vector<double> ans{
        atan2( 2.0 * (q[0] * q[1] + q[3] * q[2]), q[3] * q[3] + q[0] * q[0] - q[1] * q[1] - q[2] * q[2] ), 
        asin( aSinInput ), 
        atan2( 2.0 * (q[1] * q[2] + q[3] * q[0]), q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2] )
    };
    return ans;
}

std::vector<double> eul2quat(std::vector<double> eul)
{
	std::vector<double> eulhalf{eul[0]/2,eul[1]/2,eul[2]/2};
    eul = eulhalf;
    std::vector<double> ans{
        cos(eul[0]) * cos(eul[1]) * sin(eul[2]) - sin(eul[0]) * sin(eul[1]) * cos(eul[2]),
        cos(eul[0]) * sin(eul[1]) * cos(eul[2]) + sin(eul[0]) * cos(eul[1]) * sin(eul[2]),
        sin(eul[0]) * cos(eul[1]) * cos(eul[2]) - cos(eul[0]) * sin(eul[1]) * sin(eul[2]),
        cos(eul[0]) * cos(eul[1]) * cos(eul[2]) + sin(eul[0]) * sin(eul[1]) * sin(eul[2])
    };
    return ans; // x y z w
}