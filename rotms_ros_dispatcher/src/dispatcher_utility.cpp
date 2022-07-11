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
#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <time.h>

#include <std_msgs/Float32MultiArray.h>


void SaveLandmarkPlanData(struct VolatileTempDataCache datacache, std::string f)
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
        filesave.close();
    }
}

void SaveToolPoseData(struct VolatileTempDataCache datacache, std::string f)
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
	strftime (buffer,80,"%Y%m%d_%I%M%p",timeinfo);

	return buffer;
}