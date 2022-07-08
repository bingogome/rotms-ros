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

#include "flag_machine.hpp"
#include <vector>

FlagMachine::FlagMachine()
{
    flag_robot_conn_status_ = false;

    flag_landmark_planned_ = false;
    flag_landmark_digitized_ = false;
    flag_toolpose_planned_ = false;
    flag_registration_completed_ = false;
}

// Robot connection status flag
bool FlagMachine::flag_robot_conn_status_;

// Crucial operations status flags
bool FlagMachine::flag_landmark_planned_;
bool FlagMachine::flag_landmark_digitized_;
bool FlagMachine::flag_toolpose_planned_;
bool FlagMachine::flag_registration_completed_;

// Robot connection status setters and getters
void FlagMachine::ConnectRobot(){flag_robot_conn_status_=true;}
void FlagMachine::DisconnectRobot(){flag_robot_conn_status_=false;}
bool FlagMachine::GetFlagRobotConnStatus(){return flag_robot_conn_status_;}

// Crucial operations status setters and getters
void FlagMachine::PlanLandmarks(){flag_landmark_planned_=true;}
void FlagMachine::DigitizeLandmarks(){flag_landmark_digitized_=true;}
void FlagMachine::PlanToolPose(){flag_toolpose_planned_=true;}
void FlagMachine::CompleteRegistration(){flag_registration_completed_=true;}

void FlagMachine::UnPlanLandmarks(){flag_landmark_planned_=false;}
void FlagMachine::UnDigitizeLandmarks(){flag_landmark_digitized_=false;}
void FlagMachine::UnPlanToolPose(){flag_toolpose_planned_=false;}
void FlagMachine::UnCompleteRegistration(){flag_registration_completed_=false;}

bool FlagMachine::GetFlagLandmarkPlanned(){return flag_landmark_planned_;}
bool FlagMachine::GetFlagLandmarkDigitized(){return flag_landmark_digitized_;}
bool FlagMachine::GetFlagToolPosePlanned(){return flag_toolpose_planned_;}
bool FlagMachine::GetFlagRegistered(){return flag_registration_completed_;}