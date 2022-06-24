#include "flag_machine.hpp"
#include <vector>

FlagMachine::FlagMachine()
{
    flag_landmark_planned_ = false;
    flag_landmark_digitized_ = false;
    flag_toolpose_planned_ = false;
    flag_registration_completed_ = false;
}

bool FlagMachine::flag_landmark_planned_;
bool FlagMachine::flag_landmark_digitized_;
bool FlagMachine::flag_toolpose_planned_;
bool FlagMachine::flag_registration_completed_;

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