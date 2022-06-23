#include "flag_machine.hpp"

FlagMachine::FlagMachine()
{
    // flag_landmark_planned_ = false;
    // flag_landmark_digitized_ = false;
    // flag_placepose_planned_ = false;
    // flag_registration_completed_ = false;
}

bool FlagMachine::flag_landmark_planned_;
bool FlagMachine::flag_landmark_digitized_;
bool FlagMachine::flag_placepose_planned_;
bool FlagMachine::flag_registration_completed_;

void FlagMachine::PlanLandmarks(){flag_landmark_planned_=true;}
void FlagMachine::DigitizeLandmarks(){flag_landmark_digitized_=true;}
void FlagMachine::PlanPlacePose(){flag_placepose_planned_=true;}
void FlagMachine::CompleteRegistration(){flag_registration_completed_=true;}

void FlagMachine::UnPlanLandmarks(){flag_landmark_planned_=false;}
void FlagMachine::UnDigitizeLandmarks(){flag_landmark_digitized_=false;}
void FlagMachine::UnPlanPlacePose(){flag_placepose_planned_=false;}
void FlagMachine::UnCompleteRegistration(){flag_registration_completed_=false;}