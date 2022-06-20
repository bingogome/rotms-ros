#include "state_machine.hpp"
#include <vector>

/***
* Current design: 
* WorkState should have the virtual functions of all possible state transition operations (edges).
* These virtual functions are to be inhereted by the subclasses of WorkState. If they are not 
* inhereted, the operation or transition is not possible from that state.
*
* NO LONGER USING THIS :
* Old design: 
* nested siwch-case.
***/

WorkState::WorkState(std::vector<WorkState>& v) : states_(v)
{
    Deactivate();
}

bool WorkState::CheckActivated(){return activated_;}
void WorkState::Activate(){activated_=true;}
void WorkState::Deactivate(){activated_=false;}
void WorkState::SetStateNum(int i){state_num_=i;}
int WorkState::GetStateNum(){return state_num_;}
bool WorkState::CheckIfUniqueActivation()
{
    int s = 0;
    for(int i=0; i<states_.size(); i++)
    {
        if(states_[i].CheckActivated()) s++;
    }
    return s<=1;
}

void WorkState::FiducialsPlanned(){TransitionNotPossible();}
void WorkState::FiducialsDigitized(){TransitionNotPossible();}
void WorkState::ToolPosePlanned(){TransitionNotPossible();}
void WorkState::Registered(){TransitionNotPossible();}

void WorkState::ClearFiducials(){TransitionNotPossible();}
void WorkState::ClearDigitization(){TransitionNotPossible();}
void WorkState::ClearRegistration(){TransitionNotPossible();}
void WorkState::ClearToolPosePlan(){TransitionNotPossible();}

void WorkState::RePlanFiducials(){TransitionNotPossible();}
void WorkState::ReDigitize(){TransitionNotPossible();}
void WorkState::RePlanToolPose(){TransitionNotPossible();}

void WorkState::TransitionNotPossible(){TransitionNotPossible();}

