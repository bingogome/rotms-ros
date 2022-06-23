#include "flag_machine.hpp"
#include "state_machine.hpp"
#include <vector>
#include <functional>

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

WorkState::WorkState(int state_num, std::vector<WorkState>& v, FlagMachine& f) 
    : state_num_(state_num), states_(v), flags_(f)
{
    Deactivate();
}

bool WorkState::CheckActivated(){return activated_;}
void WorkState::Activate(){activated_=true;}
void WorkState::Deactivate(){activated_=false;}
int WorkState::GetStateNum(){return state_num_;}
bool WorkState::CheckIfUniqueActivation(std::vector<WorkState>& states)
{
    int s = 0;
    for(int i=0; i<states.size(); i++)
    {
        if(states[i].CheckActivated()) s++;
    }
    return s<=1;
}
WorkState& WorkState::GetActivatedState(std::vector<WorkState>& states)
{
    if ( ! WorkState::CheckIfUniqueActivation(states) )
        throw std::runtime_error(
            "State machine error: not unique states are activated!");
    
    for(int i=0;i<states.size();i++)
    {
        if (states[i].CheckActivated())
            return states[i];
    }

    FlagMachine f;
    FlagMachine& f_ = f;

    WorkState s(-1,states,f_);
    WorkState& s_ = s;

    return s_;
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

void WorkState::TransitionNotPossible()
{
    // TODO: implement this. (possibly having a return value)
}
void WorkState::Transition(int target_state, std::function<void()> func_flagchange)
{
    this->Deactivate();

    func_flagchange();

    states_[target_state].Activate();

    if ( ! WorkState::CheckIfUniqueActivation(states_) )
        throw std::runtime_error(
            "State machine error: not unique states are activated!");
}
