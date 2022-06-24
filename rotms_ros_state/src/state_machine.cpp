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


void WorkState::LandmarksPlanned(){TransitionNotPossible();}
void WorkState::LandmarksDigitized(){TransitionNotPossible();}
void WorkState::ToolPosePlanned(){TransitionNotPossible();}
void WorkState::Registered(){TransitionNotPossible();}

void WorkState::ClearLandmarks(){TransitionNotPossible();}
void WorkState::ClearDigitization(){TransitionNotPossible();}
void WorkState::ClearRegistration(){TransitionNotPossible();}
void WorkState::ClearToolPosePlan(){TransitionNotPossible();}

void WorkState::RePlanLandmarks(){TransitionNotPossible();}
void WorkState::ReDigitize(){TransitionNotPossible();}
void WorkState::RePlanToolPose(){TransitionNotPossible();}

void WorkState::TransitionNotPossible()
{
    // TODO: implement this. (possibly having a return value)
}
void WorkState::Transition(int target_state, TransitionOps funcs)
{
    this->Deactivate();

    for(int i=0; i<funcs.size(); i++)
    {
        funcs[i]();
    }

    states_[target_state].Activate();

    if ( ! WorkState::CheckIfUniqueActivation(states_) )
        throw std::runtime_error(
            "State machine error: not unique states are activated!");
}
