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
#include "rotms_operations.hpp"
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

WorkState::WorkState(int state_num, std::vector<WorkState*>& v, FlagMachine& f, TMSOperations& ops) 
    : state_num_(state_num), states_(v), flags_(f), ops_(ops)
{
    Deactivate();
}

WorkState::~WorkState()
{
    for (auto p : states_)
    {
        delete p;
    } 
}

bool WorkState::CheckActivated(){return activated_;}
void WorkState::Activate(){activated_=true;}
void WorkState::Deactivate(){activated_=false;}
int WorkState::GetStateNum(){return state_num_;}
bool WorkState::CheckIfUniqueActivation(const std::vector<WorkState*>& states)
{
    int s = 0;
    for(int i=0; i<states.size(); i++)
    {
        if(states[i]->CheckActivated()) s++;
    }
    return s<=1;
}

int WorkState::GetActivatedState(const std::vector<WorkState*>& states)
{
    if ( ! WorkState::CheckIfUniqueActivation(states) )
        throw std::runtime_error(
            "State machine error: not unique states are activated!");
    
    for(int i=0;i<states.size();i++)
    {
        if (states[i]->CheckActivated())
            return i;
    }

    return -1;
}


int WorkState::LandmarksPlanned() { TransitionNotPossible(); return -1; }
int WorkState::LandmarksDigitized() { TransitionNotPossible(); return -1; }
int WorkState::ToolPosePlanned() { TransitionNotPossible(); return -1; }
int WorkState::Registered() { TransitionNotPossible(); return -1; }

int WorkState::ClearLandmarks() { TransitionNotPossible(); return -1; }
int WorkState::ClearDigitization() { TransitionNotPossible(); return -1; }
int WorkState::ClearRegistration() { TransitionNotPossible(); return -1; }
int WorkState::ClearToolPosePlan() { TransitionNotPossible(); return -1; }

int WorkState::ReinitState() { TransitionNotPossible(); return -1; }

void WorkState::TransitionNotPossible()
{
    // TODO: implement this. Optional
}
void WorkState::Transition(int target_state, TransitionOps funcs)
{
    Deactivate();

    for(int i=0; i<funcs.size(); i++)
    {
        funcs[i]();
    }

    states_[target_state]->Activate();
    if ( ! WorkState::CheckIfUniqueActivation(states_) )
        throw std::runtime_error(
            "State machine error: not unique states are activated!");
}
