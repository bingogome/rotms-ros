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
* StateTMS should have the virtual functions of all possible state transition operations (edges).
* These virtual functions are to be inhereted by the subclasses of StateTMS. If they are not 
* inhereted, the operation or transition is not possible from that state.
*
* NO LONGER USING THIS :
* Old design: 
* nested siwch-case.
***/

StateTMS::StateTMS(int state_num, std::vector<StateTMS*>& v, FlagMachineTMS& f, OperationsTMS& ops) 
    : state_num_(state_num), states_(v), flags_(f), ops_(ops)
{
    Deactivate();
}

StateTMS::~StateTMS()
{
    for (auto p : states_)
    {
        delete p;
    } 
}

bool StateTMS::CheckActivated(){return activated_;}
void StateTMS::Activate(){activated_=true;}
void StateTMS::Deactivate(){activated_=false;}
int StateTMS::GetStateNum(){return state_num_;}
bool StateTMS::CheckIfUniqueActivation(const std::vector<StateTMS*>& states)
{
    int s = 0;
    for(int i=0; i<states.size(); i++)
    {
        if(states[i]->CheckActivated()) s++;
    }
    return s<=1;
}

int StateTMS::GetActivatedState(const std::vector<StateTMS*>& states)
{
    if ( ! StateTMS::CheckIfUniqueActivation(states) )
        throw std::runtime_error(
            "State machine error: not unique states are activated!");
    
    for(int i=0;i<states.size();i++)
    {
        if (states[i]->CheckActivated())
            return i;
    }

    return -1;
}


int StateTMS::LandmarksPlanned() { TransitionNotPossible(); return -1; }
int StateTMS::LandmarksDigitized() { TransitionNotPossible(); return -1; }
int StateTMS::ToolPosePlanned() { TransitionNotPossible(); return -1; }
int StateTMS::Registered() { TransitionNotPossible(); return -1; }

int StateTMS::ClearLandmarks() { TransitionNotPossible(); return -1; }
int StateTMS::ClearDigitization() { TransitionNotPossible(); return -1; }
int StateTMS::ClearRegistration() { TransitionNotPossible(); return -1; }
int StateTMS::ClearToolPosePlan() { TransitionNotPossible(); return -1; }

int StateTMS::ReinitState() { TransitionNotPossible(); return -1; }
int StateTMS::UsePrevRegister() { TransitionNotPossible(); return -1; }

void StateTMS::TransitionNotPossible()
{
    // TODO: implement this. Optional
}
void StateTMS::Transition(int target_state, TransitionOps funcs)
{
    Deactivate();

    for(int i=0; i<funcs.size(); i++)
    {
        funcs[i]();
    }

    states_[target_state]->Activate();
    if ( ! StateTMS::CheckIfUniqueActivation(states_) )
        throw std::runtime_error(
            "State machine error: not unique states are activated!");
}
