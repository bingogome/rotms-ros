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

StateBase::StateBase(int state_num, std::vector<StateTMS*>& v, FlagMachineTMS& f, OperationsTMS& ops) 
    : state_num_(state_num), states_(v), flags_(f), ops_(ops)
{
    Deactivate();
}

StateBase::~StateBase()
{
    for (auto p : states_)
    {
        delete p;
    } 
}

bool StateBase::CheckActivated(){return activated_;}
void StateBase::Activate(){activated_=true;}
void StateBase::Deactivate(){activated_=false;}
int StateBase::GetStateNum(){return state_num_;}

bool StateBase::CheckIfUniqueActivation(const std::vector<StateBase*>& states)
{
    int s = 0;
    for(int i=0; i<states.size(); i++)
    {
        if(states[i]->CheckActivated()) s++;
    }
    return s<=1;
}

int StateBase::GetActivatedState(const std::vector<StateBase*>& states)
{
    if ( ! StateBase::CheckIfUniqueActivation(states) )
        throw std::runtime_error(
            "State machine error: not unique states are activated!");
    
    for(int i=0;i<states.size();i++)
    {
        if (states[i]->CheckActivated())
            return i;
    }

    return -1;
}

void StateBase::TransitionNotPossible()
{
    // TODO: implement this. Optional
}

void StateBase::Transition(int target_state, TransitionOps funcs)
{
    Deactivate();

    for(int i=0; i<funcs.size(); i++)
    {
        funcs[i]();
    }

    states_[target_state]->Activate();
    if ( ! StateBase::CheckIfUniqueActivation(states_) )
        throw std::runtime_error(
            "State machine error: not unique states are activated!");
}


StateTMS::StateTMS(int state_num, std::vector<StateTMS*>& v, FlagMachineTMS& f, OperationsTMS& ops) 
    : StateBase(state_num,v,f,ops)
{}

StateTMS::~StateTMS()
{}

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

