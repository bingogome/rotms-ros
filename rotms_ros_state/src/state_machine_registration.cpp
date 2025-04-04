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

/***
* Current design: 
* StateRegistration should have the virtual functions of all possible state transition operations (edges).
* These virtual functions are to be inhereted by the subclasses of StateRegistration. If they are not 
* inhereted, the operation or transition is not possible from that state.
*
* NO LONGER USING THIS :
* Old design: 
* nested siwch-case.
***/

#include "state_machine_registration.hpp"
#include "state_machine.hpp"
#include "flag_machine_registration.hpp"
#include "operations_registration.hpp"

#include <vector>
#include <functional>


StateRegistration::StateRegistration(int state_num, std::vector<StateRegistration*>& v, FlagMachineRegistration& f, OperationsRegistration& ops) 
    : StateBase(state_num), states_(v), flags_(f), ops_(ops)
{}

StateRegistration::~StateRegistration()
{
    for (auto p : states_)
    {
        delete p;
    } 
}

int StateRegistration::LandmarksPlanned() { TransitionNotPossible(); return -1; }
int StateRegistration::LandmarksDigitized() { TransitionNotPossible(); return -1; }
int StateRegistration::Registered() { TransitionNotPossible(); return -1; }

int StateRegistration::ClearLandmarks() { TransitionNotPossible(); return -1; }
int StateRegistration::ClearDigitization() { TransitionNotPossible(); return -1; }
int StateRegistration::ClearRegistration() { TransitionNotPossible(); return -1; }

int StateRegistration::ReinitState() { TransitionNotPossible(); return -1; }
int StateRegistration::UsePrevRegister() { TransitionNotPossible(); return -1; }

bool StateRegistration::CheckIfUniqueActivation(const std::vector<StateRegistration*>& states)
{
    int s = 0;
    for(int i=0; i<states.size(); i++)
    {
        if(states[i]->CheckActivated()) s++;
    }
    return s<=1;
}

int StateRegistration::GetActivatedState(const std::vector<StateRegistration*>& states)
{
    if ( ! StateRegistration::CheckIfUniqueActivation(states) )
        throw std::runtime_error(
            "State machine error: not unique states are activated!");
    
    for(int i=0;i<states.size();i++)
    {
        if (states[i]->CheckActivated())
            return i;
    }

    return -1;
}


void StateRegistration::Transition(int target_state, TransitionOps funcs)
{
    Deactivate();

    for(int i=0; i<funcs.size(); i++)
    {
        funcs[i]();
    }

    states_[target_state]->Activate();
    if ( ! StateRegistration::CheckIfUniqueActivation(states_) )
        throw std::runtime_error(
            "State machine error: not unique states are activated!");
}
