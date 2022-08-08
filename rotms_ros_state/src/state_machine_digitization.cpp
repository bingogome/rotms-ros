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


#include "state_machine.hpp"
#include "state_machine_digitization.hpp"
#include "flag_machine_digitization.hpp"
#include "operations_digitization.hpp"

#include <vector>
#include <functional>

StateDigitization::StateDigitization(
    int state_num, 
    std::vector<StateDigitization*>& v,
    std::vector<StateRegistration*>& states_upper_registration,
    FlagMachineDigitization& f, 
    OperationsDigitization& ops
    ) 
    : StateBase(state_num), states_(v), states_upper_registration_(states_upper_registration), flags_(f), ops_(ops)
{}

StateDigitization::~StateDigitization()
{
    for (auto p : states_)
    {
        delete p;
    } 
}

int StateDigitization::RedigitizeOneLandmark() { TransitionNotPossible(); return -1; }
int StateDigitization::ReinitState() { TransitionNotPossible(); return -1; }
int StateDigitization::UsePrevDigAndRedigOneLandmark() { TransitionNotPossible(); return -1; }
int StateDigitization::ConfirmAllDigitized() { TransitionNotPossible(); return -1; }
int StateDigitization::DigitizeAllLandmarks() { TransitionNotPossible(); return -1; }

bool StateDigitization::CheckIfUniqueActivation(const std::vector<StateDigitization*>& states)
{
    int s = 0;
    for(int i=0; i<states.size(); i++)
    {
        if(states[i]->CheckActivated()) s++;
    }
    return s<=1;
}

int StateDigitization::GetActivatedState(const std::vector<StateDigitization*>& states)
{
    if ( ! StateDigitization::CheckIfUniqueActivation(states) )
        throw std::runtime_error(
            "State machine error: not unique states are activated!");
    
    for(int i=0;i<states.size();i++)
    {
        if (states[i]->CheckActivated())
            return i;
    }

    return -1;
}

void StateDigitization::Transition(int target_state, TransitionOps funcs)
{
    Deactivate();

    for(int i=0; i<funcs.size(); i++)
    {
        funcs[i]();
    }

    states_[target_state]->Activate();
    if ( ! StateDigitization::CheckIfUniqueActivation(states_) )
        throw std::runtime_error(
            "State machine error: not unique states are activated!");
}