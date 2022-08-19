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
#include "state_machine_toolplan.hpp"
#include "flag_machine_toolplan.hpp"
#include "operations_toolplan.hpp"

#include <vector>
#include <functional>

StateToolplan::StateToolplan(int state_num, std::vector<StateToolplan*>& v, FlagMachineToolplan& f, OperationsToolplan& ops) 
    : StateBase(state_num), states_(v), flags_(f), ops_(ops)
{}

StateToolplan::~StateToolplan()
{
    for (auto p : states_)
    {
        delete p;
    } 
}

int StateToolplan::ToolPosePlanned() { TransitionNotPossible(); return -1; }

int StateToolplan::ClearToolPosePlan() { TransitionNotPossible(); return -1; }

int StateToolplan::ReinitState() { TransitionNotPossible(); return -1; }

bool StateToolplan::CheckIfUniqueActivation(const std::vector<StateToolplan*>& states)
{
    int s = 0;
    for(int i=0; i<states.size(); i++)
    {
        if(states[i]->CheckActivated()) s++;
    }
    return s<=1;
}

int StateToolplan::GetActivatedState(const std::vector<StateToolplan*>& states)
{
    if ( ! StateToolplan::CheckIfUniqueActivation(states) )
        throw std::runtime_error(
            "State machine error: not unique states are activated!");
    
    for(int i=0;i<states.size();i++)
    {
        if (states[i]->CheckActivated())
            return i;
    }

    return -1;
}

void StateToolplan::Transition(int target_state, TransitionOps funcs)
{
    Deactivate();

    for(int i=0; i<funcs.size(); i++)
    {
        funcs[i]();
    }

    states_[target_state]->Activate();
    if ( ! StateToolplan::CheckIfUniqueActivation(states_) )
        throw std::runtime_error(
            "State machine error: not unique states are activated!");
}
