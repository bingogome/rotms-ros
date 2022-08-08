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

#include "state_machine_digitization_states.hpp"

bool CheckFlagIntegrityDigitization(const std::vector<StateDigitization*>& states)
{
    int num_flag = 1;
    std::vector<int> masks {0B1};
    std::vector<std::function<bool()>> flags {
        FlagMachineDigitization::GetAllDigitized
    };
    for(int i=0;i<states.size();i++)
    {
        if (states[i]->CheckActivated())
        {
            int state_num = states[i]->GetStateNum();
            for (int idx=0;idx<num_flag;idx++)
            {
                bool temp = (bool)((state_num & masks[idx]) >> (num_flag-idx-1));
                if (!(flags[idx]()==temp)) // detected inconsistence
                    return false;
            }
        }
    }
    return true;
}

std::vector<StateDigitization*> GetStatesVectorDigitization(
    FlagMachineDigitization& f, 
    OperationsDigitization& ops, 
    const std::vector<StateRegistration*>& states_upper_registration)
{   // ALWAYS CLEAN THE MEMORY AFTER FINISHED USING THE RETURNED VECTOR!!!

    int num_flag = 1;

    std::vector<StateDigitization*> vec;
    
    for(int i=0; i<std::pow(2.0,num_flag); i++)
    {
        switch (i)
        {
            case 0B0:
            {
                vec.push_back(new StateDigitization0(vec,states_upper_registration,f,ops));
                break;
            }
                
            case 0B1:
            {
                vec.push_back(new StateDigitization1(vec,states_upper_registration,f,ops));
                break;
            }
                
            default:
            {
                vec.push_back(new StateDigitization(-1,vec,states_upper_registration,f,ops));
                break;
            }
                
        }
    }

    // check if state matches index
    for(int i=0; i<std::pow(2.0,num_flag); i++)
    {
        if (i!=vec[i]->GetStateNum() && vec[i]->GetStateNum()!=-1) 
            throw std::runtime_error(
                "Wrong state number indexed!: \n" 
                + std::to_string(i) + "\n" 
                + "state num: " + std::to_string(vec[i]->GetStateNum()));
    }

    return vec;
}

// Initial state (default state)
StateDigitization0::StateDigitization0(
    std::vector<StateDigitization*>& v, 
    const std::vector<StateRegistration*>& states_upper_registration,
    FlagMachineDigitization& f, 
    OperationsDigitization& ops
    ) 
    : StateDigitization(0B0, v, states_upper_registration, f, ops) {Activate();} // default state

int StateDigitization0::RedigitizeOneLandmark(int idx)
{
    ops_.SetTempDigitizationIdx(idx);
    flags_.SetTempDigitizationIdx(idx);

    TransitionOps funcs;
    funcs.push_back(FlagMachineDigitization::ResetDigFlagArrAt);
    funcs.push_back(FlagMachineDigitization::UnconfirmAllDigitized);
    funcs.push_back(std::bind(&OperationsDigitization::OperationDigitizeOne, ops_));
    funcs.push_back(FlagMachineDigitization::SetDigFlagArrAt);
    funcs.push_back(FlagMachineDigitization::CheckAndUpdateFlag);
    Transition(0B0, funcs);

    ops_.ClearTempDigitizationIdx();
    flags_.ClearTempDigitizationIdx();

    if(flags_.GetAllDigitized()) 
        return ConfirmAllDigitized();
    else
        return 0B0;
}

int StateDigitization0::ReinitState()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineDigitization::InitializeDigFlagArr);
    funcs.push_back(FlagMachineDigitization::ClearTempDigitizationIdx);
    Transition(0B0, funcs);
    return 0B0;
}

int StateDigitization0::UsePrevDigAndRedigOneLandmark(int idx)
{
    ops_.SetTempDigitizationIdx(idx);
    flags_.SetTempDigitizationIdx(idx);

    TransitionOps funcs;
    funcs.push_back(FlagMachineDigitization::ResetDigFlagArrAt);
    funcs.push_back(FlagMachineDigitization::UnconfirmAllDigitized);
    funcs.push_back(std::bind(&OperationsDigitization::OperationDigitizeOne, ops_));
    funcs.push_back(FlagMachineDigitization::SetDigFlagArrAll);
    funcs.push_back(FlagMachineDigitization::CheckAndUpdateFlag);

    // Stay at current state first, wait until emit signal to upper state machine.
    Transition(0B0, funcs);

    ops_.ClearTempDigitizationIdx();
    flags_.ClearTempDigitizationIdx();

    return ConfirmAllDigitized();

}

int StateDigitization0::ConfirmAllDigitized()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineDigitization::SetDigFlagArrAll);
    funcs.push_back(FlagMachineDigitization::CheckAndUpdateFlag);
    
    // Stay at current state first, wait until emit signal to upper state machine.
    Transition(0B0, funcs);

    // Emit signal to upper state machine
    int activated_state_registration = StateRegistration::GetActivatedState(states_upper_registration_);
    activated_state_registration = states_upper_registration_[activated_state_registration]->LandmarksDigitized();
    if (activated_state_registration!=-1)
    {
        funcs.clear();
        Transition(0B1, funcs);
        return 0B1;
    }
    else
        return -1;
}

int StateDigitization0::DigitizeAllLandmarks()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineDigitization::ClearDigFlagArr);
    funcs.push_back(FlagMachineDigitization::UnconfirmAllDigitized);
    funcs.push_back(std::bind(&OperationsDigitization::OperationDigitizationAll, ops_));
    funcs.push_back(FlagMachineDigitization::SetDigFlagArrAll);
    funcs.push_back(FlagMachineDigitization::CheckAndUpdateFlag);
    // Stay at current state first, wait until emit signal to upper state machine.
    Transition(0B0, funcs);
    
    // Emit signal to upper state machine
    int activated_state_registration = StateRegistration::GetActivatedState(states_upper_registration_);
    activated_state_registration = states_upper_registration_[activated_state_registration]->LandmarksDigitized();
    if (activated_state_registration!=-1)
    {
        funcs.clear();
        Transition(0B1, funcs);
        return 0B1;
    }
    else
        return -1;
}

//
StateDigitization1::StateDigitization1(
    std::vector<StateDigitization*>& v, 
    const std::vector<StateRegistration*>& states_upper_registration,
    FlagMachineDigitization& f, 
    OperationsDigitization& ops
    ) 
    : StateDigitization(0B1, v, states_upper_registration, f, ops) {} 

int StateDigitization1::RedigitizeOneLandmark(int idx)
{
    ops_.SetTempDigitizationIdx(idx);
    flags_.SetTempDigitizationIdx(idx);

    TransitionOps funcs;
    funcs.push_back(FlagMachineDigitization::ResetDigFlagArrAt);
    funcs.push_back(FlagMachineDigitization::UnconfirmAllDigitized);
    funcs.push_back(std::bind(&OperationsDigitization::OperationDigitizeOne, ops_));
    funcs.push_back(FlagMachineDigitization::SetDigFlagArrAt);
    funcs.push_back(FlagMachineDigitization::CheckAndUpdateFlag);
    Transition(0B1, funcs);

    ops_.ClearTempDigitizationIdx();
    flags_.ClearTempDigitizationIdx();

    return 0B1;
}

int StateDigitization1::UsePrevDigAndRedigOneLandmark(int idx)
{
    ops_.SetTempDigitizationIdx(idx);
    flags_.SetTempDigitizationIdx(idx);

    TransitionOps funcs;
    funcs.push_back(FlagMachineDigitization::ResetDigFlagArrAt);
    funcs.push_back(FlagMachineDigitization::UnconfirmAllDigitized);
    funcs.push_back(std::bind(&OperationsDigitization::OperationDigitizeOne, ops_));
    funcs.push_back(FlagMachineDigitization::SetDigFlagArrAll);
    funcs.push_back(FlagMachineDigitization::CheckAndUpdateFlag);
    Transition(0B1, funcs);

    ops_.ClearTempDigitizationIdx();
    flags_.ClearTempDigitizationIdx();

    return 0B1;
}

int StateDigitization1::ReinitState()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineDigitization::InitializeDigFlagArr);
    funcs.push_back(FlagMachineDigitization::ClearTempDigitizationIdx);
    // Stay at current state first, wait until emit signal to upper state machine.
    Transition(0B1, funcs);
    
    // Emit signal to upper state machine
    int activated_state_registration = StateRegistration::GetActivatedState(states_upper_registration_);
    activated_state_registration = states_upper_registration_[activated_state_registration]->ClearDigitization();
    if (activated_state_registration!=-1)
    {
        funcs.clear();
        Transition(0B0, funcs);
        return 0B0;
    }
    else
        return -1;
}