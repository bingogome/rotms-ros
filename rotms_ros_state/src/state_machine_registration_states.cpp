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
#include "state_machine_registration_states.hpp"
#include "flag_machine_registration.hpp"
#include "state_machine_registration.hpp"
#include "operations_registration.hpp"

#include <vector>
#include <stdexcept>
#include <functional>

bool CheckFlagIntegrityRegistration(const std::vector<StateRegistration*>& states)
{
    int num_flag = 3;
    std::vector<int> masks {0B100, 0B010, 0B001};
    std::vector<std::function<bool()>> flags {
        FlagMachineRegistration::GetFlagLandmarkPlanned,
        FlagMachineRegistration::GetFlagLandmarkDigitized,
        FlagMachineRegistration::GetFlagRegistered
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

std::vector<StateRegistration*> GetStatesVectorRegistration(FlagMachineRegistration& f, OperationsRegistration& ops)
{   // ALWAYS CLEAN THE MEMORY AFTER FINISHED USING THE RETURNED VECTOR!!!
    
    int num_flag = 3;

    std::vector<StateRegistration*> vec;
    for(int i=0; i<std::pow(2.0,num_flag); i++)
    {
        switch (i)
        {
            case 0B000:
            {
                vec.push_back(new StateRegistration000(vec,f,ops));
                break;
            }
                
            case 0B100:
            {
                vec.push_back(new StateRegistration100(vec,f,ops));
                break;
            }
                
            case 0B110:
            {
                vec.push_back(new StateRegistration110(vec,f,ops));
                break;
            }
                
            case 0B111:
            {
                vec.push_back(new StateRegistration111(vec,f,ops));
                break;
            }
                
            default:
            {
                vec.push_back(new StateRegistration(-1,vec,f,ops));
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
StateRegistration000::StateRegistration000(std::vector<StateRegistration*>& v, FlagMachineRegistration& f, OperationsRegistration& ops) 
    : StateRegistration(0B000, v, f, ops) {Activate();} // default state

int StateRegistration000::LandmarksPlanned()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsRegistration::OperationPlanLandmarks, ops_));
    funcs.push_back(FlagMachineRegistration::PlanLandmarks);
    Transition(0B100, funcs);
    return 0B100;
}
int StateRegistration000::ReinitState()
{
    return 0B000;
}
int StateRegistration000::UsePrevRegister()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsRegistration::OperationUsePreRegistration, ops_));
    funcs.push_back(FlagMachineRegistration::PlanLandmarks);
    funcs.push_back(FlagMachineRegistration::DigitizeLandmarks);
    funcs.push_back(FlagMachineRegistration::CompleteRegistration);
    Transition(0B111, funcs);
    return 0B111;
}

// 
StateRegistration100::StateRegistration100(std::vector<StateRegistration*>& v, FlagMachineRegistration& f, OperationsRegistration& ops) 
    : StateRegistration(0B100, v, f, ops) {}

int StateRegistration100::LandmarksDigitized()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsRegistration::OperationDigitization, ops_));
    funcs.push_back(FlagMachineRegistration::DigitizeLandmarks);
    Transition(0B110, funcs);
    return 0B110;
}
int StateRegistration100::ClearLandmarks()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineRegistration::UnPlanLandmarks);
    Transition(0B000, funcs);
    return 0B000;
}
int StateRegistration100::LandmarksPlanned()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsRegistration::OperationPlanLandmarks, ops_));
    funcs.push_back(FlagMachineRegistration::PlanLandmarks);
    Transition(0B100, funcs);
    return 0B100;
}
int StateRegistration100::ReinitState()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineRegistration::UnPlanLandmarks);
    Transition(0B000, funcs);
    return 0B000;
}
int StateRegistration100::UsePrevRegister()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsRegistration::OperationUsePreRegistration, ops_));
    funcs.push_back(FlagMachineRegistration::DigitizeLandmarks);
    funcs.push_back(FlagMachineRegistration::CompleteRegistration);
    Transition(0B111, funcs);
    return 0B111;
}

//
StateRegistration110::StateRegistration110(std::vector<StateRegistration*>& v, FlagMachineRegistration& f, OperationsRegistration& ops)  
    : StateRegistration(0B110, v, f, ops) {}

int StateRegistration110::LandmarksPlanned()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsRegistration::OperationPlanLandmarks, ops_));
    funcs.push_back(FlagMachineRegistration::PlanLandmarks);
    Transition(0B100, funcs);
    return 0B100;
}
int StateRegistration110::Registered()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsRegistration::OperationRegistration, ops_));
    funcs.push_back(FlagMachineRegistration::CompleteRegistration);
    Transition(0B111, funcs);
    return 0B111;
}
int StateRegistration110::ClearDigitization()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineRegistration::UnDigitizeLandmarks);
    Transition(0B100, funcs);
    return 0B100;
}
int StateRegistration110::ClearLandmarks()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineRegistration::UnDigitizeLandmarks);
    funcs.push_back(FlagMachineRegistration::UnPlanLandmarks);
    Transition(0B000, funcs);
    return 0B000;
}
int StateRegistration110::LandmarksDigitized()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsRegistration::OperationDigitization, ops_));
    funcs.push_back(FlagMachineRegistration::DigitizeLandmarks);
    Transition(0B110, funcs);
    return 0B110;
}
int StateRegistration110::ReinitState()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineRegistration::UnPlanLandmarks);
    funcs.push_back(FlagMachineRegistration::UnDigitizeLandmarks);
    Transition(0B000, funcs);
    return 0B000;
}
int StateRegistration110::UsePrevRegister()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsRegistration::OperationUsePreRegistration, ops_));
    funcs.push_back(FlagMachineRegistration::CompleteRegistration);
    Transition(0B111, funcs);
    return 0B111;
}

//
StateRegistration111::StateRegistration111(std::vector<StateRegistration*>& v, FlagMachineRegistration& f, OperationsRegistration& ops)  
    : StateRegistration(0B111, v, f, ops) {}

int StateRegistration111::ClearRegistration()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsRegistration::OperationResetRegistration, ops_));
    funcs.push_back(FlagMachineRegistration::UnCompleteRegistration);
    Transition(0B110, funcs);
    return 0B110;
}
int StateRegistration111::ClearLandmarks()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineRegistration::UnCompleteRegistration);
    funcs.push_back(FlagMachineRegistration::UnDigitizeLandmarks);
    funcs.push_back(FlagMachineRegistration::UnPlanLandmarks);
    Transition(0B000, funcs);
    return 0B000;
}
int StateRegistration111::ReinitState()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsRegistration::OperationResetRegistration, ops_));
    funcs.push_back(FlagMachineRegistration::UnPlanLandmarks);
    funcs.push_back(FlagMachineRegistration::UnDigitizeLandmarks);
    funcs.push_back(FlagMachineRegistration::UnCompleteRegistration);
    Transition(0B000, funcs);
    return 0B000;
}
int StateRegistration111::UsePrevRegister()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsRegistration::OperationUsePreRegistration, ops_));
    Transition(0B111, funcs);
    return 0B111;
}