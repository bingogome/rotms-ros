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
#include "state_machine_tms.hpp"
#include "rotms_operations.hpp"

#include <vector>
#include <stdexcept>
#include <functional>

bool CheckFlagIntegrityTMS(const std::vector<StateTMS*>& states)
{
    std::vector<int> masks {0B100, 0B010, 0B001};
    std::vector<std::function<bool()>> flags {
        FlagMachineTMS::GetFlagLandmarkPlanned,
        FlagMachineTMS::GetFlagLandmarkDigitized,
        FlagMachineTMS::GetFlagRegistered
    };
    for(int i=0;i<states.size();i++)
    {
        if (states[i]->CheckActivated())
        {
            int state_num = states[i]->GetStateNum();
            for (int idx=0;idx<3;idx++)
            {
                bool temp = (bool)((state_num & masks[idx]) >> (3-idx-1));
                if (!(flags[idx]()==temp)) // detected inconsistence
                    return false;
            }
        }
    }
    return true;
}

std::vector<StateTMS*> GetStatesVectorTMS(FlagMachineTMS& f, OperationsTMS& ops)
{   // ALWAYS CLEAN THE MEMORY AFTER FINISHED USING THE RETURNED VECTOR!!!
    std::vector<StateTMS*> vec;
    for(int i=0; i<8; i++)
    {
        switch (i)
        {
            case 0B000:
            {
                vec.push_back(new State0000(vec,f,ops));
                break;
            }
                
            case 0B100:
            {
                vec.push_back(new State1000(vec,f,ops));
                break;
            }
                
            case 0B110:
            {
                vec.push_back(new State1100(vec,f,ops));
                break;
            }
                
            case 0B111:
            {
                vec.push_back(new State1101(vec,f,ops));
                break;
            }
                
            default:
            {
                vec.push_back(new StateTMS(-1,vec,f,ops));
                break;
            }
                
        }
    }

    // check if state matches index
    for(int i=0; i<8; i++)
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
State000::State000(std::vector<StateTMS*>& v, FlagMachineTMS& f, OperationsTMS& ops) 
    : StateTMS(0B000, v, f, ops) {Activate();} // default state

int State000::LandmarksPlanned()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineTMS::PlanLandmarks);
    Transition(0B100, funcs);
    return 0B100;
}
int State000::ReinitState()
{
    return 0B000;
}
int State000::UsePrevRegister()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsTMS::OperationUsePreRegistration, ops_));
    funcs.push_back(FlagMachineTMS::PlanLandmarks);
    funcs.push_back(FlagMachineTMS::DigitizeLandmarks);
    funcs.push_back(FlagMachineTMS::CompleteRegistration);
    Transition(0B111, funcs);
    return 0B111;
}

// 
State100::State100(std::vector<StateTMS*>& v, FlagMachineTMS& f, OperationsTMS& ops) 
    : StateTMS(0B100, v, f, ops) {}

int State100::LandmarksDigitized()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsTMS::OperationDigitization, ops_));
    funcs.push_back(FlagMachineTMS::DigitizeLandmarks);
    Transition(0B110, funcs);
    return 0B110;
}
int State100::ClearLandmarks()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineTMS::UnPlanLandmarks);
    Transition(0B000, funcs);
    return 0B000;
}
int State100::LandmarksPlanned()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineTMS::PlanLandmarks);
    Transition(0B100, funcs);
    return 0B100;
}
int State100::ReinitState()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineTMS::UnPlanLandmarks);
    Transition(0B000, funcs);
    return 0B000;
}
int State100::UsePrevRegister()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsTMS::OperationUsePreRegistration, ops_));
    funcs.push_back(FlagMachineTMS::DigitizeLandmarks);
    funcs.push_back(FlagMachineTMS::CompleteRegistration);
    Transition(0B111, funcs);
    return 0B111;
}

//
State110::State110(std::vector<StateTMS*>& v, FlagMachineTMS& f, OperationsTMS& ops)  
    : StateTMS(0B110, v, f, ops) {}

int State110::LandmarksPlanned()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineTMS::PlanLandmarks);
    Transition(0B100, funcs);
    return 0B100;
}
int State110::Registered()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsTMS::OperationRegistration, ops_));
    funcs.push_back(FlagMachineTMS::CompleteRegistration);
    Transition(0B111, funcs);
    return 0B111;
}
int State110::ClearDigitization()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineTMS::UnDigitizeLandmarks);
    Transition(0B100, funcs);
    return 0B100;
}
int State110::ClearLandmarks()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineTMS::UnDigitizeLandmarks);
    funcs.push_back(FlagMachineTMS::UnPlanLandmarks);
    Transition(0B000, funcs);
    return 0B000;
}
int State110::LandmarksDigitized()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsTMS::OperationDigitization, ops_));
    funcs.push_back(FlagMachineTMS::DigitizeLandmarks);
    Transition(0B110, funcs);
    return 0B110;
}
int State110::ReinitState()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineTMS::UnPlanLandmarks);
    funcs.push_back(FlagMachineTMS::UnDigitizeLandmarks);
    Transition(0B000, funcs);
    return 0B000;
}
int State110::UsePrevRegister()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsTMS::OperationUsePreRegistration, ops_));
    funcs.push_back(FlagMachineTMS::CompleteRegistration);
    Transition(0B111, funcs);
    return 0B111;
}

//
State111::State111(std::vector<StateTMS*>& v, FlagMachineTMS& f, OperationsTMS& ops)  
    : StateTMS(0B111, v, f, ops) {}

int State111::ClearRegistration()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsTMS::OperationResetRegistration, ops_));
    funcs.push_back(FlagMachineTMS::UnCompleteRegistration);
    Transition(0B110, funcs);
    return 0B110;
}
int State111::ClearLandmarks()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineTMS::UnCompleteRegistration);
    funcs.push_back(FlagMachineTMS::UnDigitizeLandmarks);
    funcs.push_back(FlagMachineTMS::UnPlanLandmarks);
    Transition(0B000, funcs);
    return 0B000;
}
int State111::ReinitState()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsTMS::OperationResetRegistration, ops_));
    funcs.push_back(FlagMachineTMS::UnPlanLandmarks);
    funcs.push_back(FlagMachineTMS::UnDigitizeLandmarks);
    funcs.push_back(FlagMachineTMS::UnCompleteRegistration);
    Transition(0B000, funcs);
    return 0B000;
}
int State111::UsePrevRegister()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&OperationsTMS::OperationUsePreRegistration, ops_));
    Transition(0B111, funcs);
    return 0B111;
}