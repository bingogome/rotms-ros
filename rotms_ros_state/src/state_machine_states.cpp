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
#include "state_machine_states.hpp"
#include <vector>
#include <stdexcept>
#include <functional>

bool CheckFlagIntegrity(std::vector<WorkState>& states)
{
    std::vector<int> musks {0B1000, 0B0100, 0B0010, 0B0001};
    std::vector<std::function<bool()>> flags {
        FlagMachine::GetFlagLandmarkPlanned,
        FlagMachine::GetFlagLandmarkDigitized,
        FlagMachine::GetFlagToolPosePlanned,
        FlagMachine::GetFlagRegistered
    };
    for(int i=0;i<states.size();i++)
    {
        if (states[i].CheckActivated())
        {
            int state_num = states[i].GetStateNum();
            for (int idx=0;idx<4;idx++)
            {
                bool temp = (bool)((state_num & musks[idx]) >> (4-idx-1));
                if (!(flags[idx]()==temp)) // detected inconsistence
                    return false;
            }
        }
    }
    return true;
}

const std::vector<WorkState> GetStatesVector(FlagMachine& f, TMSOperations& ops)
{
    std::vector<WorkState> vec;
    for(int i=0; i<16; i++)
    {
        switch (i)
        {
            case 0B0000:
            {
                WorkState s0B0000 = State0000(vec,f,ops);
                vec.push_back(s0B0000);
                break;
            }
                
            case 0B1000:
            {
                WorkState s0B1000 = State1000(vec,f,ops);
                vec.push_back(s0B1000);
                break;
            }
                
            case 0B1100:
            {
                WorkState s0B1100 = State1100(vec,f,ops);
                vec.push_back(s0B1100);
                break;
            }
                
            case 0B1101:
            {
                WorkState s0B1101 = State1101(vec,f,ops);
                vec.push_back(s0B1101);
                break;
            }
                
            case 0B0010:
            {
                WorkState s0B0010 = State0010(vec,f,ops);
                vec.push_back(s0B0010);
                break;
            }
                
            case 0B1010:
            {
                WorkState s0B1010 = State1010(vec,f,ops);
                vec.push_back(s0B1010);
                break;
            }
                
            case 0B1110:
            {
                WorkState s0B1110 = State1110(vec,f,ops);
                vec.push_back(s0B1110);
                break;
            }
                
            case 0B1111:
            {
                WorkState s0B1111 = State1111(vec,f,ops);
                vec.push_back(s0B1111);
                break;
            }
                
            default:
            {
                WorkState s = WorkState(-1,vec,f,ops);
                vec.push_back(s);
                break;
            }
                
        }
    }

    // check if state matches index
    for(int i=0; i<16; i++)
    {
        if (i!=vec[i].GetStateNum() && vec[i].GetStateNum()!=-1) 
            throw std::runtime_error(
                "Wrong state number indexed!: \n" 
                + std::to_string(i) + "\n" 
                + "state num: " + std::to_string(vec[i].GetStateNum()));
    }

    const std::vector<WorkState> vec_ = vec;
    return vec_;
}

// Initial state (default state)
State0000::State0000(std::vector<WorkState>& v, FlagMachine& f, TMSOperations& ops) 
    : WorkState(0B0000, v, f, ops) {Activate();} // default state

int State0000::LandmarksPlanned()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::PlanLandmarks);
    this->Transition(0B1000, funcs);
    return 0B1000;
}
int State0000::ToolPosePlanned()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::PlanToolPose);
    this->Transition(0B0010, funcs);
    return 0B0010;
}

// 
State1000::State1000(std::vector<WorkState>& v, FlagMachine& f, TMSOperations& ops) 
    : WorkState(0B1000, v, f, ops) {}

int State1000::LandmarksDigitized()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::DigitizeLandmarks);
    this->Transition(0B1100, funcs);
    return 0B1100;
}
int State1000::ToolPosePlanned()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::PlanToolPose);
    this->Transition(0B1010, funcs);
    return 0B1010;
}
int State1000::ClearLandmarks()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::UnPlanLandmarks);
    this->Transition(0B0000, funcs);
    return 0B0000;
}
int State1000::RePlanLandmarks()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::PlanLandmarks);
    this->Transition(0B1000, funcs);
    return 0B1000;
}

//
State1100::State1100(std::vector<WorkState>& v, FlagMachine& f, TMSOperations& ops)  
    : WorkState(0B1100, v, f, ops) {}

int State1100::ToolPosePlanned()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::PlanToolPose);
    this->Transition(0B1110, funcs);
    return 0B1110;
}
int State1100::Registered()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&TMSOperations::OperationRegistration, ops_))
    funcs.push_back(FlagMachine::CompleteRegistration);
    this->Transition(0B1101, funcs);
    return 0B1101;
}
int State1100::ClearDigitization()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::UnDigitizeLandmarks);
    this->Transition(0B1000, funcs);
    return 0B1000;
}
int State1100::ClearLandmarks()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::UnDigitizeLandmarks);
    funcs.push_back(FlagMachine::UnPlanLandmarks);
    this->Transition(0B0000, funcs);
    return 0B0000;
}
int State1100::ReDigitize()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::DigitizeLandmarks);
    this->Transition(0B1100, funcs);
    return 0B1100;
}

//
State1101::State1101(std::vector<WorkState>& v, FlagMachine& f, TMSOperations& ops)  
    : WorkState(0B1101, v, f, ops) {}

int State1101::ToolPosePlanned()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::PlanToolPose);
    this->Transition(0B1111, funcs);
    return 0B1111;
}
int State1101::ClearRegistration()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::UnCompleteRegistration);
    this->Transition(0B1100, funcs);
    return 0B1100;
}
int State1101::ClearLandmarks()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::UnCompleteRegistration);
    funcs.push_back(FlagMachine::UnDigitizeLandmarks);
    funcs.push_back(FlagMachine::UnPlanLandmarks);
    this->Transition(0B0000, funcs);
    return 0B0000;
}

//
State0010::State0010(std::vector<WorkState>& v, FlagMachine& f, TMSOperations& ops)  
    : WorkState(0B0010, v, f, ops) {}

int State0010::LandmarksPlanned()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::PlanLandmarks);
    this->Transition(0B1010, funcs);
    return 0B1010;
}
int State0010::ClearToolPosePlan()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::UnPlanToolPose);
    this->Transition(0B0000, funcs);
    return 0B0000;
}
int State0010::RePlanToolPose()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::PlanToolPose);
    this->Transition(0B0010, funcs);
    return 0B0010;
}

//
State1010::State1010(std::vector<WorkState>& v, FlagMachine& f, TMSOperations& ops)  
    : WorkState(0B1010, v, f, ops) {}

int State1010::ClearToolPosePlan()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::UnPlanToolPose);
    this->Transition(0B1000, funcs);
    return 0B1000;
}
int State1010::ClearLandmarks()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::UnPlanLandmarks);
    this->Transition(0B0010, funcs);
    return 0B0010;
}
int State1010::RePlanToolPose()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::PlanToolPose);
    this->Transition(0B1010, funcs);
    return 0B1010;
}
int State1010::RePlanLandmarks()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::PlanLandmarks);
    this->Transition(0B1010, funcs);
    return 0B1010;
}
int State1010::LandmarksDigitized()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::DigitizeLandmarks);
    this->Transition(0B1110, funcs);
    return 0B1110;
}

//
State1110::State1110(std::vector<WorkState>& v, FlagMachine& f, TMSOperations& ops)  
    : WorkState(0B1110, v, f, ops) {}

int State1110::ClearToolPosePlan()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::UnPlanToolPose);
    this->Transition(0B1100, funcs);
    return 0B1100;
}
int State1110::ReDigitize()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::DigitizeLandmarks);
    this->Transition(0B1110, funcs);
    return 0B1110;
}
int State1110::RePlanToolPose()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::PlanToolPose);
    this->Transition(0B1110, funcs);
    return 0B1110;
}
int State1110::ClearDigitization()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::UnDigitizeLandmarks);
    this->Transition(0B1010, funcs);
    return 0B1010;
}
int State1110::ClearLandmarks()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::UnDigitizeLandmarks);
    funcs.push_back(FlagMachine::UnPlanLandmarks);
    this->Transition(0B0010, funcs);
    return 0B0010;
}
int State1110::Registered()
{
    TransitionOps funcs;
    funcs.push_back(std::bind(&TMSOperations::OperationRegistration, ops_))
    funcs.push_back(FlagMachine::CompleteRegistration);
    this->Transition(0B1111, funcs);
    return 0B1111;
}

//
State1111::State1111(std::vector<WorkState>& v, FlagMachine& f, TMSOperations& ops)  
    : WorkState(0B1111, v, f, ops) {}

int State1111::ClearToolPosePlan()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::UnPlanToolPose);
    this->Transition(0B1101, funcs);
    return 0B1101;
}
int State1111::RePlanToolPose()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::PlanToolPose);
    this->Transition(0B1111, funcs);
    return 0B1111;
}
int State1111::ClearRegistration()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::UnCompleteRegistration);
    this->Transition(0B1110, funcs);
    return 0B1110;
}
int State1111::ClearLandmarks()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachine::UnCompleteRegistration);
    funcs.push_back(FlagMachine::UnDigitizeLandmarks);
    funcs.push_back(FlagMachine::UnPlanLandmarks);
    this->Transition(0B0010, funcs);
    return 0B0010;
}