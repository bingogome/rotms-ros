#include "flag_machine.hpp"
#include "state_machine.hpp"
#include "state_machine_states.hpp"
#include <vector>
#include <stdexcept>

const std::vector<WorkState> GetStatesVector(FlagMachine& f)
{
    std::vector<WorkState> vec;
    for(int i=0; i<16; i++)
    {
        switch (i)
        {
            case 0B0000:
            {
                WorkState s0B0000 = State0000(vec,f);
                vec.push_back(s0B0000);
                break;
            }
                
            case 0B1000:
            {
                WorkState s0B1000 = State1000(vec,f);
                vec.push_back(s0B1000);
                break;
            }
                
            case 0B1100:
            {
                WorkState s0B1100 = State1100(vec,f);
                vec.push_back(s0B1100);
                break;
            }
                
            case 0B1101:
            {
                WorkState s0B1101 = State1101(vec,f);
                vec.push_back(s0B1101);
                break;
            }
                
            case 0B0010:
            {
                WorkState s0B0010 = State0010(vec,f);
                vec.push_back(s0B0010);
                break;
            }
                
            case 0B1010:
            {
                WorkState s0B1010 = State1010(vec,f);
                vec.push_back(s0B1010);
                break;
            }
                
            case 0B1110:
            {
                WorkState s0B1110 = State1110(vec,f);
                vec.push_back(s0B1110);
                break;
            }
                
            case 0B1111:
            {
                WorkState s0B1111 = State1111(vec,f);
                vec.push_back(s0B1111);
                break;
            }
                
            default:
            {
                WorkState s = WorkState(-1,vec,f);
                vec.push_back(s);
                break;
            }
                
        }
    }

    // check if state matches index
    for(int i=0; i<16; i++)
    {
        if (i!=vec[i].GetStateNum()) 
            throw std::runtime_error("Wrong state number indexed!");
    }

    const std::vector<WorkState> vec_ = vec;
    return vec_;
}


State0000::State0000(std::vector<WorkState>& v, FlagMachine& f) 
    : WorkState(0B0000, v, f) {Activate();} // default state

void State0000::FiducialsPlanned()
{
    this->Transition(0B1000, FlagMachine::PlanLandmarks);
}
void State0000::ToolPosePlanned()
{
    this->Transition(0B0010, FlagMachine::PlanPlacePose);
}

State1000::State1000(std::vector<WorkState>& v, FlagMachine& f) 
    : WorkState(0B1000, v, f) {}

void State1000::FiducialsDigitized()
{}
void State1000::ToolPosePlanned()
{}
void State1000::ClearFiducials()
{}
void State1000::RePlanFiducials()
{}

State1100::State1100(std::vector<WorkState>& v, FlagMachine& f)  
    : WorkState(0B1100, v, f) {}

void State1100::ToolPosePlanned()
{}
void State1100::Registered()
{}
void State1100::ClearDigitization()
{}
void State1100::ClearFiducials()
{}
void State1100::ReDigitize()
{}

State1101::State1101(std::vector<WorkState>& v, FlagMachine& f)  
    : WorkState(0B1101, v, f) {}

void State1101::ToolPosePlanned()
{}
void State1101::ClearRegistration()
{}
void State1101::ClearFiducials()
{}

State0010::State0010(std::vector<WorkState>& v, FlagMachine& f)  
    : WorkState(0B0010, v, f) {}

void State0010::FiducialsPlanned()
{}
void State0010::ClearToolPosePlan()
{}
void State0010::RePlanToolPose()
{}

State1010::State1010(std::vector<WorkState>& v, FlagMachine& f)  
    : WorkState(0B1010, v, f) {}

void State1010::ClearToolPosePlan()
{}
void State1010::ClearFiducials()
{}
void State1010::RePlanToolPose()
{}
void State1010::RePlanFiducials()
{}
void State1010::FiducialsDigitized()
{}

State1110::State1110(std::vector<WorkState>& v, FlagMachine& f)  
    : WorkState(0B1110, v, f) {}

void State1110::ClearToolPosePlan()
{}
void State1110::ReDigitize()
{}
void State1110::RePlanToolPose()
{}
void State1110::ClearDigitization()
{}
void State1110::ClearFiducials()
{}
void State1110::Registered()
{}

State1111::State1111(std::vector<WorkState>& v, FlagMachine& f)  
    : WorkState(0B1111, v, f) {}

void State1111::ClearToolPosePlan()
{}
void State1111::RePlanToolPose()
{}
void State1111::ClearRegistration()
{}
void State1111::ClearFiducials()
{}