#include "state_machine.hpp"
#include "state_machine_states.hpp"
#include <vector>
#include <stdexcept>

std::vector<WorkState> GetStatesVector()
{
    std::vector<WorkState> vec;
    for(int i=0; i<16; i++)
    {
        switch (i)
        {
            case 0B0000:
            {
                WorkState s0B0000 = State0000(vec);
                vec.push_back(s0B0000);
                break;
            }
                
            case 0B1000:
            {
                WorkState s0B1000 = State1000(vec);
                vec.push_back(s0B1000);
                break;
            }
                
            case 0B1100:
            {
                WorkState s0B1100 = State1100(vec);
                vec.push_back(s0B1100);
                break;
            }
                
            case 0B1101:
            {
                WorkState s0B1101 = State1101(vec);
                vec.push_back(s0B1101);
                break;
            }
                
            case 0B0010:
            {
                WorkState s0B0010 = State0010(vec);
                vec.push_back(s0B0010);
                break;
            }
                
            case 0B1010:
            {
                WorkState s0B1010 = State1010(vec);
                vec.push_back(s0B1010);
                break;
            }
                
            case 0B1110:
            {
                WorkState s0B1110 = State1110(vec);
                vec.push_back(s0B1110);
                break;
            }
                
            case 0B1111:
            {
                WorkState s0B1111 = State1111(vec);
                vec.push_back(s0B1111);
                break;
            }
                
            default:
            {
                WorkState s = WorkState(vec);
                s.SetStateNum(i);
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

    return vec;
}


State0000::State0000(std::vector<WorkState>& v) : WorkState(v)
{
    state_num_ = 0B0000;
    Activate();
}
void State0000::FiducialsPlanned(){}
void State0000::ToolPosePlanned(){}

State1000::State1000(std::vector<WorkState>& v) : WorkState(v) 
{
    state_num_ = 0B1000;
}
void State1000::FiducialsDigitized(){}
void State1000::ToolPosePlanned(){}
void State1000::ClearFiducials(){}
void State1000::RePlanFiducials(){}

State1100::State1100(std::vector<WorkState>& v) : WorkState(v) 
{
    state_num_ = 0B1100;
}

State1101::State1101(std::vector<WorkState>& v) : WorkState(v) 
{
    state_num_ = 0B1101;
}

State0010::State0010(std::vector<WorkState>& v) : WorkState(v) 
{
    state_num_ = 0B0010;
}

State1010::State1010(std::vector<WorkState>& v) : WorkState(v) 
{
    state_num_ = 0B1010;
}

State1110::State1110(std::vector<WorkState>& v) : WorkState(v) 
{
    state_num_ = 0B1110;
}

State1111::State1111(std::vector<WorkState>& v) : WorkState(v) 
{
    state_num_ = 0B1111;
}