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

#include "state_machine_robot_states.hpp"

bool CheckFlagIntegrityRobot(const std::vector<StateRobot*>& states)
{
    int num_flag = 1;
    std::vector<int> masks {0B1};
    std::vector<std::function<bool()>> flags {
        FlagMachineRobot::GetFlagRobotConnStatus
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

std::vector<StateRobot*> GetStatesVectorRobot(FlagMachineRobot& f, OperationsRobot& ops)
{   // ALWAYS CLEAN THE MEMORY AFTER FINISHED USING THE RETURNED VECTOR!!!

    int num_flag = 1;

    std::vector<StateRobot*> vec;
    
    for(int i=0; i<std::pow(2.0,num_flag); i++)
    {
        switch (i)
        {
            case 0B0:
            {
                vec.push_back(new StateRobot0(vec,f,ops));
                break;
            }
                
            case 0B1:
            {
                vec.push_back(new StateRobot1(vec,f,ops));
                break;
            }
                
            default:
            {
                vec.push_back(new StateRobot(-1,vec,f,ops));
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
StateRobot0::StateRobot0(std::vector<StateRobot*>& v, FlagMachineRobot& f, OperationsRobot& ops) 
    : StateRobot(0B0, v, f, ops) {Activate();} // default state

int StateRobot0::ConnectRobot()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineRobot::ConnectRobot);
    Transition(0B1, funcs);
    return 0B1;
}

//
StateRobot1::StateRobot1(std::vector<StateRobot*>& v, FlagMachineRobot& f, OperationsRobot& ops) 
    : StateRobot(0B1, v, f, ops) {} 

int StateRobot1::DisconnectRobot()
{
    TransitionOps funcs;
    funcs.push_back(FlagMachineRobot::DisconnectRobot);
    Transition(0B0, funcs);
    return 0B0;
}
