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

#pragma once

#include "state_machine.hpp"
#include "flag_machine_robot.hpp"

class StateRobot : public StateBase
{

public:

    StateRobot(
        int state_num,
        std::vector<StateRobot*>& v,
        FlagMachineRobot& f,
        OperationsRobot& ops);
    virtual ~StateRobot();

    FlagMachineRobot& flags_;

    static bool CheckIfUniqueActivation(const std::vector<StateRobot*>& states);
    static int GetActivatedState(const std::vector<StateRobot*>& states);

protected:

    OperationsRobot& ops_;
    const std::vector<StateRobot*>& states_;
    void Transition(int target_state, TransitionOps funcs);

};