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
#include "flag_machine_toolplan.hpp"

//
FlagMachineToolplan::FlagMachineToolplan() : FlagMachineBase()
{
    flag_toolpose_planned_ = false;
}

// Crucial operations status flags
bool FlagMachineToolplan::flag_toolpose_planned_;

// Crucial operations status setters and getters
void FlagMachineToolplan::PlanToolPose(){flag_toolpose_planned_=true;}
void FlagMachineToolplan::UnPlanToolPose(){flag_toolpose_planned_=false;}
bool FlagMachineToolplan::GetFlagToolPosePlanned(){return flag_toolpose_planned_;}
