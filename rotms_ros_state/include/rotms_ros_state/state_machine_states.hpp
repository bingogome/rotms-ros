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
#include "flag_machine.hpp"
#include "state_machine.hpp"
#include "rotms_operations.hpp"

#include <vector>

bool CheckFlagIntegrity(const std::vector<WorkState*>& states);

std::vector<WorkState*> GetStatesVector(
    FlagMachine& f, TMSOperations& ops);

class State0000 : public WorkState
{
public:
    State0000(std::vector<WorkState*>& v, FlagMachine& f, TMSOperations& ops);

    int LandmarksPlanned() override;
    int ToolPosePlanned() override;
    int ReinitState() override;
    int UsePrevRegister() override;
};

class State1000 : public WorkState
{
public:
    State1000(std::vector<WorkState*>& v, FlagMachine& f, TMSOperations& ops);

    int LandmarksDigitized() override;
    int ToolPosePlanned() override;
    int ClearLandmarks() override;
    int LandmarksPlanned() override;
    int ReinitState() override;
    int UsePrevRegister() override;
};

class State1100 : public WorkState
{
public:
    State1100(std::vector<WorkState*>& v, FlagMachine& f, TMSOperations& ops);

    int ToolPosePlanned() override;
    int Registered() override;
    int ClearDigitization() override;
    int ClearLandmarks() override;
    int LandmarksDigitized() override;
    int ReinitState() override;
    int UsePrevRegister() override;
};

class State1101 : public WorkState
{
public:
    State1101(std::vector<WorkState*>& v, FlagMachine& f, TMSOperations& ops);

    int ToolPosePlanned() override;
    int ClearRegistration() override;
    int ClearLandmarks() override;
    int ReinitState() override;
    int UsePrevRegister() override;
};

class State0010 : public WorkState
{
public:
    State0010(std::vector<WorkState*>& v, FlagMachine& f, TMSOperations& ops);

    int LandmarksPlanned() override;
    int ClearToolPosePlan() override;
    int ToolPosePlanned() override;
    int ReinitState() override;
    int UsePrevRegister() override;
};

class State1010 : public WorkState
{
public:
    State1010(std::vector<WorkState*>& v, FlagMachine& f, TMSOperations& ops);

    int ClearToolPosePlan() override;
    int ClearLandmarks() override;
    int ToolPosePlanned() override;
    int LandmarksPlanned() override;
    int LandmarksDigitized() override;
    int ReinitState() override;
    int UsePrevRegister() override;
};

class State1110 : public WorkState
{
public:
    State1110(std::vector<WorkState*>& v, FlagMachine& f, TMSOperations& ops);

    int ClearToolPosePlan() override;
    int LandmarksDigitized() override;
    int ToolPosePlanned() override;
    int ClearDigitization() override;
    int ClearLandmarks() override;
    int Registered() override;
    int ReinitState() override;
    int UsePrevRegister() override;
};

class State1111 : public WorkState
{
public:
    State1111(std::vector<WorkState*>& v, FlagMachine& f, TMSOperations& ops);

    int ClearToolPosePlan() override;
    int ToolPosePlanned() override;
    int ClearRegistration() override;
    int ClearLandmarks() override;
    int ReinitState() override;
    int UsePrevRegister() override;
};