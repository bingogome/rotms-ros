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

#include "state_machine_digitization.hpp"
#include <vector>

bool CheckFlagIntegrityDigitization(const std::vector<StateDigitization*>& states);

std::vector<StateDigitization*> GetStatesVectorDigitization(
    FlagMachineDigitization& f, OperationsDigitization& ops);

class StateDigitization0 : public StateDigitization
{
public:
    StateDigitization0(std::vector<StateDigitization*>& v, FlagMachineDigitization& f, OperationsDigitization& ops);

    int RedigitizeOneLandmark(int idx) override;
    int ReinitState() override;
    int UsePrevDigAndRedigOneLandmark(int idx) override;
    int ConfirmAllDigitized() override;
    int DigitizeAllLandmarks() override;

};

class StateDigitization1 : public StateDigitization
{
public:
    StateDigitization1(std::vector<StateDigitization*>& v, FlagMachineDigitization& f, OperationsDigitization& ops);

    int RedigitizeOneLandmark(int idx) override;
    int UsePrevDigAndRedigOneLandmark(int idx) override;
    int ReinitState() override;

};
