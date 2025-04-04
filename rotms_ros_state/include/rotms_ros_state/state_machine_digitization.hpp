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
#include "state_machine_registration.hpp"
#include "flag_machine_digitization.hpp"
#include "operations_digitization.hpp"

class StateDigitization : public StateBase
{

public:

    StateDigitization(
        int state_num,
        std::vector<StateDigitization*>& v,
        const std::vector<StateRegistration*>& states_upper_registration,
        FlagMachineDigitization& f,
        OperationsDigitization& ops);
    virtual ~StateDigitization();

    FlagMachineDigitization& flags_;

    virtual int RedigitizeOneLandmark(int idx);
    virtual int ReinitState();
    virtual int UsePrevDigAndRedigOneLandmark(int idx);
    virtual int ConfirmAllDigitized();
    virtual int DigitizeAllLandmarks();

    static bool CheckIfUniqueActivation(const std::vector<StateDigitization*>& states);
    static int GetActivatedState(const std::vector<StateDigitization*>& states);

protected:

    OperationsDigitization& ops_;
    const std::vector<StateDigitization*>& states_;
    const std::vector<StateRegistration*>& states_upper_registration_;
    void Transition(int target_state, TransitionOps funcs);

};