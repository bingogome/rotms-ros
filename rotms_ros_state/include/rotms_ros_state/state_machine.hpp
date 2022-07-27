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
#include <vector>
#include <functional>

#include "flag_machine.hpp"
#include "rotms_operations.hpp"

typedef std::vector<std::function<void()>> TransitionOps;

/***
* Current design: 
* StateBase should have the virtual functions of all possible state transition operations (edges).
* These virtual functions are to be inhereted by the subclasses of StateBase. If they are not 
* inhereted, the operation or transition is not possible from that state.
*
* NO LONGER USING THIS :
* Old design: 
* nested siwch-case.
***/

class StateBase
{

public:

    StateBase(
        int state_num,
        std::vector<StateBase*>& v,
        FlagMachineTMS& f,
        TMSOperations& ops);
    virtual ~StateBase();

    FlagMachineTMS& flags_;
    
    bool CheckActivated();
    void Activate();
    void Deactivate();
    int GetStateNum();
    static bool CheckIfUniqueActivation(const std::vector<StateBase*>& states);
    static int GetActivatedState(const std::vector<StateBase*>& states);

    virtual int LandmarksPlanned();
    virtual int LandmarksDigitized();
    virtual int ToolPosePlanned();
    virtual int Registered();

    virtual int ClearLandmarks();
    virtual int ClearDigitization();
    virtual int ClearRegistration();
    virtual int ClearToolPosePlan();

    virtual int ReinitState();
    virtual int UsePrevRegister();

protected:
    
    const int state_num_;
    const std::vector<StateBase*>& states_;
    TMSOperations& ops_;
    bool activated_;
    
    virtual void TransitionNotPossible();
    void Transition(int target_state, TransitionOps funcs);

};
