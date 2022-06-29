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
* WorkState should have the virtual functions of all possible state transition operations (edges).
* These virtual functions are to be inhereted by the subclasses of WorkState. If they are not 
* inhereted, the operation or transition is not possible from that state.
*
* NO LONGER USING THIS :
* Old design: 
* nested siwch-case.
***/

class WorkState
{

public:

    WorkState(
        int state_num,
        std::vector<WorkState*>& v,
        FlagMachine& f,
        TMSOperations& ops);
    virtual ~WorkState();
    
    bool CheckActivated();
    void Activate();
    void Deactivate();
    int GetStateNum();
    static bool CheckIfUniqueActivation(const std::vector<WorkState*>& states);
    static int GetActivatedState(const std::vector<WorkState*>& states);

    virtual int LandmarksPlanned();
    virtual int LandmarksDigitized();
    virtual int ToolPosePlanned();
    virtual int Registered();

    virtual int ClearLandmarks();
    virtual int ClearDigitization();
    virtual int ClearRegistration();
    virtual int ClearToolPosePlan();

protected:
    
    const int state_num_;
    const std::vector<WorkState*>& states_;
    FlagMachine& flags_;
    TMSOperations& ops_;
    bool activated_;
    
    virtual void TransitionNotPossible();
    void Transition(int target_state, TransitionOps funcs);

};
