#pragma once
#include <vector>
#include "flag_machine.hpp"
#include <functional>

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
        std::vector<WorkState>& v,
        FlagMachine& f);
    bool CheckActivated();
    void Activate();
    void Deactivate();
    int GetStateNum();
    static bool CheckIfUniqueActivation(std::vector<WorkState>& states);
    static WorkState& GetActivatedState(std::vector<WorkState>& states);

protected:
    
    const int state_num_;
    std::vector<WorkState>& states_;
    FlagMachine& flags_;
    bool activated_;

    virtual void FiducialsPlanned();
    virtual void FiducialsDigitized();
    virtual void ToolPosePlanned();
    virtual void Registered();

    virtual void ClearFiducials();
    virtual void ClearDigitization();
    virtual void ClearRegistration();
    virtual void ClearToolPosePlan();

    virtual void RePlanFiducials();
    virtual void ReDigitize();
    virtual void RePlanToolPose();
    
    virtual void TransitionNotPossible();
    void Transition(int target_state, std::function<void()> func);

};
