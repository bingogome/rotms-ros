#pragma once
#include <vector>

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

    WorkState(std::vector<WorkState>& v);
    bool CheckActivated();
    void Activate();
    void Deactivate();
    void SetStateNum(int i);
    int GetStateNum();
    bool CheckIfUniqueActivation();

protected:
    
    int state_num_;
    std::vector<WorkState>& states_;
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

};
