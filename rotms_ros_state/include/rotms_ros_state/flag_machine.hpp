#pragma once

class FlagMachine
{

public:

    FlagMachine();

    static void PlanLandmarks();
    static void DigitizeLandmarks();
    static void PlanToolPose();
    static void CompleteRegistration();

    static void UnPlanLandmarks();
    static void UnDigitizeLandmarks();
    static void UnPlanToolPose();
    static void UnCompleteRegistration();

    static bool GetFlagLandmarkPlanned();
    static bool GetFlagLandmarkDigitized();
    static bool GetFlagToolPosePlanned();
    static bool GetFlagRegistered();

private:

    static bool flag_landmark_planned_;
    static bool flag_landmark_digitized_;
    static bool flag_toolpose_planned_;
    static bool flag_registration_completed_;
};