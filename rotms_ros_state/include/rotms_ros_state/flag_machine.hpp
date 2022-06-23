#pragma once

class FlagMachine
{

public:

    FlagMachine();

    static void PlanLandmarks();
    static void DigitizeLandmarks();
    static void PlanPlacePose();
    static void CompleteRegistration();

    static void UnPlanLandmarks();
    static void UnDigitizeLandmarks();
    static void UnPlanPlacePose();
    static void UnCompleteRegistration();

private:

    static bool flag_landmark_planned_;
    static bool flag_landmark_digitized_;
    static bool flag_placepose_planned_;
    static bool flag_registration_completed_;
};