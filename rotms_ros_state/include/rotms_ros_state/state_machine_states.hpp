#pragma once
#include "flag_machine.hpp"
#include "state_machine.hpp"
#include <vector>

bool CheckFlagIntegrity(std::vector<WorkState>& states);

const std::vector<WorkState> GetStatesVector(FlagMachine& f);

class State0000 : public WorkState
{
public:
    State0000(std::vector<WorkState>& v, FlagMachine& f);
private:
    void LandmarksPlanned() override;
    void ToolPosePlanned() override;
};

class State1000 : public WorkState
{
public:
    State1000(std::vector<WorkState>& v, FlagMachine& f);
private:
    void LandmarksDigitized() override;
    void ToolPosePlanned() override;
    void ClearLandmarks() override;
    void RePlanLandmarks() override;
};

class State1100 : public WorkState
{
public:
    State1100(std::vector<WorkState>& v, FlagMachine& f);
private:
    void ToolPosePlanned() override;
    void Registered() override;
    void ClearDigitization() override;
    void ClearLandmarks() override;
    void ReDigitize() override;
};

class State1101 : public WorkState
{
public:
    State1101(std::vector<WorkState>& v, FlagMachine& f);
private:
    void ToolPosePlanned() override;
    void ClearRegistration() override;
    void ClearLandmarks() override;
};

class State0010 : public WorkState
{
public:
    State0010(std::vector<WorkState>& v, FlagMachine& f);
private:
    void LandmarksPlanned() override;
    void ClearToolPosePlan() override;
    void RePlanToolPose() override;
};

class State1010 : public WorkState
{
public:
    State1010(std::vector<WorkState>& v, FlagMachine& f);
private:
    void ClearToolPosePlan() override;
    void ClearLandmarks() override;
    void RePlanToolPose() override;
    void RePlanLandmarks() override;
    void LandmarksDigitized() override;
};

class State1110 : public WorkState
{
public:
    State1110(std::vector<WorkState>& v, FlagMachine& f);
private:
    void ClearToolPosePlan() override;
    void ReDigitize() override;
    void RePlanToolPose() override;
    void ClearDigitization() override;
    void ClearLandmarks() override;
    void Registered() override;
};

class State1111 : public WorkState
{
public:
    State1111(std::vector<WorkState>& v, FlagMachine& f);
private:
    void ClearToolPosePlan() override;
    void RePlanToolPose() override;
    void ClearRegistration() override;
    void ClearLandmarks() override;
};