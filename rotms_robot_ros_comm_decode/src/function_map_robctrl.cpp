#include <map>
#include <string>
#include <ros/ros.h>
#include "function_map_robctrl.hpp"

/**
* This maps the functions to the received cmd.
*/

const std::map<std::string, void(*)(CommDecoderPubs&, std::string&)> GetFuncMapRobCtrl()
{
    std::map<std::string, void(*)(CommDecoderPubs&, std::string&)> fm;

    fm["GET_JNT_ANGS"] = GetJntAngs;
    fm["GET_EFF_POSE"] = GetEffPose;

    fm["EXECUTE_MOTION"] = ExecuteMotion;
    fm["EXECUTE_MOVE_CONFIRM"] = ExecuteMoveConfirm;

    fm["SESSION_END"] = SessionEnd;

    fm["MAN_ADJUST_BACKWARD"] = ManualAdjustBackwards;
    fm["MAN_ADJUST_CLOSER"] = ManualAdjustApproach;
    fm["MAN_ADJUST_FARTHER"] = ManualAdjustAway;
    fm["MAN_ADJUST_FORWARD"] = ManualAdjustForward;
    fm["MAN_ADJUST_LEFT"] = ManualAdjustLeft;
    fm["MAN_ADJUST_PITCH"] = ManualAdjustPitch;
    fm["MAN_ADJUST_RIGHT"] = ManualAdjustRight;
    fm["MAN_ADJUST_ROLL"] = ManualAdjustRoll;
    fm["MAN_ADJUST_YAW"] = ManualAdjustYaw;
    return fm;
}

void GetJntAngs(CommDecoderPubs& pubs, std::string& ss)
{
    msg.data = "get_jnt";
}

void GetEffPose(CommDecoderPubs& pubs, std::string& ss)
{
    msg.data = "get_eff";
}

void ExecuteMotion(CommDecoderPubs& pubs, std::string& ss)
{

}

void ExecuteMoveConfirm(CommDecoderPubs& pubs, std::string& ss)
{

}

void SessionEnd(CommDecoderPubs& pubs, std::string& ss)
{

}

void ManualAdjustBackwards(CommDecoderPubs& pubs, std::string& ss)
{

}

void ManualAdjustApproach(CommDecoderPubs& pubs, std::string& ss)
{

}

void ManualAdjustAway(CommDecoderPubs& pubs, std::string& ss)
{

}

void ManualAdjustForward(CommDecoderPubs& pubs, std::string& ss)
{

}

void ManualAdjustLeft(CommDecoderPubs& pubs, std::string& ss)
{

}

void ManualAdjustPitch(CommDecoderPubs& pubs, std::string& ss)
{

}

void ManualAdjustRight(CommDecoderPubs& pubs, std::string& ss)
{

}

void ManualAdjustRoll(CommDecoderPubs& pubs, std::string& ss)
{

}

void ManualAdjustYaw(CommDecoderPubs& pubs, std::string& ss)
{

}