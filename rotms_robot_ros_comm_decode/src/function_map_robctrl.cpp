#include <map>
#include <string>
#include <ros/ros.h>
#include "function_map_robctrl.hpp"
#include "decode_node.hpp"


CommDecoderPubsRobCtrl::CommDecoderPubsRobCtrl() : CommDecoderPubs() {}

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

void GetJntAngs(CommDecoderPubsRobCtrl& pubs, std::string& ss)
{
    ROS_INFO_STREAM(pubs.a);
}

void GetEffPose(CommDecoderPubsRobCtrl& pubs, std::string& ss)
{
    
}

void ExecuteMotion(CommDecoderPubsRobCtrl& pubs, std::string& ss)
{

}

void ExecuteMoveConfirm(CommDecoderPubsRobCtrl& pubs, std::string& ss)
{

}

void SessionEnd(CommDecoderPubsRobCtrl& pubs, std::string& ss)
{

}

void ManualAdjustBackwards(CommDecoderPubsRobCtrl& pubs, std::string& ss)
{

}

void ManualAdjustApproach(CommDecoderPubsRobCtrl& pubs, std::string& ss)
{

}

void ManualAdjustAway(CommDecoderPubsRobCtrl& pubs, std::string& ss)
{

}

void ManualAdjustForward(CommDecoderPubsRobCtrl& pubs, std::string& ss)
{

}

void ManualAdjustLeft(CommDecoderPubsRobCtrl& pubs, std::string& ss)
{

}

void ManualAdjustPitch(CommDecoderPubsRobCtrl& pubs, std::string& ss)
{

}

void ManualAdjustRight(CommDecoderPubsRobCtrl& pubs, std::string& ss)
{

}

void ManualAdjustRoll(CommDecoderPubsRobCtrl& pubs, std::string& ss)
{

}

void ManualAdjustYaw(CommDecoderPubsRobCtrl& pubs, std::string& ss)
{

}