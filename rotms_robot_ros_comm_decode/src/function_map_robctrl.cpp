#include <map>
#include <string>
#include <ros/ros.h>

/**
* This maps the functions to the received cmd.
*/


typedef std::map<std::string, void(*)(ros::Publisher&)> FuncMap;

const FuncMap GetFuncMapRobCtrl()
{
    FuncMap fm;

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

void GetJntAngs(ros::Publisher& pub)
{
    msg.data = "get_jnt";
    pub.publish(msg);
}

void GetEffPose(ros::Publisher& pub)
{
    msg.data = "get_eff";
    pub.publish(msg);
}

void ExecuteMotion(ros::Publisher& pub)
{
    msg.data = "get_jnt";
    pub.publish(msg);
}

void ExecuteMoveConfirm(ros::Publisher& pub)
{
    msg.data = "get_jnt";
    pub.publish(msg);
}

void SessionEnd(ros::Publisher& pub)
{
    msg.data = "get_jnt";
    pub.publish(msg);
}

void ManualAdjustBackwards(ros::Publisher& pub)
{
    msg.data = "get_jnt";
    pub.publish(msg);
}

void ManualAdjustApproach(ros::Publisher& pub)
{
    msg.data = "get_jnt";
    pub.publish(msg);
}

void ManualAdjustAway(ros::Publisher& pub)
{
    msg.data = "get_jnt";
    pub.publish(msg);
}

void ManualAdjustForward(ros::Publisher& pub)
{
    msg.data = "get_jnt";
    pub.publish(msg);
}

void ManualAdjustLeft(ros::Publisher& pub)
{
    msg.data = "get_jnt";
    pub.publish(msg);
}

void ManualAdjustPitch(ros::Publisher& pub)
{
    msg.data = "get_jnt";
    pub.publish(msg);
}

void ManualAdjustRight(ros::Publisher& pub)
{
    msg.data = "get_jnt";
    pub.publish(msg);
}

void ManualAdjustRoll(ros::Publisher& pub)
{
    msg.data = "get_jnt";
    pub.publish(msg);
}

void ManualAdjustYaw(ros::Publisher& pub)
{
    msg.data = "get_jnt";
    pub.publish(msg);
}