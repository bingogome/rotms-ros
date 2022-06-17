#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

class Dispatcher
{

public:

    Dispatcher(ros::NodeHandle& n);

private:

    ros::Publisher pub_registration_ = 
        n_.advertise<geometry_msgs::Pose>("/Rotms/State/Update/Registration", 5);

    void RegistrationCallBack(const std_msgs::String::ConstPtr& msg);

};