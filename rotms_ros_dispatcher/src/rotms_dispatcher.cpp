#include "rotms_dispatcher.hpp"
#include <geometry_msgs/Pose.h>

void Dispatcher::RegistrationCallBack(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data.compare("_end_robot_connection_")==0)
    {
        // Read the points-pair from cache file

        // Perform registration algorithm
        std::tuple<std::vector<std::vector<double>>, std::vector<double>> reg = 
            getRegistrationResult(a,b);
        std::vector<std::vector<double>> R = std::get<0>(tulp);
        std::vector<double> p = std::get<1>(tulp);

        // Convert to quaternion

        // Write the registration result to a cache file

        // Publish the registration result to rot

        pub_registration_.publish()
    }
    
}