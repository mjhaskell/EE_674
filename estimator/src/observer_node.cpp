#include "estimator/observer.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "estimator");
    ros::NodeHandle nh;

    est::Observer observer_obj;

    ros::spin();

    return 0;
}
