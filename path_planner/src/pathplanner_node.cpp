#include "path_planner/pathplanner.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;

    PathPlanner planner_obj;

    ros::spin();

    return 0;
}
