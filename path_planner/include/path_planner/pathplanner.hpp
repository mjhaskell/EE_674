#ifndef PATHPLANNER_HPP
#define PATHPLANNER_HPP

#include <ros/ros.h>
#include "uav_msgs/WaypointArray.h"

class PathPlanner
{
public:
    PathPlanner();
    virtual ~PathPlanner();
    void publishWaypoints();

protected:
    void setupWaypoints();

private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_nh_private;
    ros::Publisher m_wpts_pub;

    uav_msgs::Waypoint m_wpt;
    uav_msgs::WaypointArray m_wpts_msg;
};

#endif // PATHPLANNER_HPP
