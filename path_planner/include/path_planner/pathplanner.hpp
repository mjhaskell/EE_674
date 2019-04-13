#ifndef PATHPLANNER_HPP
#define PATHPLANNER_HPP

#include <ros/ros.h>
#include "uav_msgs/WaypointArray.h"
#include "uav_msgs/Map.h"

class PathPlanner
{
public:
    PathPlanner();
    virtual ~PathPlanner();
    void publishWaypoints();

protected:
    void mapCallback(const uav_msgs::MapConstPtr& msg);
    void setupWaypoints();

private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_nh_private;
    ros::Subscriber m_map_sub;
    ros::Publisher m_wpts_pub;

    uav_msgs::Waypoint m_wpt;
    uav_msgs::WaypointArray m_wpts_msg;
};

#endif // PATHPLANNER_HPP
