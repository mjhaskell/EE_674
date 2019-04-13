#include "path_planner/pathplanner.hpp"

PathPlanner::PathPlanner() :
    m_nh{ros::NodeHandle()},
    m_nh_private{"~"}
{
    setupDefaultWaypoints();

    uint32_t queue_size{5};
    m_map_sub = m_nh.subscribe("command/map",queue_size,&PathPlanner::mapCallback,this);
    m_wpts_pub = m_nh.advertise<uav_msgs::WaypointArray>("command/waypoints",queue_size);
}

PathPlanner::~PathPlanner()
{}

void PathPlanner::publishWaypoints()
{
    for (int i{0}; i < 5; i++)
        m_wpts_pub.publish(m_wpts_msg);
}

void PathPlanner::mapCallback(const uav_msgs::MapConstPtr &msg)
{
    if (!msg->map_changed)
        return;
    if (msg->use_default_waypoints)
    {
        setupDefaultWaypoints();
        m_wpts_pub.publish(m_wpts_msg);
    }
}

float rad(double degree)
{
    return degree * 3.14159/180.0;
}

void PathPlanner::setupDefaultWaypoints()
{
    float Va{25.0};

    m_wpt.ned.x = 0.0;
    m_wpt.ned.y = 0.0;
    m_wpt.ned.z = -50.0;
    m_wpt.chi = 0.0;
    m_wpt.airspeed = Va;
    m_wpts_msg.waypoints.push_back(m_wpt);

    m_wpt.ned.x = 1000.0;
    m_wpt.ned.y = 0.0;
    m_wpt.ned.z = -50.0;
    m_wpt.chi = rad(45);
    m_wpt.airspeed = Va;
    m_wpts_msg.waypoints.push_back(m_wpt);

    m_wpt.ned.x = 0.0;
    m_wpt.ned.y = 1000.0;
    m_wpt.ned.z = -50.0;
    m_wpt.chi = rad(45);
    m_wpt.airspeed = Va;
    m_wpts_msg.waypoints.push_back(m_wpt);

    m_wpt.ned.x = 1000.0;
    m_wpt.ned.y = 1000.0;
    m_wpt.ned.z = -50.0;
    m_wpt.chi = rad(-135);
    m_wpt.airspeed = Va;
    m_wpts_msg.waypoints.push_back(m_wpt);

    m_wpts_msg.mode = uav_msgs::WaypointArray::MODE_LINE;
    m_wpts_msg.wrap_waypoints = true;
    m_wpts_msg.waypoints_changed = true;
    m_wpts_msg.manager_requests_waypoints = false;
    m_wpts_msg.num_waypoints = m_wpts_msg.waypoints.size();
    m_wpts_msg.header.stamp = ros::Time::now();
}
