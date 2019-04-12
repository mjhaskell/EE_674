#ifndef PATHFOLLOWER_HPP
#define PATHFOLLOWER_HPP

#include <ros/ros.h>
#include "uav_msgs/Path.h"
#include "uav_msgs/Command.h"
#include "uav_msgs/State.h"
#include <Eigen/Core>

class PathFollower
{
public:
    PathFollower();

protected:
    void pathCallback(const uav_msgs::PathConstPtr& msg);
    void stateCallback(const uav_msgs::StateConstPtr& msg);
    void followStraightLine();
    void followOrbit();
    void initializePathMsg();

private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_nh_private;
    ros::Subscriber m_path_sub;
    ros::Subscriber m_state_sub;
    ros::Publisher m_cmd_pub;

    uav_msgs::Path m_path_msg;
    uav_msgs::Command m_cmd_msg;
    uav_msgs::State m_state;

    double m_chi_inf;
    double m_k_path;
    double m_k_orbit;
    double m_gravity;

    Eigen::Vector3d m_q, m_p_i, m_r_i, m_ep_i, m_n, m_s, m_d;
    Eigen::Matrix3d m_Ri_p;
};

#endif // PATHFOLLOWER_HPP
