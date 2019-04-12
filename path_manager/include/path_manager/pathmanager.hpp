#ifndef PATHMANAGER_HPP
#define PATHMANAGER_HPP

#include <ros/ros.h>
#include <Eigen/Core>
#include "path_manager/dubinsparams.hpp"
#include "uav_msgs/WaypointArray.h"
#include "uav_msgs/Path.h"
#include "uav_msgs/State.h"

class PathManager
{
public:
    PathManager();
    virtual ~PathManager();

protected:
    void stateCallback(const uav_msgs::StateConstPtr& msg);
    void wptCallback(const uav_msgs::WaypointArrayConstPtr& msg);
    void lineManager(const uav_msgs::StateConstPtr& state);
    void filletManager(const uav_msgs::StateConstPtr& state);
    void dubinsManager(const uav_msgs::StateConstPtr& state);
    void initializePtrs();
    void incrementPtrs();
    bool inHalfSpace(const Eigen::Vector3d& pos);

private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_nh_private;
    ros::Subscriber m_state_sub;
    ros::Subscriber m_wpts_sub;
    ros::Publisher m_path_pub;

    uav_msgs::WaypointArray m_wpts;
    uav_msgs::Path m_path_msg;

    unsigned long m_ptr_prev;
    unsigned long m_ptr_cur;
    unsigned long m_ptr_next;
    bool m_ptrs_updated;
    bool m_need_new_wpts;
    int m_num_waypoints;
    Eigen::Vector3d m_halfspace_n, m_halfspace_r;
    int m_manager_state;
    DubinsParams m_dubins_path;
    bool m_state_changed;
    double m_R_min;

    Eigen::Vector3d m_w_im1, m_w_i, m_w_ip1;
    Eigen::Vector3d m_q_im1, m_q_i, m_n_i, m_p;
};

#endif // PATHMANAGER_HPP
