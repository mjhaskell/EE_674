#ifndef PATHMANAGER_HPP
#define PATHMANAGER_HPP

#include <ros/ros.h>
#include <Eigen/Core>
#include "path_manager/dubinsparams.hpp"

class PathManager
{
public:
    PathManager();
    virtual ~PathManager();

private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_nh_private;
    ros::Subscriber m_wpts_sub;
    ros::Publisher m_path_pub;

    int m_ptr_prev;
    int m_ptr_cur;
    int m_ptr_next;
    bool m_ptrs_updated;
    bool m_need_new_wpts;
    int m_num_waypoints;
    Eigen::Vector3d m_halfspace_n, m_halfspace_r;
    int m_manager_state;
    DubinsParams m_dubins_path;
    bool m_state_changed;
};

#endif // PATHMANAGER_HPP
