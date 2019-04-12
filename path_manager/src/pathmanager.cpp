#include "path_manager/pathmanager.hpp"

PathManager::PathManager() :
    m_nh{ros::NodeHandle()},
    m_nh_private{"~"},
    m_need_new_wpts{true},
    m_num_waypoints{0},
    m_manager_state{1},
    m_state_changed{true},
    m_R_min{25*25/9.80665/tan(20*3.14159/180.0)}
{
    initializePtrs();

    uint32_t queue_size{5};
    m_state_sub = m_nh.subscribe("states/truth",queue_size,&PathManager::stateCallback,this);
    m_wpts_sub = m_nh.subscribe("command/waypoints",queue_size,&PathManager::wptCallback,this);
    m_path_pub = m_nh.advertise<uav_msgs::Path>("command/path", queue_size);
}

PathManager::~PathManager()
{
}

void PathManager::stateCallback(const uav_msgs::StateConstPtr& msg)
{
    if (m_wpts.waypoints_changed)
    {
        m_wpts.waypoints_changed = false;
        m_num_waypoints = m_wpts.num_waypoints;
        initializePtrs();
        m_manager_state = 1;
        m_need_new_wpts = false;
    }

    if (m_path_msg.path_changed)
        m_path_msg.path_changed = false;

    if (m_num_waypoints == 0)
    {
        m_wpts.manager_requests_waypoints = true;
        ROS_WARN("[path_manager] No waypoints received. Requesting new waypoints.");
        return;
    }
    else if (m_num_waypoints == 1)
        int a{0};
    else if (m_num_waypoints == 2)
        int b{0};


    if (m_wpts.mode == uav_msgs::WaypointArray::MODE_LINE)
        lineManager(msg);
    else if (m_wpts.mode == uav_msgs::WaypointArray::MODE_FILLET)
        filletManager(msg);
    else if (m_wpts.mode == uav_msgs::WaypointArray::MODE_DUBINS)
        dubinsManager(msg);
    else
        ROS_ERROR("[path_manager] Undefined waypoint type.");

    m_path_pub.publish(m_path_msg);
}

void PathManager::wptCallback(const uav_msgs::WaypointArrayConstPtr& msg)
{
    m_wpts = *msg;
}

void PathManager::lineManager(const uav_msgs::StateConstPtr &state)
{
    m_p << state->pn, state->pe, -state->h;

    uav_msgs::Waypoint wpt;
    wpt = m_wpts.waypoints[m_ptr_prev];
    m_w_im1 << wpt.ned.x, wpt.ned.y, wpt.ned.z;
    wpt = m_wpts.waypoints[m_ptr_cur];
    m_w_i << wpt.ned.x, wpt.ned.y, wpt.ned.z;
    wpt = m_wpts.waypoints[m_ptr_next];
    m_w_ip1 << wpt.ned.x, wpt.ned.y, wpt.ned.z;

    m_q_im1 = m_w_i - m_w_im1;
    m_q_im1 /= m_q_im1.norm();
    m_q_i = m_w_ip1 - m_w_i;
    m_q_i /= m_q_i.norm();

    m_n_i = m_q_im1 + m_q_i;
    m_n_i /= m_n_i.norm();

    m_halfspace_r = m_w_i;
    m_halfspace_n = m_n_i;

    m_path_msg.airspeed = m_wpts.waypoints[m_ptr_cur].airspeed;
    m_path_msg.mode = uav_msgs::Path::MODE_LINE;

    if (inHalfSpace(m_p))
    {
        incrementPtrs();
        m_path_msg.path_changed = true;
        m_path_msg.line_origin.x = m_w_i(0);
        m_path_msg.line_origin.y = m_w_i(1);
        m_path_msg.line_origin.z = m_w_i(2);
        m_path_msg.line_direction.x = m_q_i(0);
        m_path_msg.line_direction.y = m_q_i(1);
        m_path_msg.line_direction.z = m_q_i(2);
    }
    else
    {
        m_path_msg.path_changed = false;
        m_path_msg.line_origin.x = m_w_im1(0);
        m_path_msg.line_origin.y = m_w_im1(1);
        m_path_msg.line_origin.z = m_w_im1(2);
        m_path_msg.line_direction.x = m_q_im1(0);
        m_path_msg.line_direction.y = m_q_im1(1);
        m_path_msg.line_direction.z = m_q_im1(2);
    }
}

void PathManager::filletManager(const uav_msgs::StateConstPtr &state)
{
    m_p << state->pn, state->pe, -state->h;

    uav_msgs::Waypoint wpt;
    wpt = m_wpts.waypoints[m_ptr_prev];
    m_w_im1 << wpt.ned.x, wpt.ned.y, wpt.ned.z;
    wpt = m_wpts.waypoints[m_ptr_cur];
    m_w_i << wpt.ned.x, wpt.ned.y, wpt.ned.z;
    wpt = m_wpts.waypoints[m_ptr_next];
    m_w_ip1 << wpt.ned.x, wpt.ned.y, wpt.ned.z;

    m_q_im1 = m_w_i - m_w_im1;
    m_q_im1 /= m_q_im1.norm();
    m_q_i = m_w_ip1 - m_w_i;
    m_q_i /= m_q_i.norm();

    double var_phi{acos(-m_q_im1.transpose()*m_q_i)};

    if (m_manager_state == 1)
    {
        m_path_msg.path_changed = m_state_changed;
        m_state_changed = false;
        m_path_msg.mode = uav_msgs::Path::MODE_LINE;
        m_path_msg.line_origin.x = m_w_im1(0);
        m_path_msg.line_origin.y = m_w_im1(1);
        m_path_msg.line_origin.z = m_w_im1(2);
        m_path_msg.line_direction.x = m_q_im1(0);
        m_path_msg.line_direction.y = m_q_im1(1);
        m_path_msg.line_direction.z = m_q_im1(2);
        m_path_msg.airspeed = wpt.airspeed;

        m_halfspace_r = m_w_i - (m_R_min/tan(var_phi/2.0))*m_q_im1;
        m_halfspace_n = m_q_im1;

        if (inHalfSpace(m_p))
        {
            m_manager_state = 2;
            m_state_changed = true;
        }
    }
    else
    {
        m_path_msg.path_changed = m_state_changed;
        m_state_changed = false;
        Eigen::Vector3d direction;
        direction = (m_q_im1 - m_q_i);
        direction /= direction.norm();
        Eigen::Vector3d c;
        c = m_w_i - (m_R_min/sin(var_phi/2.0))*direction;
        double lam{m_q_im1(0)*m_q_i(1)-m_q_im1(1)*m_q_i(0)};

        m_path_msg.mode = uav_msgs::Path::MODE_ORBIT;
        m_path_msg.airspeed = wpt.airspeed;
        m_path_msg.orbit_center.x = c(0);
        m_path_msg.orbit_center.y = c(1);
        m_path_msg.orbit_center.z = c(2);
        if (lam > 0)
            m_path_msg.orbit_direction = uav_msgs::Path::DIR_CW;
        else
            m_path_msg.orbit_direction = uav_msgs::Path::DIR_CCW;

        m_halfspace_r = m_w_i + (m_R_min/tan(var_phi/2.0))*m_q_i;
        m_halfspace_n = m_q_i;

        if (inHalfSpace(m_p))
        {
            incrementPtrs();
            m_manager_state = 1;
            m_state_changed = true;
        }
    }
}

void PathManager::dubinsManager(const uav_msgs::StateConstPtr &state)
{
    m_p << state->pn, state->pe, -state->h;
}

void PathManager::initializePtrs()
{
    m_ptr_prev = 0;
    m_ptr_cur = 1;
    m_ptr_next = 2;
    m_ptrs_updated = true;
}

void PathManager::incrementPtrs()
{
    m_ptr_prev++;
    m_ptr_prev *= (m_ptr_prev >= m_num_waypoints) ? 0 : 1;
    m_ptr_cur++;
    m_ptr_cur *= (m_ptr_cur >= m_num_waypoints) ? 0 : 1;
    m_ptr_next++;
    m_ptr_next *= (m_ptr_next >= m_num_waypoints) ? 0 : 1;
    m_ptrs_updated = true;
}

bool PathManager::inHalfSpace(const Eigen::Vector3d& pos)
{
    if ((pos - m_halfspace_r).transpose() * m_halfspace_n >= 0)
        return true;
    else
        return false;
}
