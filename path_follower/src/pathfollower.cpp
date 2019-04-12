#include "path_follower/pathfollower.hpp"
#include <Eigen/Dense>

PathFollower::PathFollower() :
    m_nh{ros::NodeHandle()},
    m_nh_private{"~"},
    m_gravity{9.80665}
{
    m_chi_inf = m_nh_private.param<double>("chi_inf", 80*3.14159/180.0);
    m_k_path = m_nh_private.param<double>("k_path", 0.01);
    m_k_orbit = m_nh_private.param<double>("k_orbit", 2.5);

    this->initializePathMsg();

    uint32_t queue_size{5};
    m_path_sub = m_nh.subscribe("command/path",queue_size,&PathFollower::pathCallback,this);
    m_state_sub = m_nh.subscribe("states/truth",queue_size,&PathFollower::stateCallback,this);
    m_cmd_pub = m_nh.advertise<uav_msgs::Command>("command/high_level", queue_size);
}

void PathFollower::pathCallback(const uav_msgs::PathConstPtr& msg)
{
    m_path_msg = *msg;
    m_path_msg.path_changed = true;
}

void PathFollower::stateCallback(const uav_msgs::StateConstPtr& msg)
{
    m_state = *msg;

    if (m_path_msg.mode == uav_msgs::Path::MODE_LINE)
    {
        if (m_path_msg.path_changed)
        {
            followStraightLine();
            m_cmd_pub.publish(m_cmd_msg);
        }
    }
    else if (m_path_msg.mode == uav_msgs::Path::MODE_ORBIT)
    {
        followOrbit();
        m_cmd_pub.publish(m_cmd_msg);
    }
    else
        ROS_ERROR("[path_follower] Received unhandled path mode.");
    m_path_msg.path_changed = false;
}

void wrap(double& chi_c, double chi)
{
    static double PI{3.14159};

    while (chi_c-chi > PI)
        chi_c -= 2*PI;
    while (chi_c-chi < -PI)
        chi_c += 2*PI;
}

void PathFollower::followStraightLine()
{
    m_q << m_path_msg.line_direction.x,
           m_path_msg.line_direction.y,
           m_path_msg.line_direction.z;

    double chi_q{atan2(m_q(1), m_q(0))};
    wrap(chi_q, m_state.chi);

    m_Ri_p << cos(chi_q), sin(chi_q), 0,
           -sin(chi_q), cos(chi_q), 0,
            0,          0,          1;

    m_p_i << m_state.pn, m_state.pe, -m_state.h;
    m_r_i << m_path_msg.line_origin.x,
             m_path_msg.line_origin.y,
             m_path_msg.line_origin.z;

    // chi_c
    m_ep_i = m_p_i - m_r_i;
    double epy{(m_Ri_p*m_ep_i)(1)};
    double chi_d{-m_chi_inf * 2/3.14159 * atan(m_k_path*epy)};
    double chi_c{chi_q + chi_d};

    // h_c
    m_n = m_q.cross(Eigen::Vector3d{0,0,1});
    m_n /= m_n.norm();
    m_s = m_ep_i - (m_ep_i.transpose() * m_n)*m_n;
    double sd{m_q(2)*sqrt((m_s(0)*m_s(0)+m_s(1)*m_s(1))/(m_q(0)*m_q(0)+m_q(1)*m_q(1)))};
    double h_c{-m_r_i(2) - sd};

    m_cmd_msg.Va = m_path_msg.airspeed;
    m_cmd_msg.chi = chi_c;
    m_cmd_msg.h = h_c;
    m_cmd_msg.phi_ff = 0.0;
    m_cmd_msg.header.stamp = ros::Time::now();
}

void PathFollower::followOrbit()
{
    m_p_i << m_state.pn, m_state.pe, -m_state.h;
    Eigen::Vector3d orbit_center;
    orbit_center << m_path_msg.orbit_center.x,
                    m_path_msg.orbit_center.y,
                    m_path_msg.orbit_center.z;
    m_d = m_p_i - orbit_center;
    double R{m_path_msg.orbit_radius};

    int direction{0};
    if (m_path_msg.orbit_direction == uav_msgs::Path::DIR_CW)
        direction = 1;
    else if (m_path_msg.orbit_direction == uav_msgs::Path::DIR_CCW)
        direction = -1;
    else
        ROS_ERROR("[path_follower] Received invalid orbit direction from path.");

    // chi_c
    double var_phi{atan2(m_d(1), m_d(0))};
    double chi0{var_phi + direction*3.14159/2.0};
    double chi_c{chi0 + direction*atan(m_k_orbit * (m_d.norm()-R)/R)};

    // phi feedforward
    double phi_ff{direction*atan2(pow(m_state.Vg,2), (m_gravity*R*cos(m_state.chi-m_state.psi)))};

    m_cmd_msg.Va = m_path_msg.airspeed;
    m_cmd_msg.chi = chi_c;
    m_cmd_msg.h = -m_path_msg.orbit_center.z;
    m_cmd_msg.phi_ff = phi_ff;
    m_cmd_msg.header.stamp = ros::Time::now();
}

void PathFollower::initializePathMsg()
{
    m_path_msg.mode = uav_msgs::Path::MODE_LINE;
    m_path_msg.header.stamp = ros::Time::now();
    m_path_msg.airspeed = 25.0;
    m_path_msg.line_origin.x = 0;
    m_path_msg.line_origin.y = 0;
    m_path_msg.line_origin.z = -50;
    m_path_msg.path_changed = true;
    m_path_msg.line_direction.x = 1;
    m_path_msg.line_direction.y = 0;
    m_path_msg.line_direction.z = 0;
}
