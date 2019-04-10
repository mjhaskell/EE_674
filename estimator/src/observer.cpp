#include "estimator/observer.hpp"

namespace est
{

Observer::Observer() :
    m_nh{ros::NodeHandle()},
    m_nh_private{"~"},
    m_lpf_static{0.9,0.0},
    m_prev_t{0.0}
{
    m_rho = m_nh_private.param<double>("rho", 1.2682);
    m_gravity = m_nh_private.param<double>("gravity", 9.80665);

    uint32_t queue_size{5};
    m_sensor_sub = m_nh.subscribe("sensors",queue_size,&Observer::sensorCallback,this);
    m_state_pub = m_nh.advertise<uav_msgs::State>("states/estimates",queue_size);
}

Observer::~Observer()
{
}

void Observer::sensorCallback(const uav_msgs::SensorsConstPtr& msg)
{
    if (m_prev_t == 0.0)
    {
        m_prev_t = msg->header.stamp.toSec();
        return;
    }
    double now{msg->header.stamp.toSec()};
    double dt{now - m_prev_t};
    m_prev_t = now;
    if (dt <= 0)
        return;

    m_xhat.p = m_lpf_gyro_x.update(msg->gyro_x - m_xhat.bx);
    m_xhat.q = m_lpf_gyro_y.update(msg->gyro_y - m_xhat.by);
    m_xhat.r = m_lpf_gyro_z.update(msg->gyro_z - m_xhat.bz);

    m_xhat.h = m_lpf_static.update(msg->static_pressure) / (m_rho*m_gravity);
    m_xhat.Va = sqrt(2.0 * m_lpf_diff.update(msg->diff_pressure) / m_rho);

    m_att_ekf.update(dt, msg, m_xhat);
    m_pos_ekf.update(dt, msg, m_xhat);

    m_xhat.alpha = m_xhat.theta;
    m_xhat.beta = 0.0;
    m_xhat.bx = 0.0;
    m_xhat.by = 0.0;
    m_xhat.bz = 0.0;
    m_xhat.header.stamp = ros::Time::now();

    m_state_pub.publish(m_xhat);
}

} // end namespace est
