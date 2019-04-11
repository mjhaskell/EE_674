#ifndef OBSERVER_HPP
#define OBSERVER_HPP

#include "estimator/alphafilter.hpp"
#include "estimator/attitude_ekf.hpp"
#include "estimator/position_ekf.hpp"
#include <ros/ros.h>
#include "uav_msgs/Sensors.h"
#include "uav_msgs/State.h"

namespace est
{

class Observer
{
public:
    Observer();
    virtual ~Observer();

protected:
    void sensorCallback(const uav_msgs::SensorsConstPtr& msg);
    void initializeStateMsg();

private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_nh_private;
    ros::Subscriber m_sensor_sub;
    ros::Publisher m_state_pub;

    uav_msgs::State m_xhat;

    AlphaFilter m_lpf_gyro_x;
    AlphaFilter m_lpf_gyro_y;
    AlphaFilter m_lpf_gyro_z;
    AlphaFilter m_lpf_accel_x;
    AlphaFilter m_lpf_accel_y;
    AlphaFilter m_lpf_accel_z;
    AlphaFilter m_lpf_static;
    AlphaFilter m_lpf_diff;

    PositionEKF m_pos_ekf;
    AttitudeEKF m_att_ekf;

    double m_rho;
    double m_gravity;
    double m_prev_t;
};

} // end namespace est

#endif // OBSERVER_HPP 
