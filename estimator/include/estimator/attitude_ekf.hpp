#ifndef ATTITUDE_EKF_HPP
#define ATTITUDE_EKF_HPP

#include <Eigen/Core>
#include "uav_msgs/Sensors.h"
#include "uav_msgs/State.h"

class AttitudeEKF
{
public:
    AttitudeEKF();
    virtual ~AttitudeEKF();
    void update(double ts, const uav_msgs::SensorsConstPtr& meas, uav_msgs::State& state);

protected:
    void propagateModel(uav_msgs::State& state);
    void measurementUpdate(uav_msgs::State& state, const uav_msgs::SensorsConstPtr& measurement);
    Eigen::Vector2d f(const Eigen::Vector2d& x, const uav_msgs::State& u);
    Eigen::Vector3d h(const Eigen::Vector2d& x, const uav_msgs::State& u);
    void updateJacobianA(const Eigen::Vector2d& x, const uav_msgs::State& state);
    void updateJacobianC(const Eigen::Vector2d& x, const uav_msgs::State& state);

private:
    Eigen::Matrix2d m_Q;
    Eigen::Matrix3d m_Q_gyro;
    Eigen::Matrix3d m_R_accel;
    int m_N;
    Eigen::Vector2d m_xhat;
    Eigen::Matrix2d m_P;
    double m_ts;

    double m_eps;
    Eigen::Vector3d m_omega, m_h, m_y;
    Eigen::Matrix2d m_A, m_Ad;
    Eigen::Matrix<double,3,2> m_C;
    Eigen::Matrix<double,2,3> m_G, m_Gd, m_L;
};

#endif // ATTITUDE_EKF_HPP
