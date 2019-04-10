#ifndef POSITION_EKF_HPP
#define POSITION_EKF_HPP

#include <Eigen/Core>
#include "uav_msgs/Sensors.h"
#include "uav_msgs/State.h"

class PositionEKF
{
    typedef Eigen::Matrix<double,7,7> Mat7;
    typedef Eigen::Matrix<double,7,1> Vec7;

public:
    PositionEKF();
    virtual ~PositionEKF();
    void update(double ts, const uav_msgs::SensorsConstPtr& msg, uav_msgs::State& state);

protected:
    void propagateModel(uav_msgs::State& state);
    void measurementUpdate(uav_msgs::State& state, const uav_msgs::SensorsConstPtr& measurement);
    Vec7 f(const Vec7& x, const uav_msgs::State& u);
    Eigen::Vector4d h_gps(const Vec7& x, const uav_msgs::State& u);
    Eigen::Vector2d h_pseudo(const Vec7& x, const uav_msgs::State& u);
    void updateJacobians(Vec7 x, uav_msgs::State state);

private:
    Mat7 m_Q, m_P;
    Eigen::Matrix4d m_R_gps;
    Eigen::Matrix2d m_R_pseudo;
    int m_N;
    Vec7 m_xhat;
    double m_ts;

    double m_gps_n_old;
    double m_gps_e_old;
    double m_gps_Vg_old;
    double m_gps_chi_old;

    double m_eps;
    Eigen::Vector4d m_h_gps, m_y_gps;
    Eigen::Vector2d m_h_pseudo;
    Mat7 m_A, m_Ad;
    Eigen::Matrix<double,3,2> m_C;
    Eigen::Matrix<double,2,3> m_L;
};

#endif // POSITION_EKF_HPP
