#include "estimator/attitude_ekf.hpp"
#include <Eigen/Dense>

AttitudeEKF::AttitudeEKF() :
    m_N{10},
    m_ts{.002/double(m_N)},
    m_eps{0.01}
{
    double deg2rad{3.14159/180.0};
    double gyro_var{pow(0.13*deg2rad, 2)};
    double accel_var{pow(0.0025*9.80665, 2)};

    m_Q = Eigen::Matrix2d::Identity() * 1e-6;
    m_Q_gyro = Eigen::Matrix3d::Identity() * gyro_var;
    m_R_accel = Eigen::Matrix3d::Identity() * accel_var;
    m_xhat.setZero();
    m_P = Eigen::Matrix2d::Identity() * 0.1;
}

AttitudeEKF::~AttitudeEKF()
{
}

void AttitudeEKF::update(double ts, const uav_msgs::SensorsConstPtr &meas, uav_msgs::State &state)
{
//    m_ts = ts / double(m_N);
    propagateModel(state);
    measurementUpdate(state, meas);
    state.phi = m_xhat(0);
    state.theta = m_xhat(1);
}

void AttitudeEKF::propagateModel(uav_msgs::State &state)
{
    static Eigen::Matrix2d I_2x2{Eigen::Matrix2d::Identity()};

    for (int i{0}; i < m_N; i++)
    {
        m_xhat += m_ts*f(m_xhat,state);
        updateJacobianA(m_xhat,state);

        m_Ad = I_2x2 + m_A*m_ts + m_A*m_A*m_ts*m_ts/2.0;
        m_Gd = m_G*m_ts;

        m_P = m_Ad*m_P*m_Ad.transpose() + m_Gd*m_Q_gyro*m_Gd.transpose() +
                m_Q*m_ts*m_ts;
    }
}

void AttitudeEKF::measurementUpdate(uav_msgs::State &state, const uav_msgs::SensorsConstPtr &measurement)
{
    static Eigen::Matrix2d I_2x2{Eigen::Matrix2d::Identity()};

    m_h = h(m_xhat, state);
    updateJacobianC(m_xhat,state);
    m_y << measurement->accel_x, measurement->accel_y, measurement->accel_z;

    Eigen::Matrix3d temp;
    temp = m_R_accel + m_C*m_P*m_C.transpose();
    m_L = m_P*m_C.transpose()*temp.inverse();
    m_P = (I_2x2 - m_L*m_C)*m_P*(I_2x2-m_L*m_C).transpose() + m_L*m_R_accel*m_L.transpose();
    m_xhat += m_L * (m_y - m_h);
}

Eigen::Vector2d AttitudeEKF::f(const Eigen::Vector2d& x, const uav_msgs::State& u)
{
    m_omega << u.p, u.q, u.r;

    m_G << 1.0, sin(x(0))*tan(x(1)), cos(x(0))*tan(x(1)),
           0.0, cos(x(0)), -sin(x(0));

    return m_G * m_omega;
}

Eigen::Vector3d AttitudeEKF::h(const Eigen::Vector2d &x, const uav_msgs::State &u)
{
    static double g{9.80665};

    double C_theta{cos(x(1))}, S_theta{sin(x(1))};
    Eigen::Vector3d _h;
    _h << (u.q*u.Va+g)*S_theta,
           u.r*u.Va*C_theta - u.p*u.Va*S_theta-g*C_theta*sin(x(0)),
          -u.q*u.Va*C_theta - g*C_theta*cos(x(0));
    return _h;
}

void AttitudeEKF::updateJacobianA(const Eigen::Vector2d& x, const uav_msgs::State& state)
{
    Eigen::Vector2d f_x, f_eps;
    f_x = f(x, state);

    Eigen::Vector2d x_eps;
    for (int i{0}; i < 2; i++)
    {
        x_eps = x;
        x_eps(i) += m_eps;
        m_A.col(i) = (f(x_eps, state) - f_x) / m_eps;
    }
}

void AttitudeEKF::updateJacobianC(const Eigen::Vector2d &x, const uav_msgs::State &state)
{
    Eigen::Vector3d h_x, h_eps;
    h_x = h(x,state);

    Eigen::Vector2d x_eps;
    for (int i{0}; i < 2; i++)
    {
        x_eps = x;
        x_eps(i) += m_eps;
        m_C.col(i) = (h(x_eps, state) - h_x) / m_eps;
    }
}
