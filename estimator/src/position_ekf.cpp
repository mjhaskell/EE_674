#include "estimator/position_ekf.hpp"
#include <Eigen/Dense>

PositionEKF::PositionEKF() :
    m_N{25},
    m_gps_n_old{9999},
    m_gps_e_old{9999},
    m_gps_Vg_old{9999},
    m_gps_chi_old{9999},
    m_eps{0.01}
{
    double gps_ne_var{pow(0.21, 2)};
    double gps_Vg_var{pow(0.05, 2)};
    double gps_chi_var{pow(0.005, 2)};

    m_Q = Mat7::Identity() * 0.01;
    m_R_gps = Eigen::Vector4d{gps_ne_var,gps_ne_var,gps_Vg_var,gps_chi_var}.asDiagonal();
    m_R_pseudo = Eigen::Matrix2d::Identity() * 0.01;
    m_xhat << 0,0,25,0,0,0,0;
    m_P = Mat7::Identity() * 0.5;
}

PositionEKF::~PositionEKF()
{

}

void PositionEKF::update(double ts, const uav_msgs::SensorsConstPtr &meas, uav_msgs::State &state)
{
    m_ts = ts / double(m_N);
    propagateModel(state);
    measurementUpdate(state, meas);
    state.pn = m_xhat(0);
    state.pe = m_xhat(1);
    state.Vg = m_xhat(2);
    state.chi = m_xhat(3);
    state.wn = m_xhat(4);
    state.we = m_xhat(5);
    state.psi = m_xhat(6);
}

void PositionEKF::propagateModel(uav_msgs::State &state)
{
    static Mat7 I{Mat7::Identity()};

    for (int i{0}; i < m_N; i++)
    {
        m_xhat += m_ts * f(m_xhat, state);
        updateJacobianA(m_xhat, state);
        m_Ad = I + m_A*m_ts + m_A*m_A*m_ts*m_ts/2.0;

        m_P = m_Ad*m_P*m_Ad.transpose() + m_Q*m_ts*m_ts;
    }
}

void wrap(double& chi_c, double chi)
{
    while (chi_c-chi > 3.14159)
        chi_c -= 2*3.14159;
    while (chi_c-chi < -3.14159)
        chi_c += 2*3.14159;
}

void PositionEKF::measurementUpdate(uav_msgs::State &state, const uav_msgs::SensorsConstPtr &measurement)
{
    static Mat7 I{Mat7::Identity()};

    m_h_pseudo = h_pseudo(m_xhat, state);
    updatePseudoJacobian(m_xhat, state);

    Eigen::Matrix2d temp;
    temp = m_R_pseudo + m_C_pseudo*m_P*m_C_pseudo.transpose();
    m_L_pseudo = m_P*m_C_pseudo.transpose()*temp.inverse();

    m_P = (I - m_L_pseudo*m_C_pseudo)*m_P*(I-m_L_pseudo*m_C_pseudo).transpose() + m_L_pseudo*m_R_pseudo*m_L_pseudo.transpose();
    m_xhat += m_L_pseudo * -m_h_pseudo;

    if (measurement->gps_n != m_gps_n_old || measurement->gps_e!=m_gps_e_old ||
        measurement->gps_Vg!=m_gps_Vg_old || measurement->gps_chi!=m_gps_chi_old)
    {
        m_h_gps = h_gps(m_xhat,state);
        updateGpsJacobian(m_xhat,state);
        m_y_gps << measurement->gps_n, measurement->gps_e, measurement->gps_Vg, measurement->gps_chi;
        Eigen::Matrix4d temp_gps;
        temp_gps = m_R_gps + m_C_gps*m_P*m_C_gps.transpose();
        m_L_gps = m_P*m_C_gps.transpose()*temp_gps.inverse();
        wrap(m_y_gps(3), m_h_gps(3));
        m_P = (I-m_L_gps*m_C_gps)*m_P*(I-m_L_gps*m_C_gps).transpose() + m_L_gps*m_R_gps*m_L_gps.transpose();

        m_gps_n_old = measurement->gps_n;
        m_gps_e_old = measurement->gps_e;
        m_gps_Vg_old = measurement->gps_Vg;
        m_gps_chi_old = measurement->gps_chi;
    }
}

PositionEKF::Vec7 PositionEKF::f(const PositionEKF::Vec7 &x, const uav_msgs::State &u)
{
    static double g{9.80665};

    double psi_d{u.q*sin(u.phi)/cos(u.theta) + u.r*cos(u.phi)/cos(u.theta)};
    double Vg{x(2)};
    double chi{x(3)};
    double wn{x(4)};
    double we{x(5)};
    double psi{x(6)};

    Vec7 _f;
    _f << Vg*cos(chi),
          Vg*sin(chi),
          ((u.Va*cos(psi)+wn)*(-u.Va*psi_d*sin(psi))+(u.Va*sin(psi)+we)*
                             (u.Va*psi_d*cos(psi))) / Vg,
          g/Vg * tan(u.phi)*cos(chi-psi),
          0.0,
          0.0,
          psi_d;

    return _f;
}

Eigen::Vector4d PositionEKF::h_gps(const PositionEKF::Vec7 &x, const uav_msgs::State &u)
{
    m_h_gps << x(0), x(1), x(2), x(3);
    return m_h_gps;
}

Eigen::Vector2d PositionEKF::h_pseudo(const PositionEKF::Vec7 &x, const uav_msgs::State &u)
{
    double Vg{x(2)};
    double chi{x(3)};
    double wn{x(4)};
    double we{x(5)};
    double psi{x(6)};
    m_h_pseudo << u.Va*cos(psi) + wn - Vg*cos(chi),
                  u.Va*sin(psi) + we - Vg*sin(chi);
    return m_h_pseudo;
}

void PositionEKF::updateJacobianA(const PositionEKF::Vec7 &x, const uav_msgs::State &state)
{
    Vec7 f_x, f_eps;
    f_x = f(x, state);

    Vec7 x_eps;
    for (int i{0}; i < m_xhat.rows(); i++)
    {
        x_eps = x;
        x_eps(i) += m_eps;
        m_A.col(i) = (f(x_eps,state) - f_x) / m_eps;
    }
}

void PositionEKF::updatePseudoJacobian(const PositionEKF::Vec7& x, const uav_msgs::State& state)
{
    Eigen::Vector2d h_pseudo_x;
    h_pseudo_x = h_pseudo(x,state);

    Vec7 x_eps;
    for (int i{0}; i < m_xhat.rows(); i++)
    {
        x_eps = x;
        x_eps(i) += m_eps;
        m_C_pseudo.col(i) = (h_pseudo(x_eps,state) - h_pseudo_x) / m_eps;
    }
}

void PositionEKF::updateGpsJacobian(const PositionEKF::Vec7& x, const uav_msgs::State& state)
{
    Eigen::Vector4d h_gps_x;
    h_gps_x = h_gps(x,state);

    Vec7 x_eps;
    for (int i{0}; i < m_xhat.rows(); i++)
    {
        x_eps = x;
        x_eps(i) += m_eps;
        m_C_gps.col(i) = (h_gps(x_eps,state) - h_gps_x) / m_eps;
    }
}
