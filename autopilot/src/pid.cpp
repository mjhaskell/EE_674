#include "autopilot/pid.hpp"

PID::PID() :
    m_kp{0},
    m_ki{0},
    m_kd{0},
    m_sigma{0.05},
    m_up_limit{1.0},
    m_low_limit{-1.0},
    m_ts{0.01},
    m_error_int{0},
    m_error_d1{0},
    m_y_dot{0},
    m_y_d1{0}
{
}

PID::~PID()
{
}

double PID::update(double y_ref,double y,double ts,bool wrap_flag)
{
    m_ts = ts;
    double error{y_ref - y};
    if (wrap_flag)
    {
        while (error > PI)
            error -= 2*PI;
        while (error < -PI)
            error += 2*PI;
    }
    this->integrateError(error);
    this->differentiateY(y);

    double u_unsat{m_kp*error + m_ki*m_error_int - m_kd*m_y_dot};
    double u_sat{this->saturate(u_unsat)};

    if (m_ki != 0)
        this->integratorAntiWindup(u_sat,u_unsat);

    return u_sat;
}

double PID::update(double y_ref,double y,double y_dot,double ts,bool wrap_flag)
{
    m_ts = ts;
    double error{y_ref - y};
    if (wrap_flag)
    {
        while (error > PI)
            error -= 2*PI;
        while (error < -PI)
            error += 2*PI;
    }
    this->integrateError(error);

    double u_unsat{m_kp*error + m_ki*m_error_int - m_kd*y_dot};
    double u_sat{this->saturate(u_unsat)};

    if (m_ki != 0)
        this->integratorAntiWindup(u_sat,u_unsat);

    return u_sat;
}

void PID::setGains(double kp,double ki,double kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void PID::setSaturationLimits(double limit)
{
    m_up_limit = limit;
    m_low_limit = -limit;
}

void PID::setSaturationLimits(double low_limit,double up_limit)
{
    m_up_limit = up_limit;
    m_low_limit = low_limit;
}

void PID::setSigma(const double sigma)
{
    m_sigma = sigma;
}

void PID::reset()
{
    m_error_int = 0.0;
    m_error_d1 = 0.0;
    m_y_dot = 0.0;
    m_y_d1 = 0.0;
}

double PID::saturate(double u)
{
    if (u > m_up_limit)
        u = m_up_limit;
    else if (u < m_low_limit)
        u = m_low_limit;
    return u;
}

void PID::integrateError(double error)
{
    m_error_int += m_ts/2.0 * (error + m_error_d1);
    m_error_d1 = error;
}

void PID::differentiateY(double y)
{
    double a1{(2*m_sigma - m_ts) / (2*m_sigma + m_ts)};
    double a2{2.0 / (2*m_sigma + m_ts)};

    m_y_dot = a1*m_y_dot + a2*(y - m_y_d1);
    m_y_d1 = y;
}

void PID::integratorAntiWindup(double u_sat, double u_unsat)
{
    m_error_int += m_ts/m_ki * (u_sat - u_unsat);
}
