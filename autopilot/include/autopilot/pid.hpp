#ifndef PID_HPP
#define PID_HPP

class PID
{
public:
    PID();
    virtual ~PID();
    double update(double y_ref, double y,double ts,bool wrap_flag=false);
    double update(double y_ref,double y,double y_dot,double ts,
                  bool wrap_flag=false);
    void setGains(double kp,double ki=0,double kd=0);
    void setSaturationLimits(double limit);
    void setSaturationLimits(double low_limit,double up_limit);
    void setSigma(const double sigma);
    void reset();

protected:
    double saturate(double u);
    void integrateError(double error);
    void differentiateY(double y);
    void integratorAntiWindup(double u_sat, double u_unsat);
    double rad(double degree);
    const double PI{3.14159};

private:
    double m_kp;
    double m_ki;
    double m_kd;
    double m_sigma;
    double m_up_limit;
    double m_low_limit;
    double m_ts;

    double m_error_int;
    double m_error_d1;
    double m_y_dot;
    double m_y_d1;
};

#endif // PID_HPP
