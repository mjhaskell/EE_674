#ifndef YAWDAMPER_HPP
#define YAWDAMPER_HPP

class YawDamper
{
public:
    YawDamper(double kr=0, double tau=0):
        m_kr{kr},
        m_tau{tau},
        m_z{0.0}
    {}

    virtual ~YawDamper()
    {}

    void setGains(double kr, double tau=0.05)
    {
        m_kr = kr;
        m_tau = tau;
    }

    double update(double r, double ts)
    {
        double dr{m_kr*(r - m_z/m_tau)};
        m_z += ts*(r - m_z/m_tau);

        return dr;
    }

    void reset()
    {
        m_z = 0.0;
    }

private:
    double m_kr;
    double m_tau;
    double m_z;
};

#endif /* YAWDAMPER_HPP */
