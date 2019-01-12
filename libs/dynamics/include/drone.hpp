#ifndef DRONE_HPP
#define DRONE_HPP

#include "types.hpp"

namespace dyn
{

class Drone
{   
public:
    Drone();
    virtual ~Drone();
    void sendAttitudeCmds(const cmdVec& cmds);
    void sendMotorCmds(const uVec& inputs);
    xVec getStates() const;
    uVec getEquilibriumInputs() const;
    double getDt(const bool milli=true) const;
    double setDt(const double dt);
    void resetStates();

protected:
    void derivatives(const xVec& getStates,const uVec& inputs,xVec& k);

private:
    typedef struct
    {
        xVec k1;
        xVec k2;
        xVec k3;
        xVec k4;
        double dt;
    } rk4_t;
    typedef struct
    {
        double P;
        double I;
        double D;
    } pid_t;

    params_t m_p;
    xVec m_states;
    rk4_t m_rk4;
    Eigen::Matrix3d m_att_rot;
    pid_t m_roll;
    pid_t m_pitch;
    pid_t m_yaw_rate;
};

} // end namespace dyn
#endif // DRONE_HPP
