#include "controller.hpp"
#include <unsupported/Eigen/MatrixFunctions>
extern "C"
{
#include "solver.h"
}
// These global variables are required for 3rd party code
Vars vars;
Params params;
Workspace work;
Settings settings;
// End of global variables for 3rd party code

namespace dyn
{

Controller::Controller()
{
    m_p.setMixer();
    m_x.setZero(dyn::STATE_SIZE,1);
    m_ref.setZero(dyn::STATE_SIZE,1);
    this->setDefaultTuningParams();
    this->initializeA();
    this->initializeB();
    this->linearizeAboutCurrentAttitude();
    this->InitializeSolverParams();
    this->setSolverSettings();
    set_defaults();
    setup_indexing();
}

Controller::~Controller()
{
}

dyn::uVec Controller::calculateControl()
{
    this->updateSolverParams();

    int num_iters;
    num_iters = solve();

    dyn::uVec inputs{vars.u_0[0],vars.u_0[1],vars.u_0[2],vars.u_0[3]};
    return inputs;
}

dyn::RotMatrix Controller::getRotation() const
{
    return m_R_b2i;
}

dyn::MatrixA Controller::getA() const
{
    return m_A;
}

dyn::MatrixA Controller::getAd() const
{
    return m_Ad;
}

dyn::MatrixB Controller::getBd() const
{
    return m_Bd;
}

uVec Controller::getEquilibriumInputs() const
{
    return m_p.u_eq;
}

void Controller::setX(const xVec& x)
{
    m_x = x;
}

void Controller::setConstRef(const dyn::xVec &ref)
{
    m_ref = ref;
}

void Controller::initializeA()
{
    m_A.setZero(dyn::STATE_SIZE,dyn::STATE_SIZE);
    m_A(0,6) = 1;
    m_A(1,7) = 1;
    m_A(2,8) = -1;
    m_A(3,9) = 1;
    m_A(4,10) = 1;
    m_A(5,11) = 1;
    m_A(6,4) = -m_p.grav;
    m_A(7,3) = m_p.grav;
    m_A(6,6) = -m_p.mu/m_p.mass;
    m_A(7,7) = -m_p.mu/m_p.mass;
    m_A(8,8) = -m_p.mu/m_p.mass;
}

void Controller::initializeB()
{
    m_B.setZero(dyn::STATE_SIZE,dyn::INPUT_SIZE);
    m_B(8,0) = -1/m_p.mass;
    m_B(9,1) = 1/m_p.inertia_x;
    m_B(10,2) = 1/m_p.inertia_y;
    m_B(11,3) = 1/m_p.inertia_z;
    m_B.block(8,0,4,4) *= m_p.mixer;
}

void Controller::setSolverX0()
{
    for (int i{0};i < dyn::STATE_SIZE;i++)
        params.x_0[i] = m_x[i];
}

void Controller::setSolverConstRef()
{
    for (int i{0};i < dyn::STATE_SIZE;i++)
    {
        params.x_des_0[i] = m_ref[i];
        params.x_des_1[i] = m_ref[i];
        params.x_des_2[i] = m_ref[i];
        params.x_des_3[i] = m_ref[i];
        params.x_des_4[i] = m_ref[i];
        params.x_des_5[i] = m_ref[i];
        params.x_des_6[i] = m_ref[i];
        params.x_des_7[i] = m_ref[i];
        params.x_des_8[i] = m_ref[i];
        params.x_des_9[i] = m_ref[i];
        params.x_des_10[i] = m_ref[i];
        params.x_des_11[i] = m_ref[i];
    }
}

void Controller::setSolverStateWeights(bool final)
{
    for (int i{0};i < dyn::STATE_SIZE;i++)
    {
        params.Wy[i] = m_state_weights[i];
        if (final)
            params.Wy_final[i] = m_state_weights[i];
        else
            params.Wy_final[i] = 0.0;
    }
}

void Controller::setSolverInputWeights()
{
    for (int i{0};i < dyn::INPUT_SIZE;i++)
        params.Wu[i] = m_input_weights[i];
}

void Controller::setSolverA()
{
    for (int j{0};j < m_Ad.cols();j++)
        for (int i{0};i < m_Ad.rows();i++)
            params.A[i+j*m_Ad.rows()] = m_Ad(i,j);
}

void Controller::setSolverB()
{
    for (int j{0};j < m_Bd.cols();j++)
        for (int i{0};i < m_Bd.rows();i++)
            params.B[i+j*m_Bd.rows()] = m_Bd(i,j);
}

void Controller::setStateWeights(const dyn::xVec& weights,bool final)
{
    m_state_weights = weights;
    setSolverStateWeights(final);
}

void Controller::setInputWeights(const dyn::uVec& weights)
{
    m_input_weights = weights;
    setSolverInputWeights();
}

void Controller::setControlRate(double rate, bool hz)
{
    if (hz)
        m_rate = 1.0/rate;
    else
        m_rate = rate;
}

void Controller::setSlewRate(double slew_rate)
{
    m_slew_rate = slew_rate;
}

void Controller::setDefaultTuningParams()
{
    m_rate = 0.01;
    m_slew_rate = 0.005;
    setSolverSlewRate();
    m_state_weights << 10,10,100, 2.5,2.5,2, .8,.8,1, 0.1,0.1,0.1;
    m_input_weights.setZero(dyn::INPUT_SIZE,1);
    setSolverWeights();
}

void Controller::updateSolverParams()
{
    this->setSolverX0();
    this->setSolverConstRef();
    this->linearizeAboutCurrentAttitude();
}

void Controller::setSolverEquilibriumInputs()
{
    for (int i{0};i < dyn::INPUT_SIZE;i++)
    {
        params.u_des_0[i] = m_p.u_eq[i];
        params.u_des_1[i] = m_p.u_eq[i];
        params.u_des_2[i] = m_p.u_eq[i];
        params.u_des_3[i] = m_p.u_eq[i];
        params.u_des_4[i] = m_p.u_eq[i];
        params.u_des_5[i] = m_p.u_eq[i];
        params.u_des_6[i] = m_p.u_eq[i];
        params.u_des_7[i] = m_p.u_eq[i];
        params.u_des_8[i] = m_p.u_eq[i];
        params.u_des_9[i] = m_p.u_eq[i];
        params.u_des_10[i] = m_p.u_eq[i];
    }
}

void Controller::setSolverInputLimits()
{
    params.u_min[0] = 0.0;
    params.u_max[0] = 1.0;
}

void Controller::setSolverSlewRate()
{
    params.S[0] = m_slew_rate;
}

void Controller::linearizeAboutCurrentAttitude()
{
    this->updateRotation();
    this->updateA();
    this->discretizeAB();
    this->setSolverA();
    this->setSolverB();
}

void Controller::setSolverWeights()
{
    setSolverStateWeights();
    setSolverInputWeights();
}

void Controller::updateRotation()
{
    double roll{m_x(dyn::RX)};
    double pitch{m_x(dyn::RY)};
    double yaw{m_x(dyn::RZ)};
    m_R_b2i(0,0) = cos(pitch)*cos(yaw);
    m_R_b2i(0,1) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
    m_R_b2i(0,2) = cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);
    m_R_b2i(1,0) = cos(pitch)*sin(yaw);
    m_R_b2i(1,1) = sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw);
    m_R_b2i(1,2) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw);
    m_R_b2i(2,0) = -sin(pitch);
    m_R_b2i(2,1) = sin(roll)*cos(pitch);
    m_R_b2i(2,2) = cos(roll)*cos(pitch);
}

void Controller::updateA()
{
    Eigen::Matrix3d down_to_height;
    down_to_height << 1,0,0, 0,1,0, 0,0,-1;
    m_A.block(dyn::PX,dyn::VX,3,3) = down_to_height*m_R_b2i;
    m_A.block(dyn::VX,dyn::RX,3,3) << 0,-m_p.grav*cos(m_x(dyn::RY)),0,
                                      m_p.grav*cos(m_x(dyn::RY))*cos(m_x(dyn::RX)),-m_p.grav*sin(m_x(dyn::RY))*sin(m_x(dyn::RX)),0,
                                     -m_p.grav*cos(m_x(dyn::RY))*sin(m_x(dyn::RX)),-m_p.grav*sin(m_x(dyn::RY))*cos(m_x(dyn::RX)),0;
}

void Controller::discretizeAB()
{
    m_Ad = (m_A*m_rate).exp();
    m_Bd = m_rate*(Eigen::MatrixXd::Identity(dyn::STATE_SIZE,dyn::STATE_SIZE)+m_A*m_rate/2+m_A*m_A*m_rate*m_rate/6+m_A*m_A*m_A*m_rate*m_rate*m_rate/24)*m_B;
}

void Controller::InitializeSolverParams()
{
    this->setSolverX0();
    this->setSolverConstRef();
    this->linearizeAboutCurrentAttitude();
    this->setSolverWeights();
    this->setSolverInputLimits();
    this->setSolverSlewRate();
    this->setSolverEquilibriumInputs();
}

void Controller::setSolverSettings()
{
    settings.verbose = 0;
    settings.max_iters = 25;
    settings.eps = 1e-6;
    settings.resid_tol = 1e-4;
}

} //end namespace dyn
