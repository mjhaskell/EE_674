#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "types.hpp"

namespace dyn
{

class Controller
{
public:
    Controller();
    virtual ~Controller();
    dyn::uVec calculateControl();
    dyn::RotMatrix getRotation() const;
    dyn::MatrixA getA() const;
    dyn::MatrixA getAd() const;
    dyn::MatrixB getBd() const;
    dyn::uVec getEquilibriumInputs() const;
    void setX(const dyn::xVec& x);
    void setConstRef(const dyn::xVec& ref);
    void setStateWeights(const dyn::xVec& weights,bool final=false);
    void setInputWeights(const dyn::uVec& weights);
    void setControlRate(double rate,bool hz=false);
    void setSlewRate(double slew_rate);
    void setDefaultTuningParams();

protected:
    void updateSolverParams();
    void initializeA();
    void initializeB();
    void setSolverX0();
    void setSolverConstRef();
    void setSolverStateWeights(bool final=false);
    void setSolverInputWeights();
    void setSolverA();
    void setSolverB();
    void setSolverEquilibriumInputs();
    void setSolverInputLimits();
    void linearizeAboutCurrentAttitude();
    void setSolverWeights();
    void setSolverSlewRate();
    void updateRotation();
    void updateA();
    void discretizeAB();
    void InitializeSolverParams();
    void setSolverSettings();
    dyn::xVec m_x;

private:
    double m_rate;
    double m_slew_rate;
    dyn::params_t m_p;
    dyn::xVec m_ref;
    dyn::xVec m_state_weights;
    dyn::uVec m_input_weights;
    dyn::MatrixA m_A;
    dyn::MatrixA m_Ad;
    dyn::MatrixB m_B;
    dyn::MatrixB m_Bd;
    dyn::RotMatrix m_R_b2i;
};

} //end namespace dyn
#endif // CONTROLLER_HPP
