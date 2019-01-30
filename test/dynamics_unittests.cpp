#include "gtest/gtest.h"
#include "types.hpp"
#include "drone.hpp"
#include "controller.hpp"

template <typename Derived>
bool expectEigenNear(const Eigen::MatrixBase<Derived> &mat1,const Eigen::MatrixBase<Derived> &mat2,double delta)
{
    if (mat1.rows()!=mat2.rows() && mat1.cols()!=mat2.cols())
        return false;
    Derived diff{(mat1-mat2).cwiseAbs()};
    return diff.maxCoeff() < delta;
}

class DroneProtectedFunctionTester : public dyn::Drone
{
public:
    DroneProtectedFunctionTester() {}
    dyn::xVec findDerivatives()
    {
        dyn::xVec state_derivatives;
        this->derivatives(m_states,m_force_torque,state_derivatives);
        return state_derivatives;
    }

    dyn::xVec m_states;
    dyn::uVec m_force_torque;
    dyn::params_t m_p;
};

TEST(DroneProtectedFunctionTester,AskedToTakeDerivatesAtEquilibrium_ReturnsZero)
{
    DroneProtectedFunctionTester quadcopter;
    quadcopter.m_states.setZero(dyn::STATE_SIZE,1);
    quadcopter.m_force_torque << quadcopter.m_p.mass*quadcopter.m_p.grav,0,0,0;
    dyn::xVec actual_derivatives{quadcopter.findDerivatives()};

    dyn::xVec expected_derivatives;
    expected_derivatives.setZero(dyn::STATE_SIZE,1);

    EXPECT_EQ(expected_derivatives,actual_derivatives);
}

TEST(DroneProtectedFunctionTester,AskedToTakeDerivatesWithRandomStatesAndInputs_ReturnsCorrectDerivates)
{
    DroneProtectedFunctionTester quadcopter;
    quadcopter.m_states << -4.75,3.8,5,0.25,-0.2,1,1.2,-2.1,0,0.1,-0.1,0.3;
    quadcopter.m_force_torque << 15.9,1.2,-1.2,1.54;
    dyn::xVec actual_derivatives{quadcopter.findDerivatives()};

    dyn::xVec expected_derivatives;
    expected_derivatives << 2.403362,-0.022869,0.270789,
                            0.046093,-0.171112,0.271342,
                            1.278946,2.088654,4.105563,
                            22.666981,-22.616038,15.714286;

    EXPECT_TRUE(expectEigenNear(expected_derivatives,actual_derivatives,1e-6));
}

class DroneTestFixture : public dyn::Drone, public ::testing::Test
{
public:
    DroneTestFixture()
    {
        m_eq_offset.setZero(dyn::INPUT_SIZE,1);
        m_expected_states.setZero(dyn::STATE_SIZE,1);
    }

    void runSimulation(int steps)
    {
        for (int i{0}; i < steps; i++)
            m_quadcopter.sendMotorCmds(m_p.u_eq+m_eq_offset);

        m_actual_states = m_quadcopter.getStates();
    }

    dyn::Drone m_quadcopter;
    dyn::uVec m_eq_offset;
    dyn::xVec m_expected_states;
    dyn::xVec m_actual_states;

private:
    dyn::params_t m_p;
};

TEST_F(DroneTestFixture,GivenEquilibriumInputsWhenAtEquilibrium_DoesNotMove)
{
    int steps{1};
    this->runSimulation(steps);

    EXPECT_TRUE(expectEigenNear(m_expected_states,m_actual_states,1e-6));
}

TEST_F(DroneTestFixture,GivenAboveEquilibriumInputsWhenAtEquilibrium_MovesUp)
{
    int steps{500};
    double offset{.25};
    m_eq_offset << offset,offset,offset,offset;
    this->runSimulation(steps);

    m_expected_states(dyn::PZ) = 2.204978;
    m_expected_states(dyn::VZ) = -4.385592;

    EXPECT_TRUE(expectEigenNear(m_expected_states,m_actual_states,1e-6));
}

TEST_F(DroneTestFixture,GivenZeroInputsWhenAtEquilibrium_FallsDown)
{
    int steps{50};
    double offset{-0.55};
    m_eq_offset << offset,offset,offset,offset;
    this->runSimulation(steps);

    m_expected_states(dyn::PZ) = -0.048996;
    m_expected_states(dyn::VZ) = 0.979367;

    EXPECT_TRUE(expectEigenNear(m_expected_states,m_actual_states,1e-6));
}

TEST_F(DroneTestFixture,GivenInputsToYawPositiveWhenAtEquilibrium_YawsPositive)
{
    int steps{500};
    double off{0.1};
    m_eq_offset << -off,off,-off,off;
    this->runSimulation(steps);

    m_expected_states(dyn::RZ) = 0.408163;
    m_expected_states(dyn::WZ) = 0.816327;

    EXPECT_TRUE(expectEigenNear(m_expected_states,m_actual_states,1e-6));
}

TEST_F(DroneTestFixture,GivenInputsToYawNegativeWhenAtEquilibrium_YawsNegative)
{
    int steps{500};
    double off{0.1};
    m_eq_offset << off,-off,off,-off;
    this->runSimulation(steps);

    m_expected_states(dyn::RZ) = -0.408163;
    m_expected_states(dyn::WZ) = -0.816327;

    EXPECT_TRUE(expectEigenNear(m_expected_states,m_actual_states,1e-6));
}

TEST_F(DroneTestFixture,GivenInputsToRollPositiveWhenAtEquilibrium_RollsPositive)
{
    int steps{100};
    double off{0.1};
    m_eq_offset << 0,-off,0,off;
    this->runSimulation(steps);

    m_expected_states(dyn::PY) = 0.009859;
    m_expected_states(dyn::VY) = 0.192859;
    m_expected_states(dyn::PZ) = -0.000598;
    m_expected_states(dyn::VZ) = -0.041511;
    m_expected_states(dyn::RX) = 0.302882;
    m_expected_states(dyn::WX) = 3.028816;

    EXPECT_TRUE(expectEigenNear(m_expected_states,m_actual_states,1e-6));
}

TEST_F(DroneTestFixture,GivenInputsToRollNegativeWhenAtEquilibrium_RollsNegative)
{
    int steps{100};
    double off{0.1};
    m_eq_offset << 0,off,0,-off;
    this->runSimulation(steps);

    m_expected_states(dyn::PY) = -0.009859;
    m_expected_states(dyn::VY) = -0.192859;
    m_expected_states(dyn::PZ) = -0.000598;
    m_expected_states(dyn::VZ) = -0.041511;
    m_expected_states(dyn::RX) = -0.302882;
    m_expected_states(dyn::WX) = -3.028816;

    EXPECT_TRUE(expectEigenNear(m_expected_states,m_actual_states,1e-6));
}

TEST_F(DroneTestFixture,GivenInputsToPitchPositiveWhenAtEquilibrium_PitchesPositive)
{
    int steps{100};
    double off{0.1};
    m_eq_offset << off,0,-off,0;
    this->runSimulation(steps);

    m_expected_states(dyn::PX) = -0.009859;
    m_expected_states(dyn::VX) = -0.192859;
    m_expected_states(dyn::PZ) = -0.000598;
    m_expected_states(dyn::VZ) = -0.041511;
    m_expected_states(dyn::RY) = 0.302882;
    m_expected_states(dyn::WY) = 3.028816;

    EXPECT_TRUE(expectEigenNear(m_expected_states,m_actual_states,1e-6));
}

TEST_F(DroneTestFixture,GivenInputsToPitchNegativeWhenAtEquilibrium_PitchesNegative)
{
    int steps{100};
    double off{0.1};
    m_eq_offset << -off,0,off,0;
    this->runSimulation(steps);

    m_expected_states(dyn::PX) = 0.009859;
    m_expected_states(dyn::VX) = 0.192859;
    m_expected_states(dyn::PZ) = -0.000598;
    m_expected_states(dyn::VZ) = -0.041511;
    m_expected_states(dyn::RY) = -0.302882;
    m_expected_states(dyn::WY) = -3.028816;

    EXPECT_TRUE(expectEigenNear(m_expected_states,m_actual_states,1e-6));
}

class ControllerTestFixture : public dyn::Controller, public ::testing::Test
{
public:
    ControllerTestFixture()
    {
        double roll{3.14/4};
        double pitch{3.14/8};
        double yaw{-3.14/6};
        this->setAttitude(roll,pitch,yaw);
    }

    void setAttitude(double roll,double pitch,double yaw)
    {
        this->m_x.block(dyn::RX,0,3,1) << roll, pitch, yaw;
    }
};

TEST_F(ControllerTestFixture,AskedToUpdateRotation_UpdatesCorrectly)
{
    this->updateRotation();

    dyn::RotMatrix expected_rotation;
    expected_rotation << 0.800292,0.587706,-0.118889,
                        -0.461765,0.477592,-0.747448,
                        -0.382499,0.653075,0.653595;

    dyn::RotMatrix actual_rotation{this->getRotation()};

    EXPECT_TRUE(expectEigenNear(expected_rotation,actual_rotation,1e-6));
}

TEST_F(ControllerTestFixture,AskedToUpdateA_UpdatesCorrectly)
{
    this->updateRotation();
    this->updateA();

    dyn::MatrixA expected_A;
    expected_A.setZero(dyn::STATE_SIZE,dyn::STATE_SIZE);
    expected_A.block(dyn::PX,dyn::VX,3,3) << 0.800292,0.587706,-0.118889,
                                            -0.461765,0.477592,-0.747448,
                                             0.382499,-0.653075,-0.653595;
    Eigen::Matrix3d identity;
    identity << 1,0,0,  0,1,0,  0,0,1;
    expected_A.block(dyn::RX,dyn::WX,3,3) = identity;
    expected_A.block(dyn::VX,dyn::VX,3,3) = identity*-0.033333;
    expected_A.block(dyn::VX,dyn::RX,3,3) << 0,-9.064005,0,
                                             6.411771,-2.652234,0,
                                             -6.406667,-2.654347,0;

    dyn::MatrixA actual_A{this->getA()};

    EXPECT_TRUE(expectEigenNear(expected_A,actual_A,1e-6));
}

TEST_F(ControllerTestFixture,AskedToDiscretizeAandB_DiscretizesCorrectly)
{
    this->updateRotation();
    this->updateA();
    this->discretizeAB();

    dyn::MatrixA expected_Ad;
    expected_Ad << 1,0,0,0.000226,-0.000425,0,0.008002,0.005876,-0.001189,0.000001,-0.000001,0,
                   0,1,0,0.000392,0.000245,0,-0.004617,0.004775,-0.007473,0.000001,0.000001,0,
                   0,0,1,0,0,0,0.003824,-0.006530,-0.006535,0,0,0,
                   0,0,0,1,0,0,0,0,0,0.01,0,0,
                   0,0,0,0,1,0,0,0,0,0,0.01,0,
                   0,0,0,0,0,1,0,0,0,0,0,0.01,
                   0,0,0,0,-0.090625,0,0.999667,0,0,0,-0.000453,0,
                   0,0,0,0.064107,-0.026518,0,0,0.999667,0,0.000321,-0.000133,0,
                   0,0,0,-0.064056,-0.026539,0,0,0,0.999667,-0.000320,-0.000133,0,
                   0,0,0,0,0,0,0,0,0,1,0,0,
                   0,0,0,0,0,0,0,0,0,0,1,0,
                   0,0,0,0,0,0,0,0,0,0,0,1;

    dyn::MatrixB expected_Bd;
    expected_Bd << 0.000026,0.000026,0.000027,0.000027,
                   0.000167,0.000166,0.000166,0.000166,
                   0.000146,0.000146,0.000146,0.000146,
                   0,-0.003786,0,0.003786,
                   0.003786,0,-0.003786,0,
                  -0.000102,0.000102,-0.000102,0.000102,
                  -0.000114,0,0.000114,0,
                  -0.000033,-0.000081,0.000033,0.000081,
                  -0.044617,-0.044503,-0.044550,-0.044664,
                   0,-0.757204,0,0.757204,
                   0.757204,0,-0.757204,0,
                  -0.020408,0.020408,-0.020408,0.020408;

    dyn::MatrixA actual_Ad{this->getAd()};
    dyn::MatrixB actual_Bd{this->getBd()};

    EXPECT_TRUE(expectEigenNear(expected_Ad,actual_Ad,1e-6));
    EXPECT_TRUE(expectEigenNear(expected_Bd,actual_Bd,1e-6));
}

class ControllerTester : public ::testing::Test
{
public:
    ControllerTester() {}
    void runController()
    {
        m_mpc.setX(m_current_states);
        m_mpc.setConstRef(m_ref);
        m_actual_input = m_mpc.calculateControl();
    }

    dyn::Controller m_mpc;
    dyn::xVec m_current_states;
    dyn::xVec m_ref;
    dyn::uVec m_actual_input;
};

TEST_F(ControllerTester,AskedToCalculateControlWhenAtRefCmd_ReturnsEquilibriumCommands)
{
    m_current_states.setZero(dyn::STATE_SIZE,1);
    m_ref.setZero(dyn::STATE_SIZE,1);
    runController();

    double eq{0.55};
    dyn::uVec expected_input{eq,eq,eq,eq};

    EXPECT_TRUE(expectEigenNear(expected_input,m_actual_input,1e-4));
}

TEST_F(ControllerTester,AskedToCalculateControlWithPositiveHeightRefCmd_ReturnsFullThrottleCommands)
{
    m_current_states.setZero(dyn::STATE_SIZE,1);
    m_ref.setZero(dyn::STATE_SIZE,1);
    m_ref(dyn::PZ,0) = 5;
    runController();

    double motor_cmd{1.0};
    dyn::uVec expected_input{motor_cmd,motor_cmd,motor_cmd,motor_cmd};

    EXPECT_TRUE(expectEigenNear(expected_input,m_actual_input,1e-4));
}

TEST_F(ControllerTester,AskedToCalculateControlWithNegativeHeightRefCmd_ReturnsZeroThrottleCommands)
{
    m_current_states.setZero(dyn::STATE_SIZE,1);
    m_ref.setZero(dyn::STATE_SIZE,1);
    m_ref(dyn::PZ,0) = -5;
    runController();

    double motor_cmd{0};
    dyn::uVec expected_input{motor_cmd,motor_cmd,motor_cmd,motor_cmd};

    EXPECT_TRUE(expectEigenNear(expected_input,m_actual_input,1e-4));
}
