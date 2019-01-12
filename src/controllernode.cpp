#include "controllernode.hpp"
#include <chrono>

namespace quad
{

ControllerNode::ControllerNode() :
    m_rate{0.01},
    m_is_running{false}
{
    m_states.setZero(dyn::STATE_SIZE,1);
    m_ref << 0,0,3, 0,0,0, 0,0,0, 0,0,0;
    m_cmds.setZero(dyn::INPUT_SIZE,1);
    m_odom.pose.pose.position.x = 0;
    m_odom.pose.pose.position.y = 0;
    m_odom.pose.pose.position.z = 0;
    m_odom.pose.pose.orientation.w = 1;
    m_odom.pose.pose.orientation.x = 0;
    m_odom.pose.pose.orientation.y = 0;
    m_odom.pose.pose.orientation.z = 0;
}

quad::ControllerNode::~ControllerNode()
{
    wait();
}

void ControllerNode::run()
{
    m_is_running = true;

    while (m_is_running)
    {
        auto t_start{std::chrono::high_resolution_clock::now()};
        m_controller.setX(m_states);
        m_controller.setConstRef(m_ref);
        m_cmds = m_controller.calculateControl();
        emit sendInputs(&m_cmds);
        while(std::chrono::duration<double,std::milli>(std::chrono::high_resolution_clock::now()-t_start).count() < m_rate) {}
    }
}

void ControllerNode::startNode()
{
    start();
}

void ControllerNode::resetNode()
{
    m_states.setZero(dyn::STATE_SIZE,1);
    m_cmds = m_controller.getEquilibriumInputs();
    emit sendInputs(&m_cmds);
}

void ControllerNode::stopRunning()
{
    m_is_running = false;
}

void ControllerNode::setRefCmd(const Eigen::Vector4d &ref)
{
    m_ref.block(dyn::PX,0,3,1) = ref.segment<3>(dyn::PX);
    m_ref(dyn::RZ,0) = ref(3);
}

void ControllerNode::setWeights(const dyn::xVec &state_weights, const dyn::uVec &input_weights)
{
    m_controller.setStateWeights(state_weights);
    m_controller.setInputWeights(input_weights);
}

void ControllerNode::setRates(double ts, double slew)
{
    m_controller.setControlRate(ts);
    m_controller.setSlewRate(slew);
}

void ControllerNode::updateStates(const dyn::xVec* states)
{
    m_states = *states;
}

} // end namespace quad
