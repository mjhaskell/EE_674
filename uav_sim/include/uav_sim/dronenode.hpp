#ifndef DRONENODE_HPP
#define DRONENODE_HPP

#include "dynamics/fixedwing.hpp"
#include <ros/ros.h>
#include <QThread>
#include "uav_msgs/State.h"
#include "uav_msgs/Delta.h"

namespace uav
{

class DroneNode : public QThread
{
    Q_OBJECT
public:
    DroneNode(int argc, char** argv);
    virtual ~DroneNode();
    bool rosIsConnected() const;
    bool init();
    bool init(const std::string& master_url,const std::string& host_url,bool use_ip=true);
    void setUseRos(const bool use_ros);
    bool useRos() const;
    void run();
    bool startNode();
    void stopRunning();
    void resetNode();
    std::string getStateTopics();
    void setupRosComms(const std::string topic="/states/truth");

signals:
    void feedbackStates(const dyn::State* states);
    void statesChanged(fixedwing::State* state);
    void rosLostConnection();

public slots:
    void updateInputs(const fixedwing::Input* inputs);
    void updateWind(const Eigen::Vector3d* wind);

protected:
    void runRosNode();
    void runNode();
    void updateDynamics();
    void deltaCallback(const uav_msgs::DeltaConstPtr& msg);
    void resetState();

private:
    int m_argc;
    char** m_argv;
    bool m_use_ros;
    std::string m_node_name{"drone_node"};
    fixedwing::Params m_p;
    FixedWing m_drone;
    double m_rate;
    fixedwing::Input m_inputs;
    fixedwing::State m_states;
    uav_msgs::State m_state_msg;
//    uav_msgs::State m_sub_odom;
//    ros::Subscriber m_state_sub;
    ros::Subscriber m_delta_sub;
    ros::Publisher m_state_pub;
    bool m_ros_is_connected;
    bool m_is_running;
};

} // end namespace quad
#endif // DRONENODE_HPP
