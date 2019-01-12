#ifndef DRONENODE_HPP
#define DRONENODE_HPP

#include <ros/ros.h>
#include <QThread>
#include "nav_msgs/Odometry.h"
#include "drone.hpp"

namespace quad
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
    std::string getOdometryTopics();
    void setupRosComms(const std::string topic="/sim_states");

signals:
    void feedbackStates(const dyn::xVec* states);
    void statesChanged(nav_msgs::Odometry* odom);
    void rosLostConnection();

public slots:
    void updateInputs(const dyn::uVec* inputs);

protected:
    void runRosNode();
    void runNode();
    void updateDynamics();
    void stateCallback(const nav_msgs::OdometryConstPtr& msg);
    void resetOdometry();

private:
    int m_argc;
    char** m_argv;
    bool m_use_ros;
    std::string m_node_name{"drone_node"};
    dyn::Drone m_drone;
    double m_rate;
    dyn::uVec m_inputs;
    dyn::xVec m_states;
    nav_msgs::Odometry m_odom;
    nav_msgs::Odometry m_sub_odom;
    ros::Subscriber m_state_sub;
    ros::Subscriber m_odom_sub;
    ros::Publisher m_state_pub;
    bool m_ros_is_connected;
    bool m_is_running;
};

} // end namespace quad
#endif // DRONENODE_HPP
