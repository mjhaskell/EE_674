#ifndef DRONENODE_HPP
#define DRONENODE_HPP

#include "dynamics/fixedwing.hpp"
#include <ros/ros.h>
#include <QThread>
#include "uav_msgs/State.h"
#include "uav_msgs/Delta.h"
#include "uav_msgs/Sensors.h"
#include "uav_msgs/Map.h"

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
    void setupRosComms(const std::string topic="/command/delta");
    void updateWind(const Eigen::Vector3d& wind);
    void setUseGust(const bool use);
    void sendDefaultWaypoints(int mode);

signals:
    void feedbackStates(const dyn::State* states);
    void statesChanged(fixedwing::State* state);
    void rosLostConnection();

protected:
    void runRosNode();
    void runNode();
    void updateDynamics();
    void deltaCallback(const uav_msgs::DeltaConstPtr& msg);
    void resetState();
    void updateStateMsg();
    void updateSensorsMsg();

private:
    int m_argc;
    char** m_argv;
    bool m_use_ros;
    std::string m_node_name{"uav_sim"};
    fixedwing::Params m_p;
    FixedWing m_drone;
    double m_rate;
    fixedwing::Input m_inputs;
    fixedwing::State m_states;
    fixedwing::Sensors m_sensors;
    uav_msgs::Sensors m_sensors_msg;
    uav_msgs::State m_state_msg;
    ros::Subscriber m_delta_sub;
    ros::Publisher m_state_pub;
    ros::Publisher m_status_pub;
    ros::Publisher m_sensors_pub;
    ros::Publisher m_map_pub;
    bool m_ros_is_connected;
    bool m_is_running;
    uav_msgs::Map m_map_msg;
};

} // end namespace quad
#endif // DRONENODE_HPP
