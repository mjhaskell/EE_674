#include "uav_sim/dronenode.hpp"
#include <ros/ros.h>
#include <string>
#include "uav_msgs/State.h"
#include "uav_msgs/Status.h"
#include "geometry/quat.h"
#include <chrono>

namespace uav
{

DroneNode::DroneNode(int argc, char** argv) :
    m_argc{argc},
    m_argv{argv},
    m_use_ros{false},
    m_rate{m_drone.getDt()},
    m_states{m_drone.getFixedwingStates()},
    m_ros_is_connected{false},
    m_is_running{false}
{
    m_inputs = m_drone.getEquilibriumInputs();
}

DroneNode::~DroneNode()
{
    if(ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }
    wait();
}

bool DroneNode::rosIsConnected() const
{
    return m_ros_is_connected;
}

bool DroneNode::init()
{
    ros::init(m_argc,m_argv,m_node_name);
    if (!ros::master::check())
        return false;
    if (m_use_ros)
        this->setupRosComms();
    return m_ros_is_connected = true;
}

bool DroneNode::init(const std::string& master_url, const std::string& host_url,bool use_ip)
{
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    if (use_ip)
        remappings["__ip"] = host_url;
    else
        remappings["__hostname"] = host_url;
    ros::init(remappings,m_node_name);
    if (!ros::master::check())
        return false;
    if (m_use_ros)
        this->setupRosComms();
    return m_ros_is_connected = true;
}

void DroneNode::setUseRos(const bool use_ros)
{
    m_use_ros = use_ros;
}

bool DroneNode::useRos() const
{
    return m_use_ros;
}

void DroneNode::run()
{
    m_is_running = true;
    if (m_use_ros)
        this->runRosNode();
    else
        this->runNode();
}

bool DroneNode::startNode()
{
    if (m_use_ros)
    {
        if (!ros::master::check())
            return false;
        uav_msgs::Status status;
        status.reset = false;
        status.is_flying = true;
        m_status_pub.publish(status);
    }
    start();
    return true;
}

void DroneNode::stopRunning()
{
    m_is_running = false;
}

void DroneNode::resetNode()
{
    if (m_use_ros)
    {
        uav_msgs::Status status;
        status.reset = true;
        status.is_flying = false;
        m_status_pub.publish(status);
    }

    m_drone.resetStates();
    m_states = m_drone.getFixedwingStates();
    m_inputs = m_drone.getEquilibriumInputs();
    this->resetState();
    emit statesChanged(&m_states);
}

std::string DroneNode::getStateTopics()
{
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    std::stringstream topics;

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin();it != master_topics.end();it++)
    {
        const ros::master::TopicInfo &info{*it};
        if (info.datatype == "uav_msgs/Delta")
            topics << info.name << ",";
    }
    return topics.str();
}

void DroneNode::updateWind(const Eigen::Vector3d& wind)
{
    m_drone.setWindSS(wind);
}

void DroneNode::setUseGust(const bool use)
{
    m_drone.setUseGust(use);
}

void DroneNode::runRosNode()
{
    ros::Rate publish_rate{500};
    while (ros::ok() && ros::master::check() && m_is_running)
    {
        this->updateDynamics();

        updateSensorsMsg();
        m_sensors_pub.publish(m_sensors_msg);

        updateStateMsg();
        m_state_pub.publish(m_state_msg);

        emit statesChanged(&m_states);

        ros::spinOnce();
        publish_rate.sleep();
    }
    if (m_is_running)
    {
        m_ros_is_connected = false;
        emit rosLostConnection();
    }
}

void DroneNode::runNode()
{
    while (m_is_running)
    {
        auto t_start{std::chrono::high_resolution_clock::now()};
        this->updateDynamics();
//        emit feedbackStates(&m_states);
        emit statesChanged(&m_states);
        while(std::chrono::duration<double,std::milli>(std::chrono::high_resolution_clock::now()-t_start).count() < m_rate) {}
    }
}

void DroneNode::setupRosComms(const std::string topic)
{
    ros::start();
    ros::NodeHandle nh;
    uint32_t queue_size{50};
    m_delta_sub = nh.subscribe(topic, queue_size, &DroneNode::deltaCallback, this);
    m_state_pub = nh.advertise<uav_msgs::State>("states/truth", queue_size);
    m_status_pub = nh.advertise<uav_msgs::Status>("sim/status", queue_size);
    m_sensors_pub = nh.advertise<uav_msgs::Sensors>("/sensors", queue_size);
}

void DroneNode::updateStateMsg()
{
    m_state_msg.pn = m_states.dyn.p(0);
    m_state_msg.pe = m_states.dyn.p(1);
    m_state_msg.h = -m_states.dyn.p(2);

    Eigen::Vector3d euler{m_states.dyn.q.euler()};

    m_state_msg.phi = euler(0);
    m_state_msg.theta = euler(1);
    m_state_msg.psi = euler(2);

    m_state_msg.p = m_states.dyn.w(0);
    m_state_msg.q = m_states.dyn.w(1);
    m_state_msg.r = m_states.dyn.w(2);

    m_state_msg.Va = m_states.Va;
    m_state_msg.alpha = m_states.alpha;
    m_state_msg.beta = m_states.beta;
    m_state_msg.Vg = m_states.Vg;
    m_state_msg.gamma = m_states.gamma;
    m_state_msg.chi = m_states.chi;

    m_state_msg.wn = m_states.wn;
    m_state_msg.we = m_states.we;
    m_state_msg.bx = m_states.bx;
    m_state_msg.by = m_states.by;
    m_state_msg.bz = m_states.bz;

    if (m_use_ros)
        m_state_msg.header.stamp = ros::Time::now();
}

void DroneNode::updateSensorsMsg()
{
    m_sensors_msg.gps_n = m_sensors.gps_n;
    m_sensors_msg.gps_e = m_sensors.gps_e;
    m_sensors_msg.gps_h = m_sensors.gps_h;
    m_sensors_msg.gps_Vg = m_sensors.gps_Vg;
    m_sensors_msg.gps_chi = m_sensors.gps_chi;
    m_sensors_msg.gyro_x = m_sensors.gyro_x;
    m_sensors_msg.gyro_y = m_sensors.gyro_y;
    m_sensors_msg.gyro_z = m_sensors.gyro_z;
    m_sensors_msg.accel_x = m_sensors.accel_x;
    m_sensors_msg.accel_y = m_sensors.accel_y;
    m_sensors_msg.accel_z = m_sensors.accel_z;
    m_sensors_msg.static_pressure = m_sensors.static_pressure;
    m_sensors_msg.diff_pressure = m_sensors.diff_pressure;
    if (m_use_ros)
        m_sensors_msg.header.stamp = ros::Time::now();
}

void DroneNode::updateDynamics()
{
    m_drone.sendDeltas(m_inputs);
    m_states = m_drone.getFixedwingStates();
    m_sensors = m_drone.getSensorData();
}

void DroneNode::deltaCallback(const uav_msgs::DeltaConstPtr& msg)
{
    m_inputs.dt = msg->dt;
    m_inputs.da = msg->da;
    m_inputs.de = msg->de;
    m_inputs.dr = msg->dr;
}

void DroneNode::resetState()
{
    m_state_msg.pn = 0;
    m_state_msg.pe = 0;
    m_state_msg.h = 100;

    m_state_msg.phi = 0;
    m_state_msg.theta = 0;
    m_state_msg.psi = 0;

    m_state_msg.p = 0;
    m_state_msg.q = 0;
    m_state_msg.r = 0;

    m_state_msg.Va = 25.0;
    m_state_msg.alpha = 0;
    m_state_msg.beta = 0;
    m_state_msg.Vg = m_state_msg.Va;
    m_state_msg.gamma = 0;
    m_state_msg.chi = 0;

    m_state_msg.wn = 0;
    m_state_msg.we = 0;
    m_state_msg.bx = 0;
    m_state_msg.by = 0;
    m_state_msg.bz = 0;
}

} // end namespace quad
