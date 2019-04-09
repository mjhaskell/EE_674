#ifndef AUTOPILOT_HPP
#define AUTOPILOT_HPP

#include <ros/ros.h>
#include "autopilot/pid.hpp"
#include "autopilot/yawdamper.hpp"
#include "uav_msgs/State.h"
#include "uav_msgs/Status.h"
#include "uav_msgs/Delta.h"
#include "uav_msgs/Command.h"

namespace ctrl
{

class Autopilot
{
public:
    Autopilot();
    virtual ~Autopilot();

protected:
    void statusCallback(const uav_msgs::StatusConstPtr& msg);
    void cmdCallback(const uav_msgs::CommandConstPtr& msg);
    void stateCallback(const uav_msgs::StateConstPtr& msg);
    void setUpControllers();
    void resetNode();
    double rad(double degrees);

private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_nh_private;
    ros::Subscriber m_state_sub;
    ros::Subscriber m_status_sub;
    ros::Subscriber m_cmd_sub;
    ros::Publisher m_delta_pub;
    ros::Publisher m_state_cmd_pub;

    // lateral controllers
    PID m_aileron_from_roll;
    PID m_roll_from_course;
    YawDamper m_yaw_damper;

    // longitudinal controllers
    PID m_elevator_from_pitch;
    PID m_pitch_from_altitude;
    PID m_throttle_from_airspeed;

    uav_msgs::State m_state_cmd;
    uav_msgs::Delta m_cmd;
    double m_chi_c, m_h_c, m_Va_c, m_phi_ff;
    bool m_is_running;
    double m_prev_t;
};

} // end namespace mav
#endif // AUTOPILOT_HPP
