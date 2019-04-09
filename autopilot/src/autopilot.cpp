#include "autopilot/autopilot.hpp"

namespace ctrl
{

Autopilot::Autopilot() :
    m_nh{ros::NodeHandle()},
    m_nh_private{"~"},
    m_chi_c{0},
    m_h_c{50},
    m_Va_c{25},
    m_phi_ff{0},
    m_is_running{false},
    m_prev_t{0}
{
    this->setUpControllers();
    uint32_t queue_size{5};
    m_state_sub = m_nh.subscribe("states/truth",queue_size,
                                 &Autopilot::stateCallback, this);
    m_status_sub = m_nh.subscribe("sim/status",queue_size,
                                 &Autopilot::statusCallback,this);
    m_cmd_sub = m_nh.subscribe("command/high_level", queue_size,
                               &Autopilot::cmdCallback, this);
    m_delta_pub = m_nh.advertise<uav_msgs::Delta>("command/delta",
                                    queue_size);
    m_state_cmd_pub = m_nh.advertise<uav_msgs::State>(
                                    "states/commanded",queue_size);
}

Autopilot::~Autopilot()
{
}

void Autopilot::statusCallback(const uav_msgs::StatusConstPtr& msg)
{
    m_is_running = msg->is_flying;
    if (msg->reset)
        this->resetNode();
}

void Autopilot::cmdCallback(const uav_msgs::CommandConstPtr& msg)
{
    m_Va_c = msg->Va;
    m_h_c = msg->h;
    m_chi_c = msg->chi;
    m_phi_ff = msg->phi_ff;
}

void Autopilot::stateCallback(const uav_msgs::StateConstPtr& msg)
{
    if (m_is_running)
    {
        if (m_prev_t == 0.0)
        {
            m_prev_t = msg->header.stamp.toSec();
            return;
        }
        double now{msg->header.stamp.toSec()};
        double dt{now - m_prev_t};
        m_prev_t = now;
        if (dt <= 0)
            return;    
        
        // lateral autopilot update
        double phi_c{m_roll_from_course.update(m_chi_c,msg->chi,dt,true)};
        m_cmd.da = m_aileron_from_roll.update(phi_c,msg->phi,msg->p,dt);
        m_cmd.dr = m_yaw_damper.update(msg->r,dt);

        // longitudinal autopilot update
        double theta_c{m_pitch_from_altitude.update(m_h_c,msg->h,dt)};
        m_cmd.de = m_elevator_from_pitch.update(theta_c,msg->theta,msg->q,dt);
        m_cmd.dt = m_throttle_from_airspeed.update(m_Va_c,msg->Va,dt);
        m_cmd.header.stamp = ros::Time::now();

        m_delta_pub.publish(m_cmd);

        m_state_cmd.h = m_h_c;
        m_state_cmd.Va = m_Va_c;
        m_state_cmd.phi = phi_c;
        m_state_cmd.theta = theta_c;
        m_state_cmd.chi = m_chi_c;
        m_state_cmd.header.stamp = ros::Time::now();

        m_state_cmd_pub.publish(m_state_cmd);
    }
}

void Autopilot::setUpControllers()
{
    double P,I,D,tau;
    P = m_nh_private.param<double>("roll_kp",0.41);
    I = m_nh_private.param<double>("roll_ki",0.0);
    D = m_nh_private.param<double>("roll_kd",0.08);
    m_aileron_from_roll.setGains(P,I,D);
    m_aileron_from_roll.setSaturationLimits(rad(45));

    P = m_nh_private.param<double>("course_kp",1.5);
    I = m_nh_private.param<double>("course_ki",0.3);
    D = m_nh_private.param<double>("course_kd",0.0);
    m_roll_from_course.setGains(P,I,D);
    m_roll_from_course.setSaturationLimits(rad(30.0));
    
    P = m_nh_private.param<double>("yaw_kp",1.0);
    tau = m_nh_private.param<double>("yaw_tau",0.05);
    m_yaw_damper.setGains(P,tau);

    P = m_nh_private.param<double>("pitch_kp",-4.5);
    I = m_nh_private.param<double>("pitch_ki",0.0);
    D = m_nh_private.param<double>("pitch_kd",-0.7);
    m_elevator_from_pitch.setGains(P,I,D);
    m_elevator_from_pitch.setSaturationLimits(rad(45.0));

    P = m_nh_private.param<double>("altitude_kp",0.05);
    I = m_nh_private.param<double>("altitude_ki",0.011);
    D = m_nh_private.param<double>("altitude_kd",0.0);
    m_pitch_from_altitude.setGains(P,I,D);
    m_pitch_from_altitude.setSaturationLimits(rad(30.0));

    P = m_nh_private.param<double>("airspeed_kp",1.25);
    I = m_nh_private.param<double>("airspeed_ki",0.35);
    D = m_nh_private.param<double>("airspeed_kd",0.0);
    m_throttle_from_airspeed.setGains(P,I,D);
    m_throttle_from_airspeed.setSaturationLimits(0.0,1.0);
}

void Autopilot::resetNode()
{
    m_prev_t = 0.0;
    m_chi_c = 0.0;
    m_h_c = 50.0;
    m_Va_c = 25.0;
    m_cmd.da = 0.0;
    m_cmd.de = 0.0;
    m_cmd.dr = 0.0;
    m_cmd.dt = 0.5;
    m_state_cmd.Va = m_Va_c;
    m_state_cmd.chi = m_chi_c;
    m_state_cmd.h = m_h_c;
    m_state_cmd.phi = 0.0;
    m_state_cmd.theta = 0.0;

    // reset integrators on controllers
    m_aileron_from_roll.reset();
    m_roll_from_course.reset();
    m_yaw_damper.reset();
    m_elevator_from_pitch.reset();
    m_pitch_from_altitude.reset();
    m_throttle_from_airspeed.reset();
}

double Autopilot::rad(double degrees)
{
    return degrees * 3.14159 / 180.0;
}

} // end namespace mav
