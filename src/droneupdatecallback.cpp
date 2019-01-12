#include "droneupdatecallback.hpp"
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>

DroneUpdateCallback::DroneUpdateCallback(osg::ref_ptr<osgGA::TrackballManipulator> manipulator) :
    m_manipulator{manipulator},
    m_q_i2c{0,0,0,1},
    m_max_angle{osg::DegreesToRadians(17.0)},
    m_pos_offset{0,0,-0.045},
    m_pos{0,0,0},
    m_att{0,0,0,1},
    m_eye{-5.0,0,-1.0},
    m_center{0,0,0},
    m_up{0,0,-1.0}
{
}

void DroneUpdateCallback::updateManipulator()
{
    osg::Vec3d pos_i{m_pos.x()-m_eye.x(),m_pos.y(),0};
    osg::Vec3d pos_c{m_q_i2c.conj()*pos_i};

    double angle_to_drone{atan2(pos_c.y(),pos_c.x())};
    double rot_angle{0};
    if (angle_to_drone > m_max_angle)
        rot_angle = angle_to_drone - m_max_angle;
    else if (angle_to_drone < -m_max_angle)
        rot_angle = angle_to_drone + m_max_angle;
    m_q_i2c *= osg::Quat{rot_angle,-m_up};

    m_center.set(pos_c.length(),0,0);
    m_center = m_q_i2c*m_center;
    m_center.set(m_center.x()+m_eye.x(),m_center.y(),m_pos.z());

    m_manipulator->setTransformation(m_eye,m_center,m_up);
}

void DroneUpdateCallback::resetManipulator()
{
    m_q_i2c.set(0,0,0,1.0);
    m_pos.set(0,0,0);
    m_att.set(0,0,0,1.0);
    m_center.set(0,0,0);
    m_manipulator->setTransformation(m_eye,m_center,m_up);
}

void DroneUpdateCallback::operator()(osg::Node *node, osg::NodeVisitor *nv)
{
    osg::PositionAttitudeTransform *pat{dynamic_cast<osg::PositionAttitudeTransform*>(node)};
    pat->setPosition(m_pos+m_pos_offset);
    pat->setAttitude(m_att);
    this->updateManipulator();

    traverse(node, nv);
}

void DroneUpdateCallback::updateStates(osg::Vec3d new_pos, osg::Quat new_att)
{
    m_pos = new_pos;
    m_att = new_att;
}
