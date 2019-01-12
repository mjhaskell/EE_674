#ifndef DRONEUPDATECALLBACK_HPP
#define DRONEUPDATECALLBACK_HPP

#include <osg/Node>

namespace osgGA
{
    class TrackballManipulator;
}

class DroneUpdateCallback : public osg::NodeCallback
{
public:
    DroneUpdateCallback(osg::ref_ptr<osgGA::TrackballManipulator>);
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
    void updateStates(osg::Vec3d new_pos,osg::Quat new_att);
    void resetManipulator();

protected:
    osg::ref_ptr<osgGA::TrackballManipulator> m_manipulator{nullptr};
    osg::Quat m_q_i2c;
    double m_max_angle;
    void updateManipulator();

private:
    osg::Vec3d m_pos_offset;
    osg::Vec3d m_pos;
    osg::Quat  m_att;
    osg::Vec3d m_eye;
    osg::Vec3d m_center;
    osg::Vec3d m_up;
};

#endif // DRONEUPDATECALLBACK_HPP
