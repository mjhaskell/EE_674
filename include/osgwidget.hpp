#ifndef OSGWIDGET_HPP
#define OSGWIDGET_HPP

#include <QOpenGLWidget>

#include <osg/ref_ptr>
#include <osgViewer/GraphicsWindow>
#include <osgViewer/CompositeViewer>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/NodeTrackerManipulator>
#include <osgText/Text>
#include <dronenode.hpp>
#include "droneupdatecallback.hpp"

class OSGWidget : public QOpenGLWidget
{
  Q_OBJECT

public:
  OSGWidget(QWidget* parent = 0,Qt::WindowFlags f = 0);
  virtual ~OSGWidget();
  void resetManipulatorView();

public slots:
  void updateDroneStates(nav_msgs::Odometry* odom);

protected:
  virtual void paintEvent(QPaintEvent* paintEvent);
  virtual void paintGL();
  virtual void resizeGL(int width, int height);
  virtual void keyPressEvent(QKeyEvent* event);
  virtual void keyReleaseEvent(QKeyEvent* event);
  virtual void mouseMoveEvent(QMouseEvent* event);
  virtual void mousePressEvent(QMouseEvent* event);
  virtual void mouseReleaseEvent(QMouseEvent* event);
  virtual void wheelEvent(QWheelEvent* event);
  virtual bool event(QEvent* event);
  void timerEvent(QTimerEvent *);
  void repaintOsgGraphicsAfterInteraction(QEvent* event);

private:
  virtual void on_resize(int width, int height);
  osgGA::EventQueue* getEventQueue() const;
  osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> m_graphics_window;
  osg::ref_ptr<osgViewer::CompositeViewer> m_viewer;
  osg::ref_ptr<osgViewer::View> m_view;
  osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> m_manipulator;
  osg::ref_ptr<osgGA::TrackballManipulator> m_custom_manipulator;
  osg::ref_ptr<osgGA::NodeTrackerManipulator> m_tracker_manipulator;
  osg::ref_ptr<osg::Group> m_root;
  osg::ref_ptr<DroneUpdateCallback> m_drone_update_callback;
  void setupManipulators();
  void setupCamera(osg::Camera* camera);
  void setupView(osg::Camera* camera);
  void setupViewer();
  void setupCameraAndView();
  osg::ref_ptr<osg::Node> createFloor();
  osg::ref_ptr<osg::Node> createOrigin(osg::Vec3d &scale_factor);
  void insertGround();
  void insertStructures();
  void insertClouds();
  void insertPinetrees();
  void insertTrees();
  void setupEnvironment();
  osg::ref_ptr<osg::PositionAttitudeTransform> createDrone(double bounding_radius);
  osg::ref_ptr<osg::PositionAttitudeTransform> createCastle(double bounding_radius);
  osg::ref_ptr<osg::PositionAttitudeTransform> createTreehouse(double bounding_radius);
  osg::ref_ptr<osg::PositionAttitudeTransform> createTower(double bounding_radius);
  osg::ref_ptr<osg::Node> createCloud(double bounding_radius);
  osg::ref_ptr<osg::Node> createPinetree(double bounding_radius);
  osg::ref_ptr<osg::Node> createTree(double bounding_radius);
  int m_timer_id{0};
  void setupTimer();
};

#endif // OSGWIDGET_HPP
