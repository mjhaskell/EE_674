#include "osgwidget.hpp"
#include "droneupdatecallback.hpp"

#include <osg/Camera>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osgGA/EventQueue>
#include <osgViewer/View>
#include <osgViewer/ViewerEventHandlers>
#include <osg/PositionAttitudeTransform>
#include <QKeyEvent>
#include <QPainter>
#include <QWheelEvent>
#include <osgDB/ReadFile>
#include <osgUtil/SmoothingVisitor>

void OSGWidget::setupTimer()
{
    double frames_per_second{30};
    double time_step{1.0/frames_per_second};
    double timer_duration_ms{time_step * 1000};
    m_timer_id = startTimer(timer_duration_ms);
}

OSGWidget::OSGWidget(QWidget* parent,Qt::WindowFlags flags):
    QOpenGLWidget{parent,flags},
    m_graphics_window{new osgViewer::GraphicsWindowEmbedded{this->x(),this->y(),this->width(),this->height()}},
    m_viewer{new osgViewer::CompositeViewer},
    m_view{new osgViewer::View},
    m_manipulator{new osgGA::KeySwitchMatrixManipulator},
    m_custom_manipulator{new osgGA::TrackballManipulator},
    m_tracker_manipulator{new osgGA::NodeTrackerManipulator},
    m_root{new osg::Group},
    m_drone_update_callback{new DroneUpdateCallback{m_custom_manipulator}}
{
    this->setupManipulators();
    this->setupCameraAndView();
    this->setupEnvironment();

    double drone_radius{0.3};
    osg::ref_ptr<osg::PositionAttitudeTransform> drone_pat{this->createDrone(drone_radius)};
    drone_pat->addUpdateCallback(m_drone_update_callback);
    m_root->addChild(drone_pat);
    m_tracker_manipulator->setTrackNode(drone_pat);

    this->setFocusPolicy(Qt::StrongFocus);
    this->setMouseTracking(true);
    this->update();
    setupTimer();
}

OSGWidget::~OSGWidget()
{
    killTimer(m_timer_id);
}

void OSGWidget::resetManipulatorView()
{
    m_drone_update_callback->resetManipulator();
}

void OSGWidget::updateDroneStates(nav_msgs::Odometry* odom)
{
    osg::Vec3d pos{odom->pose.pose.position.x,odom->pose.pose.position.y,odom->pose.pose.position.z};
    osg::Quat att{odom->pose.pose.orientation.x,
                 odom->pose.pose.orientation.y,
                 odom->pose.pose.orientation.z,
                 odom->pose.pose.orientation.w};
    m_drone_update_callback->updateStates(pos,att);
}

void OSGWidget::timerEvent(QTimerEvent *)
{
    update();
}

void OSGWidget::paintEvent(QPaintEvent* paintEvent)
{
    this->makeCurrent();

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    this->paintGL();

    painter.end();

    this->doneCurrent();
}

void OSGWidget::paintGL()
{
    m_viewer->frame();
}

void OSGWidget::resizeGL(int width, int height)
{
    this->getEventQueue()->windowResize(this->x(), this->y(), width, height);
    m_graphics_window->resized(this->x(), this->y(), width, height);

    this->on_resize(width, height);
}

void OSGWidget::keyPressEvent(QKeyEvent* event)
{
    QString keyString{event->text()};
    const char* keyData{keyString.toLocal8Bit().data()};

    if(event->key() == Qt::Key_H)
    {
        m_view->home();
        return;
    }

    this->getEventQueue()->keyPress(osgGA::GUIEventAdapter::KeySymbol(*keyData));
}

void OSGWidget::keyReleaseEvent(QKeyEvent* event)
{
    QString keyString{event->text()};
    const char* keyData{keyString.toLocal8Bit().data()};

    this->getEventQueue()->keyRelease(osgGA::GUIEventAdapter::KeySymbol(*keyData));
}

void OSGWidget::mouseMoveEvent(QMouseEvent* event)
{
    auto pixelRatio{this->devicePixelRatio()};

    this->getEventQueue()->mouseMotion(static_cast<float>(event->x() * pixelRatio),
                                        static_cast<float>(event->y() * pixelRatio));
}

void OSGWidget::mousePressEvent(QMouseEvent* event)
{
    static int left_mouse_button{1};
    static int middle_mouse_button{2};
    static int right_mouse_button{3};

    unsigned int button{0};

    switch(event->button())
    {
    case Qt::LeftButton:
        button = left_mouse_button;
        break;

    case Qt::MiddleButton:
        button = middle_mouse_button;
        break;

    case Qt::RightButton:
        button = right_mouse_button;
        break;

    default:
        break;
    }

    auto pixelRatio{this->devicePixelRatio()};

    this->getEventQueue()->mouseButtonPress(static_cast<float>(event->x() * pixelRatio),
                                             static_cast<float>(event->y() * pixelRatio),
                                             button);

}

void OSGWidget::mouseReleaseEvent(QMouseEvent* event)
{
    static int left_mouse_button{1};
    static int middle_mouse_button{2};
    static int right_mouse_button{3};

    unsigned int button{0};

    switch(event->button())
    {
    case Qt::LeftButton:
        button = left_mouse_button;
        break;

    case Qt::MiddleButton:
        button = middle_mouse_button;
        break;

    case Qt::RightButton:
        button = right_mouse_button;
        break;

    default:
        break;
    }

    auto pixelRatio{this->devicePixelRatio()};

    this->getEventQueue()->mouseButtonRelease(static_cast<float>(pixelRatio * event->x()),
                                               static_cast<float>(pixelRatio * event->y()),
                                               button);
}

void OSGWidget::wheelEvent(QWheelEvent* event)
{
    event->accept();
    int delta{event->delta()};

    osgGA::GUIEventAdapter::ScrollingMotion motion{delta > 0 ?   osgGA::GUIEventAdapter::SCROLL_UP
                                                               : osgGA::GUIEventAdapter::SCROLL_DOWN};
    this->getEventQueue()->mouseScroll(motion);
}

void OSGWidget::on_resize(int width, int height)
{
    std::vector<osg::Camera*> cameras;
    m_viewer->getCameras(cameras);

    auto pixelRatio{this->devicePixelRatio()};

    unsigned int viewport_x{0};
    unsigned int viewport_y{0};
    cameras[0]->setViewport(viewport_x, viewport_y, width * pixelRatio, height * pixelRatio);
}

osgGA::EventQueue* OSGWidget::getEventQueue() const
{
    osgGA::EventQueue *eventQueue{m_graphics_window->getEventQueue()};

    if(eventQueue)
        return eventQueue;
    else
        throw std::runtime_error("Unable to obtain valid event queue");
}

void OSGWidget::setupManipulators()
{
    m_custom_manipulator->setAllowThrow(false);
    m_tracker_manipulator->setAllowThrow(false);

    osgGA::NodeTrackerManipulator::TrackerMode track_mode{osgGA::NodeTrackerManipulator::NODE_CENTER_AND_AZIM};
    m_tracker_manipulator->setTrackerMode(track_mode);
    osgGA::NodeTrackerManipulator::RotationMode rot_mode{osgGA::NodeTrackerManipulator::TRACKBALL};
    m_tracker_manipulator->setRotationMode(rot_mode);

    osg::Vec3d eye{-5.0,0,-1.0};
    osg::Vec3d center{0,0,0};
    osg::Vec3d up{0,0,-1};
    m_custom_manipulator->setHomePosition(eye,center,up);
    m_tracker_manipulator->setHomePosition(eye,center,up);

    m_manipulator->addMatrixManipulator('1',"Custom",m_custom_manipulator);
    m_manipulator->addMatrixManipulator('2',"Tracker",m_tracker_manipulator);
}

bool OSGWidget::event(QEvent *event)
{
    bool handled{QOpenGLWidget::event(event)};
    repaintOsgGraphicsAfterInteraction(event);
    return handled;
}

void OSGWidget::repaintOsgGraphicsAfterInteraction(QEvent* event)
{
    switch(event->type())
    {
    case QEvent::KeyPress:
    case QEvent::KeyRelease:
    case QEvent::MouseButtonDblClick:
    case QEvent::MouseButtonPress:
    case QEvent::MouseButtonRelease:
    case QEvent::MouseMove:
    case QEvent::Wheel:
        this->update();
        break;

    default:
        break;
    }
}

void OSGWidget::setupCamera(osg::Camera* camera)
{
    float aspect_ratio{static_cast<float>(this->width()) / static_cast<float>(this->height())};
    auto pixel_ratio{this->devicePixelRatio()};

    unsigned int viewport_x{0};
    unsigned int viewport_y{0};
    camera->setViewport(viewport_x, viewport_y, this->width() * pixel_ratio, this->height() * pixel_ratio);

    osg::Vec4 color_rgba{0.0f, 0.6f, 1.0f, 1.0f};
    camera->setClearColor(color_rgba);

    double angle_of_view{60.0};
    double min_distance{1.0};
    double max_distance{1000.0};
    camera->setProjectionMatrixAsPerspective(angle_of_view, aspect_ratio, min_distance, max_distance);
    camera->setGraphicsContext(m_graphics_window);
}

void OSGWidget::setupView(osg::Camera* camera)
{
    m_view->setCamera(camera);
    m_view->setSceneData(m_root.get());
    m_view->addEventHandler(new osgViewer::StatsHandler);
    m_view->setCameraManipulator(m_manipulator);
    m_view->home();
    m_view->setLightingMode(osg::View::SKY_LIGHT);
    osg::Light *light{m_view->getLight()};
    osg::Vec4 light_pos{0,0,-100,0};
    light->setPosition(light_pos);
}

void OSGWidget::setupViewer()
{
    m_viewer->addView(m_view);
    m_viewer->setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);
    m_viewer->realize();
}

void OSGWidget::setupCameraAndView()
{
    osg::Camera *camera{new osg::Camera};
    this->setupCamera(camera);
    this->setupView(camera);
    this->setupViewer();
}

osg::ref_ptr<osg::Vec3Array> getFloorVertices(float x, float y)
{
    float z{-0.01f};
    osg::ref_ptr<osg::Vec3Array> vertices{new osg::Vec3Array};
    vertices->push_back(osg::Vec3(-x, -y, z));
    vertices->push_back(osg::Vec3(-x,  y, z));
    vertices->push_back(osg::Vec3(x,   y, z));
    vertices->push_back(osg::Vec3(x,  -y, z));
    return vertices;
}

osg::ref_ptr<osg::Vec3Array> getFloorNormals()
{
    osg::ref_ptr<osg::Vec3Array> normals{new osg::Vec3Array};
    osg::Vec3 normal_dir{0.0f, 0.0f, 1.0f};
    normals->push_back(normal_dir);
    normals->push_back(normal_dir);
    normals->push_back(normal_dir);
    normals->push_back(normal_dir);
    return normals;
}

osg::ref_ptr<osg::Vec2Array> getTexCoords(float repetitions)
{
    osg::ref_ptr<osg::Vec2Array> tex_coords{new osg::Vec2Array};
    float zero{0.f};
    tex_coords->push_back(osg::Vec2{zero, zero});
    tex_coords->push_back(osg::Vec2{zero, repetitions});
    tex_coords->push_back(osg::Vec2{repetitions, repetitions});
    tex_coords->push_back(osg::Vec2{repetitions, zero});
    return tex_coords;
}

osg::Geometry* createFloorGeom()
{
    osg::Geometry *geom{new osg::Geometry};

    float x_dim{100.f};
    float y_dim{100.f};
    osg::ref_ptr<osg::Vec3Array> vertices{getFloorVertices(x_dim,y_dim)};
    geom->setVertexArray(vertices);

    osg::ref_ptr<osg::Vec3Array> normals{getFloorNormals()};
    geom->setNormalArray(normals, osg::Array::Binding::BIND_PER_VERTEX);

    float repetions{100.f};
    osg::ref_ptr<osg::Vec2Array> tex_coords{getTexCoords(repetions)};
    geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));
    geom->setTexCoordArray(0, tex_coords.get());
    osgUtil::SmoothingVisitor::smooth(*geom);
    geom->setTexCoordArray(0, tex_coords.get(), osg::Array::Binding::BIND_PER_VERTEX);

    return geom;
}

osg::ref_ptr<osg::Texture2D> createFloorTexture()
{
    osg::ref_ptr<osg::Texture2D> texture{new osg::Texture2D};
    osg::ref_ptr<osg::Image> image{osgDB::readImageFile("../obj/grass1.jpg")};
    texture->setImage(image);
    texture->setUnRefImageDataAfterApply(true);
    texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    texture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    return texture;
}

osg::ref_ptr<osg::Node> OSGWidget::createFloor()
{
    osg::Geometry *geom{createFloorGeom()};
    osg::ref_ptr<osg::Texture2D> texture{createFloorTexture()};

    osg::StateSet *geom_state_set = geom->getOrCreateStateSet();
    geom_state_set->setTextureAttributeAndModes(0, texture.get(), osg::StateAttribute::ON);

    return geom;
}

osg::Geometry* getOriginAxis(int x,int y,int z)
{
    osg::Vec3Array* v{new osg::Vec3Array};
    v->resize(2);
    (*v)[0].set(0, 0, 0);
    (*v)[1].set(x, y, z);

    osg::Geometry* geom{new osg::Geometry};
    geom->setUseDisplayList(false);
    geom->setVertexArray(v);
    osg::Vec4 color{float(z),float(y),float(x),1.f};
    osg::Vec4Array* c{new osg::Vec4Array};
    c->push_back(color);
    geom->setColorArray(c, osg::Array::BIND_OVERALL);

    GLushort idx_lines[2] = {0, 1};
    geom->addPrimitiveSet(new osg::DrawElementsUShort{osg::PrimitiveSet::LINES, 2, idx_lines});
    return geom;
}

osg::ref_ptr<osg::Node> OSGWidget::createOrigin(osg::Vec3d &scale_factor)
{
    osg::ref_ptr<osg::Geometry> x_axis{getOriginAxis(1,0,0)};
    osg::ref_ptr<osg::Geometry> y_axis{getOriginAxis(0,1,0)};
    osg::ref_ptr<osg::Geometry> z_axis{getOriginAxis(0,0,1)};

    osg::ref_ptr<osg::Geode> geode{new osg::Geode};
    geode->addDrawable(x_axis);
    geode->addDrawable(y_axis);
    geode->addDrawable(z_axis);

    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
    geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    osg::ref_ptr<osg::PositionAttitudeTransform> transform{new osg::PositionAttitudeTransform};
    transform->setScale(scale_factor);

    transform->addChild(geode);
    return transform;
}

void OSGWidget::insertGround()
{
    osg::ref_ptr<osg::Node> floor{this->createFloor()};
    m_root->addChild(floor);

    osg::Vec3d scale_factor{1,1,1};
    osg::ref_ptr<osg::Node> origin_pat{this->createOrigin(scale_factor)};
    m_root->addChild(origin_pat);
}

void OSGWidget::insertStructures()
{
    double castle_radius{40.0};
    osg::ref_ptr<osg::PositionAttitudeTransform> castle_pat{this->createCastle(castle_radius)};
    castle_pat->setPosition(osg::Vec3d{50,50,-0.33*castle_radius});
    m_root->addChild(castle_pat);

    double treehouse_radius{11.0};
    osg::ref_ptr<osg::PositionAttitudeTransform> treehouse_pat{this->createTreehouse(treehouse_radius)};
    treehouse_pat->setPosition(osg::Vec3d{-10,65,-0.77*treehouse_radius});
    m_root->addChild(treehouse_pat);

    double tower_radius{10.0};
    osg::ref_ptr<osg::PositionAttitudeTransform> tower_pat{this->createTower(tower_radius)};
    tower_pat->setPosition(osg::Vec3d{-20,-75,-0.795*tower_radius});
    m_root->addChild(tower_pat);
}

void OSGWidget::insertClouds()
{
    double cloud_radius{50.0};
    osg::ref_ptr<osg::Node> cloud{this->createCloud(cloud_radius)};
    int num_clouds{9};
    osg::ref_ptr<osg::PositionAttitudeTransform> cloud_pat[num_clouds];
    osg::Vec3d cloud_pos[num_clouds];
    cloud_pos[0] = osg::Vec3d{-200,-150,-150};
    cloud_pos[1] = osg::Vec3d{-150,150,-140};
    cloud_pos[2] = osg::Vec3d{150,-150,-160};
    cloud_pos[3] = osg::Vec3d{150,150,-170};
    cloud_pos[4] = osg::Vec3d{-100,-50,-140};
    cloud_pos[5] = osg::Vec3d{50,200,-150};
    cloud_pos[6] = osg::Vec3d{250,0,-130};
    cloud_pos[7] = osg::Vec3d{0,-250,-160};
    cloud_pos[8] = osg::Vec3d{0,0,-170};
    for (int i{0}; i < num_clouds; i++)
    {
        cloud_pat[i] = new osg::PositionAttitudeTransform;
        cloud_pat[i]->addChild(cloud);
        cloud_pat[i]->setPosition(cloud_pos[i]);
        m_root->addChild(cloud_pat[i]);
    }
}

void OSGWidget::insertPinetrees()
{
    double pinetree_radius{10};
    osg::ref_ptr<osg::Node> pinetree{this->createPinetree(pinetree_radius)};
    int num_pinetrees{10};
    osg::ref_ptr<osg::PositionAttitudeTransform> pinetree_pat[num_pinetrees];
    osg::Vec3d pinetree_pos[num_pinetrees];
    pinetree_pos[0] = osg::Vec3d{50,85,-0.848*pinetree_radius};
    pinetree_pos[1] = osg::Vec3d{85,65,-0.848*pinetree_radius};
    pinetree_pos[2] = osg::Vec3d{85,-85,-0.848*pinetree_radius};
    pinetree_pos[3] = osg::Vec3d{80,-65,-0.848*pinetree_radius};
    pinetree_pos[4] = osg::Vec3d{60,-30,-0.848*pinetree_radius};
    pinetree_pos[5] = osg::Vec3d{50,-50,-0.848*pinetree_radius};
    pinetree_pos[6] = osg::Vec3d{85,-10,-0.848*pinetree_radius};
    pinetree_pos[7] = osg::Vec3d{75,25,-0.848*pinetree_radius};
    pinetree_pos[8] = osg::Vec3d{30,-80,-0.848*pinetree_radius};
    pinetree_pos[9] = osg::Vec3d{65,-75,-0.848*pinetree_radius};
    for (int i{0}; i < num_pinetrees; i++)
    {
        pinetree_pat[i] = new osg::PositionAttitudeTransform;
        pinetree_pat[i]->addChild(pinetree);
        pinetree_pat[i]->setPosition(pinetree_pos[i]);
        m_root->addChild(pinetree_pat[i]);
    }
}

void OSGWidget::insertTrees()
{
    double tree_radius{8};
    osg::ref_ptr<osg::Node> tree{this->createTree(tree_radius)};
    int num_trees{11};
    osg::ref_ptr<osg::PositionAttitudeTransform> tree_pat[num_trees];
    osg::Vec3d tree_pos[num_trees];
    tree_pos[0] = osg::Vec3d{-55,-85,-0.695*tree_radius};
    tree_pos[1] = osg::Vec3d{-55,85,-0.695*tree_radius};
    tree_pos[2] = osg::Vec3d{-85,85,-0.695*tree_radius};
    tree_pos[3] = osg::Vec3d{-70,55,-0.695*tree_radius};
    tree_pos[4] = osg::Vec3d{-55,25,-0.695*tree_radius};
    tree_pos[5] = osg::Vec3d{-85,25,-0.695*tree_radius};
    tree_pos[6] = osg::Vec3d{-70,0,-0.695*tree_radius};
    tree_pos[7] = osg::Vec3d{-55,-25,-0.695*tree_radius};
    tree_pos[8] = osg::Vec3d{-85,-25,-0.695*tree_radius};
    tree_pos[9] = osg::Vec3d{-70,-55,-0.695*tree_radius};
    tree_pos[10] = osg::Vec3d{-85,-85,-0.695*tree_radius};
    for (int i{0}; i < num_trees; i++)
    {
        tree_pat[i] = new osg::PositionAttitudeTransform;
        tree_pat[i]->addChild(tree);
        tree_pat[i]->setPosition(tree_pos[i]);
        m_root->addChild(tree_pat[i]);
    }
}

void OSGWidget::setupEnvironment()
{
    insertGround();
    insertStructures();
    insertClouds();
    insertPinetrees();
    insertTrees();
}

osg::ref_ptr<osg::Node> scaleModel(const osg::ref_ptr<osg::Node> &model, double bounding_radius)
{
    osg::BoundingSphere bb{model->getBound()};

    osg::ref_ptr<osg::PositionAttitudeTransform> scale_trans{new osg::PositionAttitudeTransform};
    double bounding_box_radius{bb.radius()};
    double radius_ratio{bounding_radius/bounding_box_radius};
    scale_trans->setScale(osg::Vec3d{radius_ratio,radius_ratio,radius_ratio});
    scale_trans->addChild(model);

    return scale_trans.release();
}

osg::ref_ptr<osg::Node> rotateModel(const osg::ref_ptr<osg::Node> &model,const osg::Quat &q)
{
    osg::ref_ptr<osg::PositionAttitudeTransform> pat{new osg::PositionAttitudeTransform};
    pat->setAttitude(q);
    pat->addChild(model);
    return pat.release();
}

osg::ref_ptr<osg::Node> translateModel(const osg::ref_ptr<osg::Node> &model,const osg::Vec3d &offset)
{
    osg::BoundingSphere bb{model->getBound()};
    osg::ref_ptr<osg::PositionAttitudeTransform> pat{new osg::PositionAttitudeTransform};
    osg::Vec3d pos{bb.center()};
    pos = osg::Vec3d{-pos.x(),-pos.y(),-pos.z()} + offset;
    pat->setPosition(pos);
    pat->addChild(model);
    return pat.release();
}

osg::ref_ptr<osg::Node> createModel(std::string name)
{
    osg::ref_ptr<osg::Node> model{osgDB::readNodeFile(name)};

    if(model.valid())
    {
        osg::StateSet* state_set{model->getOrCreateStateSet()};
        state_set->setMode(GL_DEPTH_TEST,osg::StateAttribute::ON);
        state_set->setMode(GL_RESCALE_NORMAL,osg::StateAttribute::ON);
    }
    return model.release();
}

osg::ref_ptr<osg::PositionAttitudeTransform> OSGWidget::createDrone(double bounding_radius)
{
    osg::ref_ptr<osg::Node> model{createModel("../obj/simple_drone.obj")};
    osg::ref_ptr<osg::Node> scaled_model{scaleModel(model,bounding_radius)};
    osg::Vec3d cog_offset{0,-0.02,0};
    osg::ref_ptr<osg::Node> translated_model{translateModel(scaled_model,cog_offset)};
    double angle{osg::DegreesToRadians(90.0)};
    osg::Vec3d axis1{1,0,0};
    osg::Vec3d axis2{0,0,1};
    osg::Quat q1{angle,axis1};
    osg::Quat q2{angle,axis2};
    osg::ref_ptr<osg::Node> rotated_model{rotateModel(translated_model,q1*q2)};
    osg::ref_ptr<osg::PositionAttitudeTransform> drone_at_origin{new osg::PositionAttitudeTransform};
    drone_at_origin->addChild(rotated_model);

    return drone_at_origin.release();
}

osg::ref_ptr<osg::PositionAttitudeTransform> OSGWidget::createCastle(double bounding_radius)
{
    osg::ref_ptr<osg::Node> model{createModel("../obj/castle.obj")};
    osg::ref_ptr<osg::Node> scaled_model{scaleModel(model,bounding_radius)};
    osg::Vec3d cog_offset{0,0,-0.1*bounding_radius};
    osg::ref_ptr<osg::Node> translated_model{translateModel(scaled_model,cog_offset)};
    double angle{osg::DegreesToRadians(180.0)};
    osg::Vec3d axis{1,0,0};
    osg::Quat q{angle,axis};
    osg::ref_ptr<osg::Node> rotated_model{rotateModel(translated_model,q)};
    osg::ref_ptr<osg::PositionAttitudeTransform> castle_at_origin{new osg::PositionAttitudeTransform};
    castle_at_origin->addChild(rotated_model);

    return castle_at_origin.release();
}

osg::ref_ptr<osg::PositionAttitudeTransform> OSGWidget::createTreehouse(double bounding_radius)
{
    osg::ref_ptr<osg::Node> model{createModel("../obj/treehouse.3ds")};
    osg::ref_ptr<osg::Node> scaled_model{scaleModel(model,bounding_radius)};
    osg::Vec3d cog_offset{0,0,0};
    osg::ref_ptr<osg::Node> translated_model{translateModel(scaled_model,cog_offset)};
    double angle{osg::DegreesToRadians(180.0)};
    osg::Vec3d axis1{1,0,0};
    osg::Vec3d axis2{0,0,1};
    osg::Quat q1{angle,axis1};
    osg::Quat q2{angle,axis2};
    osg::ref_ptr<osg::Node> rotated_model{rotateModel(translated_model,q1*q2)};
    osg::ref_ptr<osg::PositionAttitudeTransform> treehouse_at_origin{new osg::PositionAttitudeTransform};
    treehouse_at_origin->addChild(rotated_model);

    return treehouse_at_origin.release();
}

osg::ref_ptr<osg::PositionAttitudeTransform> OSGWidget::createTower(double bounding_radius)
{
    osg::ref_ptr<osg::Node> model{createModel("../obj/tower.3ds")};
    osg::ref_ptr<osg::Node> scaled_model{scaleModel(model,bounding_radius)};
    osg::Vec3d cog_offset{0,0,0};
    osg::ref_ptr<osg::Node> translated_model{translateModel(scaled_model,cog_offset)};
    double angle{osg::DegreesToRadians(180.0)};
    osg::Vec3d axis1{1,0,0};
    osg::Vec3d axis2{0,0,1};
    osg::Quat q1{angle,axis1};
    osg::Quat q2{angle,axis2};
    osg::ref_ptr<osg::Node> rotated_model{rotateModel(translated_model,q1*q2)};
    osg::ref_ptr<osg::PositionAttitudeTransform> tower_at_origin{new osg::PositionAttitudeTransform};
    tower_at_origin->addChild(rotated_model);

    return tower_at_origin.release();
}

osg::ref_ptr<osg::Node> OSGWidget::createCloud(double bounding_radius)
{
    osg::ref_ptr<osg::Node> model{createModel("../obj/cloud.obj")};
    osg::ref_ptr<osg::Node> scaled_model{scaleModel(model,bounding_radius)};
    osg::Vec3d cog_offset{0,0,0};
    osg::ref_ptr<osg::Node> translated_model{translateModel(scaled_model,cog_offset)};

    return translated_model.release();
}

osg::ref_ptr<osg::Node> OSGWidget::createPinetree(double bounding_radius)
{
    osg::ref_ptr<osg::Node> model{createModel("../obj/firtree1.3ds")};
    osg::ref_ptr<osg::Node> scaled_model{scaleModel(model,bounding_radius)};
    osg::Vec3d cog_offset{0,0,0};
    osg::ref_ptr<osg::Node> translated_model{translateModel(scaled_model,cog_offset)};
    double angle{osg::DegreesToRadians(180.0)};
    osg::Vec3d axis{1,0,0};
    osg::Quat q{angle,axis};
    osg::ref_ptr<osg::Node> rotated_model{rotateModel(translated_model,q)};

    return rotated_model.release();
}

osg::ref_ptr<osg::Node> OSGWidget::createTree(double bounding_radius)
{
    osg::ref_ptr<osg::Node> model{createModel("../obj/Tree2.3ds")};
    osg::ref_ptr<osg::Node> scaled_model{scaleModel(model,bounding_radius)};
    osg::Vec3d cog_offset{0,0,0};
    osg::ref_ptr<osg::Node> translated_model{translateModel(scaled_model,cog_offset)};
    double angle{osg::DegreesToRadians(180.0)};
    osg::Vec3d axis{1,0,0};
    osg::Quat q{angle,axis};
    osg::ref_ptr<osg::Node> rotated_model{rotateModel(translated_model,q)};

    return rotated_model.release();
}
