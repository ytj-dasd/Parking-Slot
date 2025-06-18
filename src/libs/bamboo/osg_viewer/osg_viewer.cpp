#include "osg_viewer.h"
#include "osg_render.h"

#include <osg/GL>
#include <osgViewer/Viewer>
#include <osg/DeleteHandler>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/SphericalManipulator>
#include <osgGA/StandardManipulator>

#include <osg/CoordinateSystemNode>
// qt
#include <QApplication>
#include <QKeyEvent>
#include <QColorDialog>
#include <QInputDialog>
#include <QLayout>
#include <QMainWindow>
#include <QScreen>
#include <QWindow>

#include <common/base/dir.h>
#include <uface/base/uconverter.h>
#include <uface/resource/uresource.h>
namespace welkin::bamboo {
OSGViewer::OSGViewer(QWidget* parent) : QOpenGLWidget(parent) {
    _toolbar_view = new QToolBar("toolbar_osg_view", this);
    _action_top_view = _toolbar_view->addAction(UIcon("item_icon/view_top.png"),
        tr("TopView"), this, SLOT(onTopView()));
    _action_top_view->setShortcut(Qt::Key_O);
    _action_bottom_view = _toolbar_view->addAction(UIcon("item_icon/view_bottom.png"),
        tr("BottomView"), this, SLOT(onBottomView()));
    _action_front_view = _toolbar_view->addAction(UIcon("item_icon/view_front.png"),
        tr("FrontView"), this, SLOT(onFrontView()));
    _action_back_view = _toolbar_view->addAction(UIcon("item_icon/view_back.png"),
        tr("BackView"), this, SLOT(onBackView()));
    _action_left_view = _toolbar_view->addAction(UIcon("item_icon/view_left.png"),
        tr("LeftView"), this, SLOT(onLeftView()));
    _action_right_view = _toolbar_view->addAction(UIcon("item_icon/view_right.png"),
        tr("RightView"), this, SLOT(onRightView()));
    // _toolbar_view->addSeparator();
    // _action_change_color = _toolbar_view->addAction(UIcon("item_icon/color_select.png"),
    //     tr("ChangeColor"), this, SLOT(onChangeClearColor()));

    _combobox_render_mode = new QComboBox(this);
    _combobox_render_mode->setVisible(false);
    _combobox_render_mode->setToolTip(tr("3D Viewer Render Mode"));
    _combobox_render_mode->setMaximumHeight(25);
    _combobox_render_mode->addItem(tr("Origin"), ColorMode::MODE_ORIGIN);
    _combobox_render_mode->addItem(tr("FieldX"), ColorMode::MODE_FIELD_X);
    _combobox_render_mode->addItem(tr("FieldY"), ColorMode::MODE_FIELD_Y);
    _combobox_render_mode->addItem(tr("FieldZ"), ColorMode::MODE_FIELD_Z);
    _combobox_render_mode->addItem(tr("FieldI"), ColorMode::MODE_FIELD_INTENSITY);
    _combobox_render_mode->addItem(tr("Rand"), ColorMode::MODE_RANDOM);

    connect(_combobox_render_mode, &QComboBox::currentTextChanged, [&](){
        emit this->renderModeChanged(this->_combobox_render_mode->currentData().toInt());});
    _combobox_render_mode->setCurrentIndex(0);
    //_toolbar_view->addWidget(_combobox_render_mode);

    // this->grabKeyboard();
    _root_node = new osg::Group();
    _root_node->getOrCreateStateSet()->setMode(GL_LIGHTING,
        osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
}

OSGViewer::~OSGViewer() {}
void OSGViewer::insertComboBoxToToolBar() {
    if (!_is_insert_combobox) {
        _toolbar_view->addSeparator();
        _combobox_render_mode->setVisible(true);
        _toolbar_view->addWidget(_combobox_render_mode);
    }
}
osgViewer::Viewer* OSGViewer::getOsgViewer() {
    return _render;
}

OpenThreads::ReadWriteMutex* OSGViewer::mutex() {
    return &_osg_mutex;
}
OSGViewer::ColorMode OSGViewer::getColorMode() const {
    return ColorMode(_combobox_render_mode->currentData().toInt());
}
QSize OSGViewer::sizeHint() const {
    // return QSize(this->width(), this->height());
    // qDebug() <<"size hint: " << this->width() << this->height();
    return QSize(640, 480);
}

void OSGViewer::initializeGL() {
    // from QOpenGLFunctions
    // Initializes OpenGL function resolution for the current context.
    this->initializeOpenGLFunctions();
    // 创建视口
    this->createRenderer();

    // 设置相机的操作方式
    // osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;
    // keyswitchManipulator->addMatrixManipulator('1', "Trackball", new osgGA::TrackballManipulator());
    // keyswitchManipulator->addMatrixManipulator('2', "Flight", new osgGA::FlightManipulator());
    // keyswitchManipulator->addMatrixManipulator('3', "Drive", new osgGA::DriveManipulator());
    // keyswitchManipulator->addMatrixManipulator('4', "Terrain", new osgGA::TerrainManipulator());
    // keyswitchManipulator->addMatrixManipulator('5', "Orbit", new osgGA::OrbitManipulator());
    // keyswitchManipulator->addMatrixManipulator('6', "FirstPerson", new osgGA::FirstPersonManipulator());
    // keyswitchManipulator->addMatrixManipulator('7', "Spherical", new osgGA::SphericalManipulator());
    // keyswitchManipulator->selectMatrixManipulator(2);

    _driver = new DriveManipulator();
    // 关闭拖拽转动
    _driver->setAllowThrow(false);
    // 可以根据指定路径设置动画演示
    // osgGA::AnimationPathManipulator* apm = new osgGA::AnimationPathManipulator(pathfile);
    // apm->setTimeScale(animationSpeed); // speed = 1.0
    // keyswitchManipulator->addMatrixManipulator('8', "Path", apm);
    _render->setCameraManipulator(_driver.get());

    // add the state manipulator
    _render->addEventHandler(new osgGA::StateSetManipulator(_render->getCamera()->getOrCreateStateSet()));
    // add the thread model handler
    _render->addEventHandler(new osgViewer::ThreadingHandler);
    // add the window size toggle handler
    _render->addEventHandler(new osgViewer::WindowSizeHandler);
    // add the stats handler
    _render->addEventHandler(new osgViewer::StatsHandler);
    // add the record camera path handler
    _render->addEventHandler(new osgViewer::RecordCameraPathHandler);
    // add the LOD Scale handler
    _render->addEventHandler(new osgViewer::LODScaleHandler);
    // add the screen capture handler
    _render->addEventHandler(new osgViewer::ScreenCaptureHandler);

    // QString test_filename = "D:/BaiduNetdiskDownload/Model_2023-07-03-110131/LOD/main.osgb";
    // this->loadModel(test_filename);
}

void OSGViewer::resizeGL(int w, int h) {
    QScreen* screen = windowHandle()
                      && windowHandle()->screen() ? windowHandle()->screen() :
                      qApp->screens().front();
    _render->resize(w, h, screen->devicePixelRatio());
}

void OSGViewer::paintGL() {
    static bool is_first_frame = true;
    OpenThreads::ScopedReadLock locker(_osg_mutex);
	if (is_first_frame) {
		is_first_frame = false;
		_render->getCamera()->getGraphicsContext()->setDefaultFboId(defaultFramebufferObject());
	}
	_render->frame();
}

void OSGViewer::keyPressEvent(QKeyEvent* event) {
    // std::cout << "key press" << std::endl;
    _render->keyPressEvent(event);
}

void OSGViewer::keyReleaseEvent(QKeyEvent* event) {
    _render->keyReleaseEvent(event);
}

void OSGViewer::mousePressEvent(QMouseEvent* event) {
    _render->mousePressEvent(event);
}

void OSGViewer::mouseReleaseEvent(QMouseEvent* event) {
    _render->mouseReleaseEvent(event);
}

void OSGViewer::mouseDoubleClickEvent(QMouseEvent* event) {
    _render->mouseDoubleClickEvent(event);
}

void OSGViewer::mouseMoveEvent(QMouseEvent* event) {
    _render->mouseMoveEvent(event);
}

void OSGViewer::wheelEvent(QWheelEvent* event) {
    _render->wheelEvent(event);
}
#ifdef WIN32
void OSGViewer::enterEvent(QEnterEvent *event) {
#else
void OSGViewer::enterEvent(QEvent *event) {
#endif
    this->grabKeyboard();
}
void OSGViewer::leaveEvent(QEvent *event) {
    this->releaseKeyboard();
}
void OSGViewer::showEvent(QShowEvent *event) {
    QOpenGLWidget::showEvent(event);
    this->setCameraPosition(common::Point3d(0, 0, 1000),
        common::Point3d(0, 0, 0), common::Point3d(0, 1, 0));
}
void OSGViewer::setDefaultDisplaySettings() {
    osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
    ds->setNvOptimusEnablement(1);
    ds->setStereo(false);
}

void OSGViewer::createRenderer() {
    // call this before creating a View...
    setDefaultDisplaySettings();
    if (_render) {
        _render->deleteLater();
    }
    _render = new OSGRender(this);
    _render->setWidget(this);
    _render->setSceneData(_root_node.get());
    this->resizeGL(this->width(), this->height());
    //
    // _render->getCamera()->setProjectionMatrixAsPerspective(
    //     30.f, this->width() * 1.f / this->height(), 1.0, 1000.f);
}
double OSGViewer::getCameraFovY() const {
    return _render->getCameraFovY();
}
void OSGViewer::setCameraPosition(const common::Point3d& camera_pos,
        const common::Point3d& view_pos, const common::Point3d& up_dir) {
    osg::Vec3d eye(camera_pos.x, camera_pos.y, camera_pos.z);
    osg::Vec3d center(view_pos.x, view_pos.y, view_pos.z);
    osg::Vec3d up(up_dir.x, up_dir.y, up_dir.z);
    if (_driver.get()) {
        _driver->setTransformation(eye, center, up);
    }
}
void OSGViewer::onTopView() {
    _driver->fitInView(osg::Vec3d(0.0, 1.0, 0.0),
        osg::Vec3d(0.0, 0.0, -1.0));
}
void OSGViewer::onBottomView() {
    _driver->fitInView(osg::Vec3d(0.0, 1.0, 0.0),
        osg::Vec3d(0.0, 0.0, 1.0));
}
void OSGViewer::onFrontView() {
    _driver->fitInView(osg::Vec3d(0.0, 0.0, 1.0),
        osg::Vec3d(0.0, 1.0, 0.0));
}
void OSGViewer::onBackView() {
    _driver->fitInView(osg::Vec3d(0.0, 0.0, 1.0),
        osg::Vec3d(0.0, -1.0, 0.0));
}
void OSGViewer::onLeftView() {
    _driver->fitInView(osg::Vec3d(0.0, 0.0, 1.0),
        osg::Vec3d(1.0, 0.0, 0.0));
}
void OSGViewer::onRightView() {
    _driver->fitInView(osg::Vec3d(0.0, 0.0, 1.0),
        osg::Vec3d(-1.0, 0.0, 0.0));
}
void OSGViewer::onChangeClearColor() {
    QColor color = QColorDialog::getColor(
        _render->getClearColor(), this, tr("Select Background Color"));
    _render->setClearColor(color);
}
bool OSGViewer::hasLayer(const std::string& name) const {
    return _layers.find(name) != _layers.end();
}
OSGLayer* OSGViewer::getLayer(const std::string& name) {
    auto iter = _layers.find(name);
    if (iter == _layers.end()) {return nullptr;}
    return iter->second;
}

void OSGViewer::removeLayer(const std::string &name) {
    if (!hasLayer(name)) {return;}
    auto layer = getLayer(name);
    this->getRootNode()->removeChild(layer->getRootNode());
    _layers.erase(name);
}

void OSGViewer::removeAllLayers() {
    std::vector<std::string> names;
    for (auto& pair : _layers) {
        names.push_back(pair.first);
    }
    for (auto& name : names) {
        removeLayer(name);
    }
}
void OSGViewer::removeAllChildren() {
    for (auto& iter : _layers) {
        iter.second->removeAllChildren();
    }
    // 清空缓存
    _render->getDatabasePager()->clear();

    // _render->resizeGLObjectBuffers(1);
    _render->releaseGLObjects();
}
void OSGViewer::resetRootNode() {
    if (_render) {
        _render->setSceneData(_root_node.get());
    }
}
}
