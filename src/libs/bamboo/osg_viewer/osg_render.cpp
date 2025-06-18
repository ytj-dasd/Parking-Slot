#include "osg_render.h"
#include "osg_viewer.h"

// osg
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osg/CoordinateSystemNode>
// qt
#include <QApplication>
#include <QScreen>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>

#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>

#include <QThread>
#include <common/geometry/line3.h>
#include <uface/parameter/uparameter.h>
namespace welkin::bamboo {

class QtKeyboardMap {
public:
    QtKeyboardMap() {
        _key_map[Qt::Key_Escape    ] = osgGA::GUIEventAdapter::KEY_Escape;
        _key_map[Qt::Key_Delete    ] = osgGA::GUIEventAdapter::KEY_Delete;
        _key_map[Qt::Key_Home      ] = osgGA::GUIEventAdapter::KEY_Home;
        _key_map[Qt::Key_Enter     ] = osgGA::GUIEventAdapter::KEY_KP_Enter;
        _key_map[Qt::Key_End       ] = osgGA::GUIEventAdapter::KEY_End;
        _key_map[Qt::Key_Return    ] = osgGA::GUIEventAdapter::KEY_Return;
        _key_map[Qt::Key_PageUp    ] = osgGA::GUIEventAdapter::KEY_Page_Up;
        _key_map[Qt::Key_PageDown  ] = osgGA::GUIEventAdapter::KEY_Page_Down;
        _key_map[Qt::Key_Left      ] = osgGA::GUIEventAdapter::KEY_Left;
        _key_map[Qt::Key_Right     ] = osgGA::GUIEventAdapter::KEY_Right;
        _key_map[Qt::Key_Up        ] = osgGA::GUIEventAdapter::KEY_Up;
        _key_map[Qt::Key_Down      ] = osgGA::GUIEventAdapter::KEY_Down;
        _key_map[Qt::Key_Backspace ] = osgGA::GUIEventAdapter::KEY_BackSpace;
        _key_map[Qt::Key_Tab       ] = osgGA::GUIEventAdapter::KEY_Tab;
        _key_map[Qt::Key_Space     ] = osgGA::GUIEventAdapter::KEY_Space;
        _key_map[Qt::Key_Delete    ] = osgGA::GUIEventAdapter::KEY_Delete;
        _key_map[Qt::Key_Alt       ] = osgGA::GUIEventAdapter::KEY_Alt_L;
        _key_map[Qt::Key_Shift     ] = osgGA::GUIEventAdapter::KEY_Shift_L;
        _key_map[Qt::Key_Control   ] = osgGA::GUIEventAdapter::KEY_Control_L;
        _key_map[Qt::Key_Meta      ] = osgGA::GUIEventAdapter::KEY_Meta_L;
        _key_map[Qt::Key_F1        ] = osgGA::GUIEventAdapter::KEY_F1;
        _key_map[Qt::Key_F2        ] = osgGA::GUIEventAdapter::KEY_F2;
        _key_map[Qt::Key_F3        ] = osgGA::GUIEventAdapter::KEY_F3;
        _key_map[Qt::Key_F4        ] = osgGA::GUIEventAdapter::KEY_F4;
        _key_map[Qt::Key_F5        ] = osgGA::GUIEventAdapter::KEY_F5;
        _key_map[Qt::Key_F6        ] = osgGA::GUIEventAdapter::KEY_F6;
        _key_map[Qt::Key_F7        ] = osgGA::GUIEventAdapter::KEY_F7;
        _key_map[Qt::Key_F8        ] = osgGA::GUIEventAdapter::KEY_F8;
        _key_map[Qt::Key_F9        ] = osgGA::GUIEventAdapter::KEY_F9;
        _key_map[Qt::Key_F10       ] = osgGA::GUIEventAdapter::KEY_F10;
        _key_map[Qt::Key_F11       ] = osgGA::GUIEventAdapter::KEY_F11;
        _key_map[Qt::Key_F12       ] = osgGA::GUIEventAdapter::KEY_F12;
        _key_map[Qt::Key_F13       ] = osgGA::GUIEventAdapter::KEY_F13;
        _key_map[Qt::Key_F14       ] = osgGA::GUIEventAdapter::KEY_F14;
        _key_map[Qt::Key_F15       ] = osgGA::GUIEventAdapter::KEY_F15;
        _key_map[Qt::Key_F16       ] = osgGA::GUIEventAdapter::KEY_F16;
        _key_map[Qt::Key_F17       ] = osgGA::GUIEventAdapter::KEY_F17;
        _key_map[Qt::Key_F18       ] = osgGA::GUIEventAdapter::KEY_F18;
        _key_map[Qt::Key_F19       ] = osgGA::GUIEventAdapter::KEY_F19;
        _key_map[Qt::Key_F20       ] = osgGA::GUIEventAdapter::KEY_F20;
        _key_map[Qt::Key_hyphen    ] = '-';
        _key_map[Qt::Key_Equal     ] = '=';
        _key_map[Qt::Key_division  ] = osgGA::GUIEventAdapter::KEY_KP_Divide;
        _key_map[Qt::Key_multiply  ] = osgGA::GUIEventAdapter::KEY_KP_Multiply;
        _key_map[Qt::Key_Minus     ] = '-';
        _key_map[Qt::Key_Plus      ] = '+';
        _key_map[Qt::Key_Insert    ] = osgGA::GUIEventAdapter::KEY_KP_Insert;
        _key_map[Qt::Key_0         ] = osgGA::GUIEventAdapter::KEY_0;
        _key_map[Qt::Key_1         ] = osgGA::GUIEventAdapter::KEY_1;
        _key_map[Qt::Key_2         ] = osgGA::GUIEventAdapter::KEY_2;
        _key_map[Qt::Key_3         ] = osgGA::GUIEventAdapter::KEY_3;
        _key_map[Qt::Key_4         ] = osgGA::GUIEventAdapter::KEY_4;
        _key_map[Qt::Key_5         ] = osgGA::GUIEventAdapter::KEY_5;
        _key_map[Qt::Key_6         ] = osgGA::GUIEventAdapter::KEY_6;
        _key_map[Qt::Key_7         ] = osgGA::GUIEventAdapter::KEY_7;
        _key_map[Qt::Key_8         ] = osgGA::GUIEventAdapter::KEY_8;
        _key_map[Qt::Key_9         ] = osgGA::GUIEventAdapter::KEY_9;
    }
    ~QtKeyboardMap() {}
    int remapKey(QKeyEvent* event) {
        KeyMap::iterator itr = _key_map.find(event->key());
        if(itr == _key_map.end()) {
            return int(*(event->text().toLatin1().data()));
        } else {
            return itr->second;
        }
    }
    int remapMouse(QMouseEvent* event) {
        switch(event->button()) {
        case Qt::LeftButton: { return 1; }
        case Qt::MiddleButton: { return 2; }
        case Qt::RightButton: { return 3; }
        default: { return 0; }
        }
    }
private:
    typedef std::map<unsigned int, int> KeyMap;
    KeyMap _key_map;
};
static QtKeyboardMap s_key_map;

OSGRender::OSGRender(QObject* parent)
        : QObject(parent), osgViewer::Viewer() {}

OSGRender::~OSGRender() {}

void OSGRender::resize(int width, int height, float scale) {
    _window_scale = scale;
    float wf = width * scale;
    float hf = height * scale;

    // double wf_2 = wf * 0.05 / 2; 
    // double hf_2 = hf * 0.05 / 2; 
    if (_is_initialized) {
        /*  _camera->setViewport(new osg::Viewport(0, 0, wf, hf));*/
        _grahpics_window->resized(0, 0, wf, hf);
        _grahpics_window->getEventQueue()->windowResize(0, 0, wf, hf);
        _camera->setViewport(new osg::Viewport(0, 0, wf, hf));
        _camera->setGraphicsContext(_grahpics_window.get());
        _camera->setProjectionMatrixAsPerspective(
            _fovy, wf / hf, 1.0, 1000.f);
        // _camera->setProjectionMatrixAsOrtho(-wf_2, wf_2, -hf_2, hf_2, 0.5, 1000);
        _osg_viewer->update();
    } else {
        _is_initialized = true;
        _grahpics_window = new osgViewer::GraphicsWindowEmbedded(0, 0, wf, hf);
        // make sure the event queue has the correct window rectangle size and input range
        _grahpics_window->getEventQueue()->syncWindowRectangleWithGraphicsContext();
        _camera->setViewport(new osg::Viewport(0, 0, wf, hf));
        _camera->setGraphicsContext(_grahpics_window.get());
        _camera->setProjectionMatrixAsPerspective(
            _fovy, wf / hf, 1.0, 1000.f);
        // _camera->setProjectionMatrixAsOrtho(-wf_2, wf_2, -hf_2, hf_2, 0.5, 1000);
        // 显示点的大小
        _point = new osg::Point;
        _camera->getOrCreateStateSet()->setAttribute(_point.get());

        _clear_color_index = 0;
        _clear_color_vec.push_back(QColor(  0,   0,   0));
        _clear_color_vec.push_back(QColor( 33,  40,  48));
        _clear_color_vec.push_back(QColor(255, 255, 255));
        _clear_color_vec.push_back(QColor( 51,  51, 102));
        this->updateClearColor();

        osg::DisplaySettings::instance()->setNumOfDatabaseThreadsHint(5);
        // 关闭esc退出
        this->setKeyEventSetsDone(0);
        this->setReleaseContextAtEndOfFrameHint(false);
        // 设置默认为单线程模式
        // SingleThreaded
        // ThreadPerContext
        // ThreadPerCamera
        // CullThreadPerCameraDrawThreadPerContext
        // AutomaticSelection
        this->setThreadingModel(osgViewer::Viewer::AutomaticSelection);
        // paged 缓存的最大数量
        this->getDatabasePager()->setTargetMaximumNumberOfPageLOD(
            UParam("model_viewer_target_max_number_of_pagelod", 70));

        if (!osgDB::Registry::instance()->getOptions()) {
            osgDB::Registry::instance()->setOptions(new osgDB::Options());
        }
        // std::cout << "before: " << osgDB::Registry::instance()->getOptions()->getObjectCacheHint() << std::endl;
        osgDB::Registry::instance()->getOptions()->setObjectCacheHint(osgDB::Options::CACHE_NONE);
        // std::cout << "after: " << osgDB::Registry::instance()->getOptions()->getObjectCacheHint() << std::endl;
        
        // std::cout << "Hint: " << (this->getReleaseContextAtEndOfFrameHint() ? "true" : "flase") << std::endl;
        // this->setReleaseContextAtEndOfFrameHint(true);

        // 获取窗口， 此处添加作用不知
        // osgViewer::Viewer::Windows windows;
        // this->getWindows(windows);

	    // osg::Program* program = new osg::Program;
        // std::stringstream vp;
        // vp << "#version 420 compatibility\n"
        //    << "varying vec4 color;\n"
        //    << "void main(void)\n"
        //    << "{\n"
        //    << "    float g = (gl_Vertex[2] - 5.0) * 0.05;\n"
        //    << "    g = min(max(g, 0.0), 1.0);\n"
        //    << "    color = vec4(1 - g, g, 0, 1);\n"
        //    << "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
        //    << "}\n";
	    // program->addShader(new osg::Shader(osg::Shader::VERTEX, vp.str()));
        // std::stringstream fp;
        // fp << "#version 420 compatibility\n"
        //    << "varying vec4 color;\n"
        //    << "void main(void)\n"
        //    << "{\n"
        //    << "    gl_FragColor = color;\n"
        //    << "}\n"
        //    << "\n";
        // program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fp.str()));
	    // _root_node->getOrCreateStateSet()->setAttributeAndModes(program,
        //     osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
        // 时间间隔，单位ms,定时触发timerEvent
        _timer_id = startTimer(10, Qt::PreciseTimer);
        _last_frame_start_time.setStartTick(0);
    }
}
void OSGRender::setPickMode(int mode) {
    _pick_mode = PickMode(mode);
}
void OSGRender::setClearColor(const QColor& color) {
    if (!color.isValid()) {return;}
    _camera->setClearColor(osg::Vec4(
        color.redF(), color.greenF(),
        color.blueF(), 1.0));
}
void OSGRender::updateClearColor() {
    ++_clear_color_index;
    if (_clear_color_index < 0
            || _clear_color_index >= _clear_color_vec.size()) {
        _clear_color_index = 0;
    }
    this->setClearColor(_clear_color_vec[_clear_color_index]);
}
void OSGRender::setCameraFoyY(double fovy) {
    RETURN_IF(std::fabs(fovy - _fovy) < 1e-1)
    double fovy_tmp, aspectRatio, zNear, zFar;
    if (!_camera->getProjectionMatrixAsPerspective(
            fovy_tmp, aspectRatio, zNear, zFar)) {
        _fovy = fovy;
        _camera->setProjectionMatrixAsPerspective(
            _fovy, aspectRatio, zNear, zFar);
    }
}
double OSGRender::getCameraFovY() const {
    return _fovy * M_PI / 180.0;
    // double fovy, aspectRatio, zNear, zFar;
    // if (!_camera->getProjectionMatrixAsPerspective(
    //         fovy, aspectRatio, zNear, zFar)) {
    //     fovy = 30.0;
    // }
    // return fovy * M_PI / 180.0;
}
void OSGRender::setKeyboardModifiers(QInputEvent* event) {
    unsigned int modkey = event->modifiers()
        & (Qt::ShiftModifier | Qt::ControlModifier | Qt::AltModifier);
    unsigned int mask = 0;
    if(modkey & Qt::ShiftModifier) {
        mask |= osgGA::GUIEventAdapter::MODKEY_SHIFT;
    }
    if(modkey & Qt::ControlModifier) {
        mask |= osgGA::GUIEventAdapter::MODKEY_CTRL;
    }
    if(modkey & Qt::AltModifier) {
        mask |= osgGA::GUIEventAdapter::MODKEY_ALT;
    }
    _grahpics_window->getEventQueue()->getCurrentEventState()->setModKeyMask(mask);
}

void OSGRender::keyPressEvent(QKeyEvent* event) {
    switch (event->key()) {
    case Qt::Key_Plus : {
        _point->setSize(_point->getSize() + 0.1);
        UInfo(tr("Point Size: %1").arg(_point->getSize()));
        break;
    }
    case Qt::Key_Underscore: {
        float s = _point->getSize() - 0.1;
        if (s > 0) {_point->setSize(s);}
        UInfo(tr("Point Size: %1").arg(_point->getSize()));
        break;
    }
    case Qt::Key_B: {
        this->updateClearColor();
        break;
    }
    default:
        break;
    }

    this->setKeyboardModifiers(event);
    int value = s_key_map.remapKey(event);
    _grahpics_window->getEventQueue()->keyPress(value);
}

void OSGRender::keyReleaseEvent(QKeyEvent* event) {
    if(event->isAutoRepeat()) {
        event->ignore();
    } else {
        this->setKeyboardModifiers(event);
        int value = s_key_map.remapKey(event);
        _grahpics_window->getEventQueue()->keyRelease(value);
    }
}

void OSGRender::mousePressEvent(QMouseEvent* event) {
    this->setKeyboardModifiers(event);
    int button = s_key_map.remapMouse(event);
    _grahpics_window->getEventQueue()->mouseButtonPress(
        event->pos().x() * _window_scale, event->pos().y() * _window_scale, button);
}

void OSGRender::mouseReleaseEvent(QMouseEvent* event) {
    this->setKeyboardModifiers(event);
    int button = s_key_map.remapMouse(event);
    _grahpics_window->getEventQueue()->mouseButtonRelease(
        event->pos().x() * _window_scale, event->pos().y() * _window_scale, button);
    
    // 
    if (event->button() == Qt::LeftButton) {
        osg::Matrixd Matrix_V = _camera->getViewMatrix();
        osg::Matrixd Matrix_P = _camera->getProjectionMatrix();
        osg::Matrixd Matrix_W = _camera->getViewport()->computeWindowMatrix();
        double height = _camera->getViewport()->height();
        // 左乘
        osg::Matrixd vwp = Matrix_V * Matrix_P * Matrix_W;
        osg::Matrixd inverseVPW = osg::Matrixd::inverse(vwp);
        double px = event->pos().x() * _window_scale;
        // 默认Y向上
        double py = (height - 1 - event->pos().y()) * _window_scale;
        osg::Vec3d pt_s = osg::Vec3d(px, py, 0) * inverseVPW;
        osg::Vec3d pt_e = osg::Vec3d(px, py, 1) * inverseVPW;
        if (event->modifiers() & Qt::ControlModifier) {
            emit _osg_viewer->intersectLinePicked(
                pt_s[0], pt_s[1], pt_s[2], pt_e[0], pt_e[1], pt_e[2]);
        }
    }
}

void OSGRender::mouseDoubleClickEvent(QMouseEvent* event) {
    this->setKeyboardModifiers(event);
    int button = s_key_map.remapMouse(event);
    _grahpics_window->getEventQueue()->mouseDoubleButtonPress(
        event->pos().x() * _window_scale, event->pos().y() * _window_scale, button);
}

void OSGRender::mouseMoveEvent(QMouseEvent* event) {
    this->setKeyboardModifiers(event);
    _grahpics_window->getEventQueue()->mouseMotion(
        event->pos().x() * _window_scale, event->pos().y() * _window_scale);
}

void OSGRender::wheelEvent(QWheelEvent* event) {
    this->setKeyboardModifiers(event);
    _grahpics_window->getEventQueue()->mouseMotion(
        event->position().x() * _window_scale, event->position().y() * _window_scale);
    // 正常鼠标不带横向滚轮的
    _grahpics_window->getEventQueue()->mouseScroll(
        event->angleDelta().y() < 0 ? osgGA::GUIEventAdapter::SCROLL_UP :
        osgGA::GUIEventAdapter::SCROLL_DOWN);
        // (event->angleDelta().x() > 0 ? osgGA::GUIEventAdapter::SCROLL_LEFT :
        //  osgGA::GUIEventAdapter::SCROLL_RIGHT));
}

bool OSGRender::checkEvents() {
    // check events from any attached sources
    for (Devices::iterator eitr = _eventSources.begin();
            eitr != _eventSources.end(); ++eitr) {
        osgGA::Device* es = eitr->get();
        if (es->getCapabilities() & osgGA::Device::RECEIVE_EVENTS) {
            if(es->checkEvents()) {
                return true;
            }
        }
    }
    // get events from all windows attached to Viewer.
    Windows windows;
    this->getWindows(windows);
    for(Windows::iterator witr = windows.begin();
        witr != windows.end(); ++witr) {
        if((*witr)->checkEvents()) {
            return true;
        }
    }
    return false;
}

bool OSGRender::checkNeedToDoFrame() {
    // check if any event handler has prompted a redraw
    if(_requestRedraw) {return true;}
    if(_requestContinousUpdate) {return true;}

    // check if the view needs to update the scene graph
    // this check if camera has update callback and if scene requires to update scene graph
    if(this->requiresUpdateSceneGraph()) {return true;}
    // check if the database pager needs to update the scene
    if(this->getDatabasePager()->requiresUpdateSceneGraph()) {return true;}
    // check if the image pager needs to update the scene
    if(this->getImagePager()->requiresUpdateSceneGraph()) {return true;}
    // check if the scene needs to be redrawn
    if(this->requiresRedraw()) {return true;}
    // check if events are available and need processing
    if(this->checkEvents()) {return true;}
    // and check again if any event handler has prompted a redraw
    if(_requestRedraw) {return true;}
    if(_requestContinousUpdate) {return true;}
    return false;
}

// called from ViewerWidget paintGL() method
void OSGRender::frame(double simulationTime) {
    // limit the frame rate
    if(getRunMaxFrameRate() > 0.0) {
        double dt = _last_frame_start_time.time_s();
        double period = 1.0 / getRunMaxFrameRate();
        if(dt < period) {
            QThread::usleep(static_cast<unsigned int>(1000000.0 * (period - dt)));
        }
    }
    // avoid excessive CPU loading when no frame is required in ON_DEMAND mode
    if(getRunFrameScheme() == osgViewer::ViewerBase::ON_DEMAND) {
        double dt = _last_frame_start_time.time_s();
        if(dt < 0.01) { // 10ms
            OpenThreads::Thread::microSleep(
                static_cast<unsigned int>(1000000.0 * (0.01 - dt)));
        }
    }
    // record start frame time
    _last_frame_start_time.setStartTick();
    // make frame
    osgViewer::Viewer::frame(simulationTime);
}

void OSGRender::requestRedraw() {
    osgViewer::Viewer::requestRedraw();
}

void OSGRender::timerEvent(QTimerEvent* /*event*/) {
    // ask ViewerWidget to update 3D view
    if (getRunFrameScheme() != osgViewer::ViewerBase::ON_DEMAND ||
           checkNeedToDoFrame()) {
        _osg_viewer->update();
    }
}
}
