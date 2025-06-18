#pragma once
#include "bamboo/base/macro.h"
// #include <osgQOpenGL/Export>
#include <QColor>
#include <QObject>
#include <osg/Point>
#include <osgViewer/Viewer>

class QInputEvent;
class QKeyEvent;
class QMouseEvent;
class QWheelEvent;

namespace welkin::bamboo {
class OSGViewer;
class BAMBOO_EXPORT OSGRender : public QObject, public osgViewer::Viewer {
    Q_OBJECT
public:
    explicit OSGRender(QObject* parent = nullptr);

    ~OSGRender() override;
    
    enum PickMode {
        PICK_NONE = 0,
        PICK_POINT = 1
    };

    virtual void keyPressEvent(QKeyEvent* event);
    virtual void keyReleaseEvent(QKeyEvent* event);
    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void mouseDoubleClickEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);
    virtual void wheelEvent(QWheelEvent* event);

    virtual void resize(int width, int height, float scale);

    // overrided from osgViewer::Viewer
    virtual bool checkNeedToDoFrame() override;

    // overrided from osgViewer::ViewerBase
    void frame(double simulationTime = USE_REFERENCE_TIME) override;

    // overrided from osgViewer::Viewer
    void requestRedraw() override;
    // overrided from osgViewer::Viewer
    bool checkEvents() override;

    void setWidget(OSGViewer* osg_viewer) {_osg_viewer = osg_viewer;}
    
    double getCameraFovY() const;

    QColor getClearColor() const {
        return _clear_color_vec[_clear_color_index];}
    
    PickMode getPickMode() const {return _pick_mode;}
public slots:
    // degree
    void setCameraFoyY(double fovy);
    void setPickMode(int mode);
    void setClearColor(const QColor& color);
    void updateClearColor();
protected:
    bool initRootNode();
    // 继承自QObject
    void timerEvent(QTimerEvent* event) override;

    void setKeyboardModifiers(QInputEvent* event);

protected:
    bool _is_initialized = false;
    double _window_scale = 1.f;
    double _fovy = 30.0;
    osg::ref_ptr<osgViewer::GraphicsWindow> _grahpics_window;
    OSGViewer* _osg_viewer = nullptr;

    int _timer_id = 0;
    osg::Timer _last_frame_start_time;

    osg::ref_ptr<osg::Point> _point = nullptr;
    
    int _clear_color_index = 0;
    std::vector<QColor> _clear_color_vec;   

    PickMode _pick_mode = PICK_NONE; 
};
}

