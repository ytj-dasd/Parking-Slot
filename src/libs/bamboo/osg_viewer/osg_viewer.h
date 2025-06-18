#pragma once
#include "bamboo/base/macro.h"
#include <common/geometry/point3.h>
// #include <osgQOpenGL/Export>
#include <OpenThreads/ReadWriteMutex>

#ifdef WIN32
//#define __gl_h_
#include <osg/GL>
#endif

#include <osg/ArgumentParser>
#include <QToolBar>
#include <QComboBox>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QReadWriteLock>

#include <osgGA/TrackballManipulator>
#include "drive_manipulator.h"

#include "bamboo/base/osg_utils.h"
#include "bamboo/osg_viewer/osg_layer.h"
namespace osgViewer {
    class Viewer;
}

namespace welkin::bamboo {

class OSGRender;
class BAMBOO_EXPORT OSGViewer : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    explicit OSGViewer(QWidget* parent = nullptr);
    virtual ~OSGViewer();

    /** Get osgViewer View */
    virtual osgViewer::Viewer* getOsgViewer();

    //! get mutex
    virtual OpenThreads::ReadWriteMutex* mutex();

    //! override this to change default size or aspect ratio
    QSize sizeHint() const override;

    QToolBar* getToolBar() {return _toolbar_view;}
    using ColorMode = point_cloud::ColorRender::Mode;
    ColorMode getColorMode() const;
    QComboBox* getColorModeComboBox() {return _combobox_render_mode;}
    //
    void insertComboBoxToToolBar();
    // 单位弧度
    double getCameraFovY() const;
    void setCameraPosition(const common::Point3d& camera_pos,
        const common::Point3d& view_pos, const common::Point3d& up_dir);

    osg::Group* getRootNode() {return _root_node.get();}
    const osg::Group* getRootNode() const {return _root_node.get();}
    bool hasLayer(const std::string& name) const;
    OSGLayer* getLayer(const std::string& name);

    template<typename Layer>
    Layer* getTypeLayer(const std::string& name) {
        OSGLayer* layer = this->getLayer(name);
        if (!layer) {return nullptr;}
        return static_cast<Layer*>(layer);
    }
    template<typename Layer>
    Layer* createLayer(const std::string& name) {
        if (!this->hasLayer(name)) {
            _layers[name] = new Layer(this);
            _layers[name]->setName(name);
            _layers[name]->setViewer(this);
            _layers[name]->init();
            this->getRootNode()->addChild(
                _layers[name]->getRootNode());
        }
        return this->getTypeLayer<Layer>(name);
    }
    template <typename Layer>
    void addLayer(Layer* layer) {
        if (!layer || this->hasLayer(layer->getName())) {return;}
        auto name = layer->getName();
        _layers[name] = layer;
        _layers[name]->setViewer(this);
        _layers[name]->init();
        this->getRootNode()->addChild(_layers[name]->getRootNode());
    }
    void removeLayer(const std::string& name);
    void removeAllLayers();
signals:
    void renderModeChanged(int render_mode);
    void intersectLinePicked(qreal x1, qreal y1, qreal z1, qreal x2, qreal y2, qreal z2);

public slots:
    void onTopView();
    void onBottomView();
    void onFrontView();
    void onBackView();
    void onLeftView();
    void onRightView();
    void onChangeClearColor();

    void removeAllChildren();
    void resetRootNode();

protected:
    //! call createRender. If overloaded, this method must send initialized signal at end
    void initializeGL() override;
    void resizeGL(int w, int h) override;

    //! lock scene graph and call osgViewer::frame()
    void paintGL() override;

    //! called before creating renderer
    virtual void setDefaultDisplaySettings();

    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
#ifdef WIN32
    void enterEvent(QEnterEvent *event) override;
#else
    void enterEvent(QEvent *event) override;
#endif
    void leaveEvent(QEvent *event) override;
    void showEvent(QShowEvent *event) override;

    void createRenderer();

protected:
    QAction* _action_top_view = nullptr;
    QAction* _action_bottom_view = nullptr;
    QAction* _action_front_view = nullptr;
    QAction* _action_back_view = nullptr;
    QAction* _action_left_view = nullptr;
    QAction* _action_right_view = nullptr;
    // QAction* _action_change_color = nullptr;
    QComboBox* _combobox_render_mode = nullptr;
    bool _is_insert_combobox = false;
    QToolBar* _toolbar_view = nullptr;

    osg::ref_ptr<DriveManipulator> _driver;

    osg::ref_ptr<osg::Group> _root_node = nullptr;
    std::unordered_map<std::string, OSGLayer*> _layers;

    OSGRender* _render = nullptr;
    OpenThreads::ReadWriteMutex _osg_mutex;
    friend class OSGRenderer;
};
}

