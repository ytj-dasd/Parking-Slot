#pragma once
#include <QSplitter>
#include <uface/canvas2d/view.hpp>
#include "bamboo/cloud_viewer/cloud_viewer.h"

namespace welkin::bamboo {
class BAMBOO_EXPORT D2D3Viewer : public QSplitter {
    Q_OBJECT
public:
    explicit D2D3Viewer(QWidget* parent = nullptr);
    virtual ~D2D3Viewer();

    CloudViewer* getCloudViewer() {return _cloud_viewer;}
    uface::canvas2d::Viewer* getImageViewer() {return _image_viewer;}

    double getSceneZ() const {return _scene_z;}

    void setProjectPose(const common::Pose3d& pose);
public slots:
    void setSceneZValue(qreal z);
    void setEnableInteraction(bool enable);

    void onFitInView();
    void onFitInView(const QRectF& brect);
    void removeAllItems();
private slots:
    void onSceneMoved(const QPointF& cnt_pt, double scale);
    
protected:
    // 图片场景中z值高度
    double _scene_z = 0.0;
    // 是否允许交互
    bool _enable_interaction = true;
    // 投影坐标
    common::Pose3d _project_pose;
    //
    CloudViewer* _cloud_viewer = nullptr;
    uface::canvas2d::Viewer* _image_viewer = nullptr;
};
}
