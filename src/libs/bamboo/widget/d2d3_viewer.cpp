#include "d2d3_viewer.h"

namespace welkin::bamboo {

D2D3Viewer::D2D3Viewer(QWidget *parent)
        : QSplitter(parent) {
    _cloud_viewer = new CloudViewer(this);
    _image_viewer = new uface::canvas2d::Viewer(this);
    // 设置图元不可选择
    _image_viewer->setItemUnSelectable();

    // 将Y反向
    _image_viewer->setYFlipped(true);
    // this->setStretchFactor(0, 2);
    // this->setStretchFactor(0, 2);
    this->setSizes(QList<int>() << 100 << 100);
    //
    connect(_image_viewer, SIGNAL(sceneMoved(const QPointF&, double)),
            this, SLOT(onSceneMoved(const QPointF&, double)));
}

D2D3Viewer::~D2D3Viewer() {

}
void D2D3Viewer::setProjectPose(const common::Pose3d& pose) {
    _project_pose = pose;
}
void D2D3Viewer::setSceneZValue(qreal z) {
    _scene_z = z;
}

void D2D3Viewer::setEnableInteraction(bool enable) {
    _enable_interaction = enable;
}
void D2D3Viewer::onFitInView() {
    _image_viewer->onFitVisibleItemsInView();
    auto cnt_pt = _image_viewer->getViewCenter();
    double scale = _image_viewer->getViewScale();
    this->onSceneMoved(cnt_pt, scale);
}
void D2D3Viewer::onFitInView(const QRectF& brect) {
    _image_viewer->onFitInView(brect);
    auto cnt_pt = _image_viewer->getViewCenter();
    double scale = _image_viewer->getViewScale();
    this->onSceneMoved(cnt_pt, scale);
}

void D2D3Viewer::removeAllItems() {
    _image_viewer->removeAllLayers();
    _cloud_viewer->removeAllItems();
}
void D2D3Viewer::onSceneMoved(const QPointF& cnt_pt, double scale) {
    if (!_enable_interaction) {return;}

    double view_height = _cloud_viewer->height();
    double fovy = _cloud_viewer->getCameraFovY();
    double cam_height = (view_height * scale * 0.5) / std::tan(fovy / 2);

    auto mat_pose = _project_pose.toMatrix();
    common::Point3d obj_pt(cnt_pt.x(), cnt_pt.y(), _scene_z);
    common::Point3d cam_pt(cnt_pt.x(), cnt_pt.y(), _scene_z + cam_height);
    obj_pt.transform(mat_pose);
    cam_pt.transform(mat_pose);
    common::Point3d updir_pt(mat_pose(0, 1), mat_pose(1, 1), mat_pose(2, 1));

    _cloud_viewer->setCameraPosition(cam_pt, obj_pt, updir_pt);

    if (!_cloud_viewer->hasItem("cross_line_x")) {
        _cloud_viewer->updateLine(QString("cross_line_x"),
            common::Line3d(-0.25, 0, 0, 0.25, 0, 0),
            QColor(255, 0, 0));
    }
    if (!_cloud_viewer->hasItem("cross_line_y")) {
        _cloud_viewer->updateLine(QString("cross_line_y"),
            common::Line3d(0, -0.25, 0, 0, 0.25, 0),
            QColor(0, 255, 0));
    }
    Eigen::Matrix4d mat_trans; mat_trans.setIdentity();
    mat_trans(0, 3) = cnt_pt.x(); mat_trans(1, 3) = cnt_pt.y();
    Eigen::Matrix4d mat_cross = mat_pose * mat_trans;
    _cloud_viewer->updateShapePose("cross_line_x", mat_cross);
    _cloud_viewer->updateShapePose("cross_line_y", mat_cross);
}
}
