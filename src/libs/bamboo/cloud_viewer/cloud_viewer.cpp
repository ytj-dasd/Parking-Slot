#include "cloud_viewer.h"
#include <vtkRenderWindow.h>
#include <QKeyEvent>
#if VTK_MAJOR_VERSION >= 9
#include <QVTKRenderWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#endif
#include <iostream>
#include <QSettings>
#include <QAction>
#include <QColorDialog>
#include <uface/base/uconverter.h>
#include <uface/logger/ulogger.h>
#include <uface/resource/uresource.h>
namespace welkin::bamboo {
CloudViewer::CloudViewer(QWidget *parent)
        : VTKWidget(parent) {
    this->readSettings();
    this->setWindowTitle(tr("CloudViewer"));
#if VTK_MAJOR_VERSION >= 9
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto render_window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    render_window->AddRenderer(renderer);
    _pcl_viewer.reset(new pcl::visualization::PCLVisualizer(renderer, render_window, "CloudViewer", false));
    // _pcl_viewer->addCoordinateSystem();
    this->setRenderWindow(_pcl_viewer->getRenderWindow());
    _pcl_viewer->setupInteractor(this->interactor(), this->renderWindow());
#else
    _pcl_viewer.reset(new pcl::visualization::PCLVisualizer("CloudViewer", false));
    _pcl_viewer->addCoordinateSystem(1.0, "refrence", 0);
    this->SetRenderWindow(_pcl_viewer->getRenderWindow());
    _pcl_viewer->setupInteractor(this->GetInteractor(), this->GetRenderWindow());
#endif
    this->setBackgroundColor(_bg_color);
    // 初始化位置
    this->setCameraPosition(common::Point3d(0, 0, 80),
        common::Point3d(0, 0, 0), common::Point3d(0, 1, 0));
}

CloudViewer::~CloudViewer() {
    this->writeSettings();
}
void CloudViewer::setBackgroundColor(const QColor& color) {
    if (!color.isValid()) {return;}
    _bg_color = color;
    _pcl_viewer->setBackgroundColor(
        _bg_color.redF(), _bg_color.greenF(), _bg_color.blueF());
    this->updateHelper();
}
void CloudViewer::onChangedBackgroundColor() {
    QColor color = QColorDialog::getColor(
        _bg_color, this, tr("Select Background Color"));
    this->setBackgroundColor(color);
}
bool CloudViewer::hasCloud(const QString& cloud_id) {
    return _pcl_viewer->contains(UFQ(cloud_id));
}
bool CloudViewer::hasItem(const QString& item_id) {
    return _pcl_viewer->contains(UFQ(item_id));
}
bool CloudViewer::updatePose(const QString& item_id,
        const common::Pose3d& pose) {
    return this->updatePose(item_id, pose.toMatrix());
}
bool CloudViewer::updatePose(const QString& item_id,
        const Eigen::Matrix4d& matrix) {
    std::string std_cloud_id = UFQ(item_id);
    Eigen::Affine3f affine_pose;
    affine_pose.matrix() = matrix.cast<float>();
    if (!_pcl_viewer->updatePointCloudPose(std_cloud_id, affine_pose)) {
        return false;
    }
    this->updateHelper();
    return true;
}
bool CloudViewer::updateShapePose(const QString& item_id,
        const common::Pose3d& pose) {
    return this->updateShapePose(item_id, pose.toMatrix());
}
bool CloudViewer::updateShapePose(const QString& item_id,
        const Eigen::Matrix4d& matrix) {
    std::string std_cloud_id = UFQ(item_id);
    Eigen::Affine3f affine_pose;
    affine_pose.matrix() = matrix.cast<float>();
    if (!_pcl_viewer->updateShapePose(std_cloud_id, affine_pose)) {
        return false;
    }
    this->updateHelper();
    return true;
}

bool CloudViewer::removeShape(const QString& shape_id) {
    std::string std_shape_id = UFQ(shape_id);
    if (!_pcl_viewer->removeShape(std_shape_id)) {
        return false;
    }
    this->updateHelper();
    return true;
}
bool CloudViewer::removeCloud(const QString& cloud_id) {
    std::string std_cloud_id = UFQ(cloud_id);
    if (!_pcl_viewer->removePointCloud(std_cloud_id)) {
        return false;
    }
    this->updateHelper();
    return true;
}
bool CloudViewer::removeAllClouds() {
    if (!_pcl_viewer->removeAllPointClouds()) {
        return false;
    }
    this->updateHelper();
    return true;
}
bool CloudViewer::removeAllShapes() {
    if (!_pcl_viewer->removeAllShapes()) {
        return false;
    }
    this->updateHelper();
    return true;
}
bool CloudViewer::updateCloud(const QString& cloud_id, const PointCloudPtr& cloud) {
    if (!cloud.get()) {return false;}
    std::string std_cloud_id = UFQ(cloud_id);
    if (_color_mode == COLOR_ORIGIN) {
        PointXColorHandlerRGBField color_handler(cloud);
        if (_pcl_viewer->contains(std_cloud_id)) {
            _pcl_viewer->updatePointCloud<PointX>(cloud, color_handler, std_cloud_id);
        } else {
            _pcl_viewer->addPointCloud<PointX>(cloud, color_handler, std_cloud_id);
        }
    } else {
        std::shared_ptr<PointXColorHandlerGenericField> color_handle = nullptr;
        if (_color_mode == COLOR_FIELD_X) {
            color_handle.reset(new PointXColorHandlerGenericField(cloud, "x"));
        } else if (_color_mode == COLOR_FIELD_Y) {
            color_handle.reset(new PointXColorHandlerGenericField(cloud, "y"));
        } else if (_color_mode == COLOR_FIELD_Z) {
            color_handle.reset(new PointXColorHandlerGenericField(cloud, "z"));
        } else if (_color_mode == COLOR_FIELD_INTENSITY) {
            color_handle.reset(new PointXColorHandlerGenericField(cloud, "intensity"));
        } else {
            color_handle.reset(new PointXColorHandlerGenericField(cloud, "z"));
        }
        if (_pcl_viewer->contains(std_cloud_id)) {
            _pcl_viewer->updatePointCloud<PointX>(cloud, *color_handle, std_cloud_id);
        } else {
            _pcl_viewer->addPointCloud<PointX>(cloud, *color_handle, std_cloud_id);
        }
    }
    this->updateHelper();
    return true;
}
bool CloudViewer::updateLine(const QString& cloud_id,
        const common::Line3d& line, QColor color) {
    pcl::PointXYZ pt1;
    pt1.x = line.begin_point.x;
    pt1.y = line.begin_point.y;
    pt1.z = line.begin_point.z;
    pcl::PointXYZ pt2;
    pt2.x = line.end_point.x;
    pt2.y = line.end_point.y;
    pt2.z = line.end_point.z;
    std::string std_cloud_id = UFQ(cloud_id);
    if (_pcl_viewer->contains(std_cloud_id)) {
        _pcl_viewer->removeShape(std_cloud_id);
    }
    if (_pcl_viewer->addLine(pt1, pt2,
            color.redF(), color.greenF(), color.blueF(), UFQ(cloud_id))) {
        this->updateHelper();
        return true;
    } else {
        return false;
    }
}
bool CloudViewer::updatePolyline(const QString& polyline_id,
        const common::Polyline3d& polyline, QColor color, bool is_closed) {
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    PointCloud::Ptr cloud_ptr(new PointCloud());
    cloud_ptr->points.reserve(polyline.size() * 2);
    for (int i = 0; i < polyline.size(); ++i) {
        auto& pt = polyline.points[i];
        pcl::PointXYZ pcl_pt;
        pcl_pt.x = pt.x;
        pcl_pt.y = pt.y;
        pcl_pt.z = pt.z;
        cloud_ptr->emplace_back(pcl_pt);
    }
    if (!is_closed) {
        for (int i = polyline.size() - 2; i > 0; --i) {
            auto& pt = polyline.points[i];
            pcl::PointXYZ pcl_pt;
            pcl_pt.x = pt.x;
            pcl_pt.y = pt.y;
            pcl_pt.z = pt.z;
            cloud_ptr->emplace_back(pcl_pt);
        }
    }
    std::string std_cloud_id = UFQ(polyline_id);
    if (_pcl_viewer->contains(std_cloud_id)) {
        _pcl_viewer->removeShape(std_cloud_id);
    }
    if (_pcl_viewer->addPolygon<pcl::PointXYZ>(cloud_ptr,
            color.redF(), color.greenF(), color.blueF(), std_cloud_id)) {
        this->updateHelper();
        return true;
    } else {
        return false;
    }
}
bool CloudViewer::updatePolyline(const QString& polyline_id,
        const common::Polyline2d& polyline,
        const double min_z, const double max_z, QColor color) {
    std::size_t pt_num = polyline.points.size();
    if (pt_num < 1) {return false;}
    common::Polyline3d polyline3d;
    polyline3d.reserve(pt_num * 6);
    for (std::size_t i = 0; i < pt_num; ++i) {
        auto& pti = polyline.points[i];
        polyline3d.addPoint(pti.x, pti.y, max_z);
        polyline3d.addPoint(pti.x, pti.y, min_z);
    }
    for (std::size_t i = pt_num - 2; i > 0; --i) {
        auto& pti = polyline.points[i];
        polyline3d.addPoint(pti.x, pti.y, min_z);
    }
    for (std::size_t i = 0; i < pt_num; ++i) {
        auto& pti = polyline.points[i];
        polyline3d.addPoint(pti.x, pti.y, min_z);
        polyline3d.addPoint(pti.x, pti.y, max_z);
    }
    for (std::size_t i = pt_num - 2; i > 0; --i) {
        auto& pti = polyline.points[i];
        polyline3d.addPoint(pti.x, pti.y, max_z);
    }
    return this->updatePolyline(polyline_id, polyline3d, color);
// template <typename PointT> bool
//        addPolygon (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
//                    double r, double g, double b,
//                    const std::string &id = "polygon", int viewport = 0);
}
bool CloudViewer::updatePolyline(const QString& polyline_id,
        const common::Polyline2d& polyline,
        const double z, QColor color) {
    std::size_t pt_num = polyline.points.size();
    if (pt_num < 1) {return false;}
    common::Polyline3d polyline3d;
    polyline3d.fromPolyline2(polyline, z);
    return this->updatePolyline(polyline_id, polyline3d, color);
}
bool CloudViewer::updateBox(const QString& cloud_id,
        const common::Boxd& box, QColor color) {
    std::string std_cloud_id = UFQ(cloud_id);
    return _pcl_viewer->addCube(box.getMinX(), box.getMaxX(),
        box.getMinY(), box.getMaxY(),
        box.getMinZ(), box.getMaxZ(),
        color.redF(), color.greenF(), color.blueF(), std_cloud_id);
}
void CloudViewer::setCameraPosition(const common::Point3d& camera_pos,
        const common::Point3d& view_pos,
        const common::Point3d& up_dir) {
    _pcl_viewer->setCameraPosition(
            camera_pos.x, camera_pos.y, camera_pos.z,
            view_pos.x, view_pos.y, view_pos.z,
        up_dir.x, up_dir.y, up_dir.z);
}

double CloudViewer::getCameraFovY() const
{
    pcl::visualization::Camera camera_info;
    _pcl_viewer->getCameraParameters(camera_info, 0);
    return camera_info.fovy;
}
void CloudViewer::removeAllItems() {
    _pcl_viewer->removeAllPointClouds();
    _pcl_viewer->removeAllShapes();
}
void CloudViewer::setColorMode(int mode) {
    if (_color_mode != ColorMode(mode)) {
        _color_mode = ColorMode(mode);
        emit colorModeChanged();
        UInfo(tr("Color mode changed!"));
    }
}
void CloudViewer::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_1: {

        break;
    }
    case Qt::Key_2:

        break;
    default:
        break;
    }

    VTKWidget::keyPressEvent(event);
}

void CloudViewer::keyReleaseEvent(QKeyEvent *event)
{

    VTKWidget::keyReleaseEvent(event);
}
//  bool
//  addCube (const pcl::ModelCoefficients &coefficients,
//           const std::string &id = "cube",
//           int viewport = 0);
// bool
// addCube (float x_min, float x_max, float y_min, float y_max, float z_min, float z_max,
//          double r = 1.0, double g = 1.0, double b = 1.0, const std::string &id = "cube", int viewport = 0);
void CloudViewer::updateHelper() {
    this->update();
#if VTK_MAJOR_VERSION >= 9
    this->updateSize();
#endif
}

void CloudViewer::readSettings() {
    QSettings settings("bamboo", "bamboo");
    settings.beginGroup("cloud_viewer");
    _bg_color = settings.value("bg_color", QColor(255, 255, 255)).value<QColor>();
    settings.endGroup();
}

void CloudViewer::writeSettings() {
    QSettings settings("bamboo", "bamboo");
    settings.beginGroup("cloud_viewer");
    settings.setValue("bg_color", _bg_color);
    settings.endGroup();
}
}
