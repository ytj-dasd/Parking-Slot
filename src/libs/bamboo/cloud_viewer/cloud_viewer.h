#pragma once
#define PCL_NO_PRECOMPILE
#include "common/base/color.h"
#include "common/geometry/pose3.h"
#include "common/geometry/line3.h"
#include "common/geometry/polygon2.h"
#include "common/geometry/polygon3.h"
#include "common/geometry/polyline2.h"
#include "common/geometry/polyline3.h"
#include "bamboo/base/macro.h"
#include "bamboo/base/types.h"
#include "uface/base/uconverter.h"

#include <QWidget>
#include <vtkVersionMacros.h>
#if VTK_MAJOR_VERSION >= 9
#include <QVTKOpenGLNativeWidget.h>
#else
#include <QVTKWidget.h>
#endif
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace welkin::bamboo {
#if VTK_MAJOR_VERSION >= 9
using VTKWidget = QVTKOpenGLNativeWidget;
#else
using VTKWidget = QVTKWidget;
#endif

class BAMBOO_EXPORT CloudViewer : public VTKWidget {
    Q_OBJECT
public:
    explicit CloudViewer(QWidget* parent =  nullptr);
    virtual ~CloudViewer();

    bool hasCloud(const QString& cloud_id);
    bool hasItem(const QString& item_id);

    bool updatePose(const QString& item_id,
                    const common::Pose3d& pose);
    bool updatePose(const QString& item_id,
                    const Eigen::Matrix4d& matrix);

    bool updateShapePose(const QString& item_id,
                    const common::Pose3d& pose);
    bool updateShapePose(const QString& item_id,
                    const Eigen::Matrix4d& matrix);

    enum ColorMode {
        COLOR_ORIGIN = 0, COLOR_FIELD_X = 1,
        COLOR_FIELD_Y = 2, COLOR_FIELD_Z = 3,
        COLOR_FIELD_INTENSITY = 4};

    using PointXColorHandlerRGBField 
        = pcl::visualization::PointCloudColorHandlerRGBField<PointX>;
    using PointXColorHandlerGenericField
        = pcl::visualization::PointCloudColorHandlerGenericField<PointX>;

    bool updateCloud(const QString& cloud_id, const PointCloudPtr& cloud);

    bool removeShape(const QString& shape_id);
    bool removeCloud(const QString& cloud_id);
    bool removeAllClouds();
    bool removeAllShapes();

    bool updateLine(const QString& cloud_id,
        const common::Line3d& line, QColor color);

    bool updatePolyline(const QString& polyline_id,
        const common::Polyline3d& polyline, QColor color, bool is_closed = false);
    bool updatePolyline(const QString& polyline_id,
        const common::Polyline2d& polyline,
        const double min_z, const double max_z, QColor color);
    bool updatePolyline(const QString& polyline_id,
        const common::Polyline2d& polyline,
        const double z, QColor color);

    bool updateBox(const QString& cloud_id,
        const common::Boxd& box, QColor color);

    void setCameraPosition(const common::Point3d& camera_pos,
            const common::Point3d& view_pos,
            const common::Point3d& up_dir);
    
    // 单位弧度
    double getCameraFovY() const;

signals:
    void colorModeChanged();
public slots:
    void removeAllItems();
    void setBackgroundColor(const QColor& color);
    void setColorMode(int mode);
private slots:
    void onChangedBackgroundColor();
protected:
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;
private:
    void updateHelper();
    void readSettings();
    void writeSettings();
private:
    QColor _bg_color = QColor(255, 255, 255);
    std::shared_ptr<pcl::visualization::PCLVisualizer> _pcl_viewer;
    
    ColorMode _color_mode = COLOR_ORIGIN;
};
}
