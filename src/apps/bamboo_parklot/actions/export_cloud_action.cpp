#include "export_cloud_action.h"
#include <common/base/dir.h>
#include <common/geometry/polygon2.h>
#include <common/geometry/polygon3.h>
#include <ucanvas/item.hpp>

#include "../main_viewer.h"

namespace welkin::bamboo {
ExportCloudAction::ExportCloudAction(BProjectWidget* project_widget) 
        : BProjectAction(project_widget, true) {
    this->setText(tr("Export cloud file"));
    this->setToolTip(tr("Export cloud file"));
    this->setIcon(QIcon(":/bamboo/images/export_cloud.png"));
    _enable_progress = true;
}
ExportCloudAction::~ExportCloudAction() {}

void ExportCloudAction::setMainViewer(MainViewer* main_viewer) {
    _main_viewer = main_viewer;
}
bool ExportCloudAction::isDataPrepared() {
    return _main_viewer->shapeLayer()->itemCount() > 0;
}
bool ExportCloudAction::prepareParameter() {
    _save_path = UWidget::GetExistingDirectory(
        tr("Save path"), _save_path);
    return !_save_path.isEmpty();
}

void ExportCloudAction::process() {
    auto project = this->getProject();
    auto local_pose_filer = project->getLocalPoseFiler();
    common::Pose3d pose_trans = local_pose_filer->load();
    Eigen::Matrix4d matrix_trans = pose_trans.toMatrix();
    auto origin_folder = this->getProject()->getOriginDataFolder();
    int index = 0;
    auto save_path_str = UFQ(_save_path);
    for (const auto& group : _main_viewer->getParkSpaceGroupVec()) {
        // 无效的跳过
        if (!group.valid) {continue;}
        common::Rectd rect = group.getBoudingRect();
        rect.addPadding(0.5, 0.5);
        common::Polygon2d pg2d;
        pg2d.points = rect.getCorners();
        auto cloud = origin_folder->getCloudListFolder()->extractCloudInPolygon(pg2d);
        common::point_cloud::RangeZFilter<PointX>(*cloud, common::Rangef(-1.0, 2.0));
        pcl::transformPointCloud(*cloud, *cloud, matrix_trans);
        auto cloud_path = save_path_str + "/" + std::to_string(index) + ".pcd";
        pcl::io::savePCDFile(cloud_path, *cloud, true);
        UProgressTextValue(tr("Export cloud progress: "), index, 100);
        ++index;
    }
    UInfo("Export cloud successfully!");
    return;
}
}
