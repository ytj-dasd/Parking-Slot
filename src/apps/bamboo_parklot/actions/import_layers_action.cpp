#include "import_layers_action.h"
#include "bamboo/file_system/terrain_folder.h"
#include <bamboo/io/dxf_io.h>
#include "import_layers_dialog.h"
#include "../main_viewer.h"
#include <QFileInfo>
#include <QDebug>

namespace welkin::bamboo {
ImportLayersAction::ImportLayersAction(BProjectWidget* project_widget)
        : BProjectAction(project_widget, true) {
    this->setText(tr("Import Layers"));
    this->setToolTip(tr("Import Layers"));
    this->setIcon(QIcon(":/bamboo/images/layer.png"));
    _enable_progress = false;
}
ImportLayersAction::~ImportLayersAction() {}
void ImportLayersAction::setMainViewer(MainViewer* main_viewer) {
    _main_viewer = main_viewer;
}
bool ImportLayersAction::isDataPrepared() {
    auto dom_folder = this->getProject()->getDOMFolder();
    return dom_folder->isValid();
}
bool ImportLayersAction::prepareParameter() {
    auto project = this->getProject();
    //
    ImportLayersDialog dlg;
    auto local_pose_filer = project->getLocalPoseFiler();
    dlg.setLocalPose(local_pose_filer->load());
    auto global_pose_filer = project->getGlobalPoseFiler();
    dlg.setGlobalPose(global_pose_filer->load());
    auto proj_path = project->getPath();
    dlg.setHistoryPath(UTQ(proj_path));
    if (dlg.exec() != QDialog::Accepted) {
        return false;
    }
    _layer_filepaths = dlg.getLayerFilePaths();
    _pose_trans = dlg.getPose3d();
    if (_layer_filepaths.isEmpty()) {
        UWarn(tr("Import layer filename list cannot empty!"));
        return false;
    }
    return true;
}
void ImportLayersAction::process() {
    auto filepath = _layer_filepaths.front();
    QFileInfo fi(filepath);
    auto suffix = fi.completeSuffix();
    auto matrix_trans_inv = _pose_trans.inverse().toMatrix();
    auto shape_layer = _main_viewer->shapeLayer();
    shape_layer->removeAllItems();
    if (suffix == "dxf" || suffix == "dwg") {
        for (auto& filepath : _layer_filepaths) {
            QFileInfo fi(filepath);
            // 打开dxf文件
            DxfInput dxf_file;
            if (!dxf_file.open(UFQ(filepath))) {continue;}
            common::Layerd layerdata;
            layerdata.points = dxf_file.getPointVec();
            layerdata.polylines = dxf_file.getPolylineVec();
            layerdata.polygons = dxf_file.getPolygonVec();
            // 取反
            layerdata.transform(matrix_trans_inv);
            for (auto& polyline : layerdata.polylines) {
                if (polyline.size() != 4) {continue;}
                auto pg = UTQ(polyline.points);
                shape_layer->addPolygon(pg);
            }
            for (auto& polygon : layerdata.polygons) {
                if (polygon.size() != 4) {continue;}
                auto pg = UTQ(polygon.points);
                shape_layer->addPolygon(pg);
            }
            dxf_file.close();
        }
    } else {
        UWarn(tr("Error layer file format: %1").arg(suffix));
    }
    return;
}
}
