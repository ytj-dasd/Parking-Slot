#include "import_detection_action.h"
#include "bamboo/file_system/terrain_folder.h"
#include <bamboo/io/dxf_io.h>
#include "import_detection_dialog.h"
#include "../main_viewer.h"
#include <QFileInfo>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonParseError>
#include <QFile>


namespace welkin::bamboo {
ImportDetectionAction::ImportDetectionAction(BProjectWidget* project_widget)
        : BProjectAction(project_widget, true) {
    this->setText(tr("Import Detection Layers"));
    this->setToolTip(tr("Import Detection Layers"));
    this->setIcon(QIcon(":/bamboo/images/detection.png"));
    _enable_progress = false;
}
ImportDetectionAction::~ImportDetectionAction() {}
void ImportDetectionAction::setMainViewer(MainViewer* main_viewer) {
    _main_viewer = main_viewer;
}
bool ImportDetectionAction::isDataPrepared() {
    auto dom_folder = this->getProject()->getDOMFolder();
    return dom_folder->isValid();
}
bool ImportDetectionAction::prepareParameter() {
    auto project = this->getProject();
    //
    ImportDetectionDialog dlg;
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

void ImportDetectionAction::process() {
    auto filepath = _layer_filepaths.front();
    QFileInfo fi(filepath);
    auto suffix = fi.completeSuffix();
    auto matrix_trans_inv = _pose_trans.inverse().toMatrix();
    auto shape_layer = _main_viewer->shapeLayer();
    shape_layer->removeAllItems();

    if (suffix == "txt") {
        QFile file(filepath);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            UWarn(tr("Cannot open file: %1").arg(filepath));
            return;
        }

        QTextStream in(&file);
        common::Polygon2d polygon;
        bool polygon_started = false;

        while (!in.atEnd()) {
            QString line = in.readLine().trimmed();

            if (line.isEmpty()) {
                if (polygon.points.size() == 4) {
                    auto pg = UTQ(polygon.points);
                    shape_layer->addPolygon(pg);
                }
                polygon.points.clear();  
                polygon_started = false;
            } else {
                QStringList coords = line.split(" ");
                if (coords.size() == 2) {
                    bool xOk, yOk;
                    double x = coords[0].toDouble(&xOk);
                    double y = coords[1].toDouble(&yOk);
                    if (xOk && yOk) {
                        polygon.addPoint(common::Point2d(x, y));
                        polygon_started = true;
                    } else {
                        UWarn(tr("Invalid coordinates in file: %1").arg(filepath));
                    }
                } else {
                    UWarn(tr("Invalid line format in file: %1").arg(filepath));
                }
            }
        }

        if (polygon_started && polygon.points.size() == 4) {
            auto pg = UTQ(polygon.points);
            shape_layer->addPolygon(pg);
        }

        file.close();
    } else {
        UWarn(tr("Error layer file format: %1").arg(suffix));
    }
    return;
}

// void ImportDetectionAction::process() {
//     auto filepath = _layer_filepaths.front();
//     QFileInfo fi(filepath);
//     auto suffix = fi.completeSuffix();
//     auto matrix_trans_inv = _pose_trans.inverse().toMatrix();
//     auto shape_layer = _main_viewer->shapeLayer();
//     shape_layer->removeAllItems();
//     if (suffix == "json") {
//         QFile file(filepath);
//         if (!file.open(QIODevice::ReadOnly)) {
//             UWarn(tr("Cannot open file: %1").arg(filepath));
//             return;
//         }

//         QByteArray jsonData = file.readAll();
//         file.close();

//         QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData);
//         if (jsonDoc.isNull()) {
//             UWarn(tr("Failed to parse JSON: %1").arg(filepath));
//             return;
//         }

//         if (!jsonDoc.isArray()) {
//             UWarn(tr("JSON root is not an array: %1").arg(filepath));
//             return;
//         }

//         QJsonArray jsonPolygons = jsonDoc.array();
//         for (const auto& jsonPolygon : jsonPolygons) {
//             if (!jsonPolygon.isArray()) continue;

//             common::Polygon2d polygon;

//             QJsonArray pointsArray = jsonPolygon.toArray();
//             if (pointsArray.size() != 4) continue; 

//             for (const auto& point : pointsArray) {
//                 QJsonArray coords = point.toArray();
//                 double x = coords[0].toDouble();
//                 double y = coords[1].toDouble();
//                 polygon.addPoint(common::Point2d(x, y));
//             }

//             auto pg = UTQ(polygon.points);
//             shape_layer->addPolygon(pg);
//         }
//     } else {
//         UWarn(tr("Error layer file format: %1").arg(suffix));
//     }
//     return;
// }
}
