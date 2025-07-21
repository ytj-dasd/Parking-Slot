#include "export_layer_as_dxf_action.h"
#include "export_layer_as_dxf_dialog.h"
#include "bamboo/file_system/terrain_folder.h"
#include <bamboo/io/dxf_io.h>
#include <common/base/dir.h>
#include <common/geometry/polygon2.h>
#include <common/geometry/polygon3.h>
#include <ucanvas/item.hpp>
#include "../main_viewer.h"

namespace welkin::bamboo {
ExportLayerAsDxfAction::ExportLayerAsDxfAction(BProjectWidget* project_widget) 
        : BProjectAction(project_widget, true) {
    this->setText(tr("Export layer as dxf file"));
    this->setToolTip(tr("Export layer as dxf file"));
    this->setIcon(UIcon("item_icon/export.png"));
    _enable_progress = true;
}
ExportLayerAsDxfAction::~ExportLayerAsDxfAction() {}

void ExportLayerAsDxfAction::setMainViewer(MainViewer* main_viewer) {
    _main_viewer = main_viewer;
}
bool ExportLayerAsDxfAction::isDataPrepared() {
    return _main_viewer->shapeLayer()->itemCount() > 0;
}
bool ExportLayerAsDxfAction::prepareParameter() {
    auto project = this->getProject();
    //
    ExportLayerAsDxfDialog dlg;
    auto local_pose_filer = project->getLocalPoseFiler();
    dlg.setLocalPose(local_pose_filer->load());
    auto global_pose_filer = project->getGlobalPoseFiler();
    dlg.setGlobalPose(global_pose_filer->load());
    auto proj_path = project->getPath();
    dlg.setPath(UTQ(proj_path));
    if (dlg.exec() != QDialog::Accepted) {
        return false;
    }
    _save_path = dlg.getSavePath();
    _pose_trans = dlg.getPose3d();
    if (_save_path.isEmpty()) {
        UWarn(tr("Save path cannot empty!"));
        return false;
    }
    return true;
}

void ExportLayerAsDxfAction::exportTerrainToDxf(DxfOutput& dxf_file) {
    auto dom_folder = this->getProject()->getDOMFolder();
    auto child_size = dom_folder->getChildSize();
    // 生成cv::Mat
    common::Rectd brect;
    double res_x = -0.1, res_y = -0.1;
    for (int i = 0; i < child_size; ++i) {
        auto sub_folder = dom_folder->getChild(i);
        auto image_rect = sub_folder->getRect();
        brect.fitRect(image_rect);
        if (i == 0) {
            auto image_mat = sub_folder->getImage();
            res_x = image_rect.getLengthX() / image_mat.cols;
            res_y = image_rect.getLengthY() / image_mat.rows;
        }
    }
    if (!brect.isValid() || res_x < 0.0 || res_y < 0.0) {
        return;
    }

    int width = int(brect.getLengthX() / res_x);
    int height = int(brect.getLengthY() / res_y);
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
    for (int i = 0; i < child_size; ++i) {
        UProgressTextValue(tr("Process..."), i, child_size);
        auto sub_folder = dom_folder->getChild(i);
        auto image_rect = sub_folder->getRect();

        auto col = int((image_rect.range_x.min_v - brect.range_x.min_v) / res_x);
        auto row = int((image_rect.range_y.min_v - brect.range_y.min_v) / res_y);
        auto image_mat = sub_folder->getImage();
        cv::Rect rect(col, row, image_mat.cols, image_mat.rows);
        if (image_mat.type() == CV_8UC3) {
            image_mat.copyTo(image(rect));
        } else if (image_mat.type() == CV_8UC1) {
            cv::cvtColor(image_mat, image(rect), cv::COLOR_GRAY2BGR);
        } else if (image_mat.type() == CV_8UC4) {
            cv::cvtColor(image_mat, image(rect), cv::COLOR_BGRA2BGR);
        } else {
            UWarn(tr("Image type is %1, not supprot convert to rgb").arg(image_mat.type()));
        }
    }
    // 生成terrain.jpg
    cv::flip(image, image, 0);
    std::string dir_path = common::Dir(UFQ(_save_path)).getFilePath();
    std::string image_filename = dir_path + "/terrain.png";
    cv::imwrite(image_filename, image);

    // 生成terrain.dxf
    common::Polyline2d corners;
    corners.points = brect.getCorners();
    common::Polyline3d corners3;
    corners3.fromPolyline2(corners);
    corners3.transform(_pose_trans.toMatrix());
    dxf_file.writePolyline3(corners3.points, true, 256, "terrain");

    // 写入图片
    auto insert_point = corners3.points.at(0);
    auto uline = common::Line3d(corners3.points.at(0), corners3.points.at(1));
    auto vline = common::Line3d(corners3.points.at(0), corners3.points.at(3));
    width = image.cols;
    height = image.rows;
    auto ures = brect.getLengthX() / double(width);
    auto vres = brect.getLengthY() / double(height);
    auto uvec = uline.getDirection() * ures;
    auto vvec = vline.getDirection() * vres;
    dxf_file.writeImage("terrain.png", width, height,
        insert_point, uvec, vvec, "terrain");

    UInfo("Make full image successfully!");
    return;
}

void ExportLayerAsDxfAction::process() {
    DxfOutput dxf_file;
    if (!dxf_file.open(UFQ(_save_path))) {
        UError(tr("Cannot open dxf file: %1").arg(_save_path));
        return;
    }
    dxf_file.writeLayerIds({"closed", "colored", "three_lines", "need_check","text", "terrain"}, 4);
    // 写出图层内容
    auto matrix_trans = _pose_trans.toMatrix();
    // 库位图层
    common::Layerd layer;
    for (const auto& group : _main_viewer->getParkSpaceGroupVec()) {
        // 无效的跳过
        if (!group.valid) {continue;}
        for (const auto& ps : group.parkspaces) {
            common::Polygon2d pg2d;
            pg2d.points = ps.points;
            common::Polygon3d pg3d;
            pg3d.fromPolygon2(pg2d);
            layer.polygons.push_back(pg3d);
        }
    }
    dxf_file.writeLayer(layer.transformed(matrix_trans), 256, "closed");
    
    // 转换至3线图层
    auto convertLineToPolyline = [](const common::Line2d& line) -> common::Polyline3d {
        common::Polyline3d pl3d;
        const auto& p1 = line.begin_point;
        const auto& p2 = line.end_point;
        pl3d.push_back(common::Point3d(p1.x, p1.y, 0.0));
        pl3d.push_back(common::Point3d(p2.x, p2.y, 0.0));
        return pl3d;
    };

    std::vector<int> levels;
    std::map<int, int> color_map = {{1, 3}, {2, 4}, {3, 2}, {4, 5}, {5, 6}, {6, 1}};
    std::vector<int> deltas(4, 0);
    std::vector<int> level_counts(6, 0);
    for (const auto& group : _main_viewer->getParkSpaceGroupVec()) {
        // 无效的跳过
        if (!group.valid) {continue;}
        for (const auto& ps : group.parkspaces) {
            ps.computeLineLevel(deltas, levels);
            for (int i = 0; i < 4; ++i) {
                ++level_counts[levels[i] - 1];
                int color = color_map[levels[i]];
                common::Polyline3d pl3d = convertLineToPolyline(ps.getLine(i));
                pl3d.transform(matrix_trans);
                common::Line2d text_line;
                text_line.begin_point = common::Point2d(pl3d.points[0].x, pl3d.points[0].y);
                text_line.end_point = common::Point2d(pl3d.points[1].x, pl3d.points[1].y);
                text_line.begin_point = text_line.getCenter();
                double angle = 0;
                dxf_file.writePolyline3(pl3d.points, false, color, "colored");
                if (deltas[i] != 0) {
                    dxf_file.writeText(std::to_string(deltas[i]), text_line, angle, 0.5, color, "text");
                }
            }
        }
    }
    for (int i = 0; i < 6; ++i) {
        std::cout << "Level " << i + 1 << " count: " << level_counts[i] << std::endl;
    }

    for (const auto& group : _main_viewer->getParkSpaceGroupVec()) {
        // 无效的跳过
        if (!group.valid) {continue;}
        for (const auto& pl : group.parklines) {
            common::Polyline3d pl3d = convertLineToPolyline(pl.center_line);
            pl3d.transform(matrix_trans);
            dxf_file.writePolyline3(pl3d.points, false, 5, "three_lines");
            pl3d = convertLineToPolyline(pl.border_line1);
            pl3d.transform(matrix_trans);
            dxf_file.writePolyline3(pl3d.points, false, 256, "three_lines");
            pl3d = convertLineToPolyline(pl.border_line2);
            pl3d.transform(matrix_trans);
            dxf_file.writePolyline3(pl3d.points, false, 256, "three_lines");
        }
    }
    exportTerrainToDxf(dxf_file);

    for (const auto& group : _main_viewer->getParkSpaceGroupVec()) {
        // 无效的跳过
        if (!group.valid) {continue;}
        for (const auto& pl : group.parklines) {
            int color_flag = 256;
            if (pl.need_check) {
                color_flag = 5;
            } else {
                color_flag = 3;
            }
            common::Polyline3d pl3d;
            if (pl.type == ParkLine::TypeBorder) {
                pl3d = convertLineToPolyline(pl.border_line);
            } else {
                pl3d = convertLineToPolyline(pl.center_line);
            }
            pl3d.transform(matrix_trans);
            dxf_file.writePolyline3(pl3d.points, false, color_flag, "need_check");
            common::Line2d text_line;
            text_line.begin_point = common::Point2d(pl3d.points[0].x, pl3d.points[0].y);
            text_line.end_point = common::Point2d(pl3d.points[1].x, pl3d.points[1].y);
            text_line.begin_point = text_line.getCenter();
            double angle = 0;
            dxf_file.writeText(std::to_string(pl.test_index), text_line, angle, 0.5, color_flag, "need_check");
        }
    }

    dxf_file.close();
    UInfo("Export layer as dxf file successfully!");
    return;
}
}
