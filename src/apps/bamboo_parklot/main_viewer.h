#pragma once
#include <QLabel>
#include <ucanvas/action.hpp>
#include <ucanvas/layer.hpp>
#include <ucanvas/view.hpp>
#include "bamboo/file_system/origin_data_folder.h"
#include "bamboo/file_system/project.h"
#include "parkspacegroup.h"
#include "file_system.h"

namespace welkin::bamboo {

class ParklotFileSystem;
class MainViewer : public ucanvas::Viewer {
    Q_OBJECT
public:
    explicit MainViewer(QWidget* parent = nullptr);
    virtual ~MainViewer();

    
    ucanvas::RasterImageLayer* rasterImageLayer() const;
    ucanvas::GenericLayer* shapeLayer() const;
    void clearRasterImageLayer();
    void clearShapeLayer();
    QToolBar* drawToolBar() const;

    // 测试使用
    bool init(Project* project) {
        _project = project;
        return true;
    }
    Project* project() const {return _project;}
    const QVector<ParkSpaceGroup>& getParkSpaceGroupVec() const {
        return _park_space_group_vec;
    }

    struct CornerInfo {
        Eigen::Vector2f next_point1;    // 相邻的第一个点
        Eigen::Vector2f next_point2;    // 相邻的第二个点
        Eigen::Vector2f direction1;     // 第一条边的方向
        Eigen::Vector2f direction2;     // 第二条边的方向
        int parkspace_id;               // 角点所属的停车位ID
        int corner_id;                  // 角点在停车位中的ID
    };

    struct Vector2fCompare {
        bool operator()(const Eigen::Vector2f& a, const Eigen::Vector2f& b) const {
            if (a.x() < b.x()) {
                return true;
            } else if (a.x() == b.x()) {
                return a.y() < b.y();
            } else {
                return false;
            }
        }
    };

public slots:
    void clearViewer();
    void onOptimateSlot();
    void onOptimateSlot2();
    void onOptimateSlot3();
    void onOptimateSlot4();
    void onOptimateSlot4_2();
private slots:
    void recordCorner(const Eigen::Vector2f& corner, const ParkSpace& space, int parkspace_id, int corner_id, 
        std::map<Eigen::Vector2f, CornerInfo, Vector2fCompare>& corner_info_);
    common::Polygon2d handleLCorner(const Eigen::Vector2f& corner, const std::map<Eigen::Vector2f, CornerInfo, Vector2fCompare>& corner_info_, 
        Eigen::Vector2f& new_corner, Eigen::Vector2f& trans);
    common::Polygon2d handleTCorner(const Eigen::Vector2f& corner, const std::map<Eigen::Vector2f, CornerInfo, Vector2fCompare>& corner_info_, 
        const std::map<Eigen::Vector2f, int, Vector2fCompare>& corner_count_map, Eigen::Vector2f& new_corner, Eigen::Vector2f& trans);
    common::Polygon2d handleMultiCorner(const Eigen::Vector2f& corner, const std::map<Eigen::Vector2f, CornerInfo, Vector2fCompare>& corner_info_, Eigen::Vector2f& new_corner);
    void onAddPolygonSlot();
    void onAddRectSlot();

protected:
    void createBaseMapLayer();
    void createDrawToolBar();
protected:
    ParklotFileSystem* _file_system = nullptr;
    ucanvas::DrawToolBar* _draw_toolbar = nullptr;
    // 底图图层： 只读图层
    ucanvas::RasterImageLayer* _raster_layer = nullptr;
    ucanvas::GenericLayer* _shape_layer = nullptr;
    ucanvas::GenericLayer* _opt_layer = nullptr;
    Project* _project = nullptr;
    QVector<ParkSpaceGroup> _park_space_group_vec;
};
}
