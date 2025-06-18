#pragma once
#include <QLabel>
#include <uface/canvas2d/action.hpp>
#include <uface/canvas2d/layer.hpp>
#include <uface/canvas2d/view.hpp>
#include "bamboo/file_system/origin_data_folder.h"
#include "bamboo/file_system/project.h"
#include "parkspacegroup.h"

namespace welkin::bamboo {
class MainViewer : public uface::canvas2d::Viewer {
    Q_OBJECT
public:
    explicit MainViewer(QWidget* parent = nullptr);
    virtual ~MainViewer();

    uface::canvas2d::RasterImageLayer* rasterImageLayer() const;
    uface::canvas2d::GenericLayer* shapeLayer() const;
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
public slots:
    void clearViewer();
    void onOptimateSlot();
private slots:
    void onAddPolygonSlot();
    void onAddRectSlot();

protected:
    void createBaseMapLayer();
    void createDrawToolBar();
protected:
    uface::canvas2d::DrawToolBar* _draw_toolbar = nullptr;
    // 底图图层： 只读图层
    uface::canvas2d::RasterImageLayer* _raster_layer = nullptr;
    uface::canvas2d::GenericLayer* _shape_layer = nullptr;
    Project* _project = nullptr;
    QVector<ParkSpaceGroup> _park_space_group_vec;
};
}
