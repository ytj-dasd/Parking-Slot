// main_viewer.h隐性包含了opencv，flann会与opencv产生冲突；
// 故在opencv至前包含flann库，否则会报pcl/kdtree/kdtree_flann.h中的错误
#include "main_viewer.h"
#include <common/base/dir.h>
#include <QCompleter>
#include <QFile>
#include <QFileInfo>
#include <pcl/common/common.h>
#include <uface/canvas2d/item.hpp>
#include <uface/canvas2d/tool.hpp>
#include <uface/resource/uresource.h>
#include <uface/widget/uwidget.h>
#include <common/point_cloud/filter.h>
#include "parkspacegroup.h"
#include <QDebug>

namespace welkin::bamboo {
#define NameRole Qt::UserRole + 9
#define TextRole Qt::UserRole + 10
#define TypeRole Qt::UserRole + 11
MainViewer::MainViewer(QWidget* parent) : uface::canvas2d::Viewer(parent) {
    this->setYFlipped(true);
    // 创建底图图层
    createBaseMapLayer();
    // 创建绘制工具栏
    createDrawToolBar();
}

MainViewer::~MainViewer() {}

uface::canvas2d::RasterImageLayer *MainViewer::rasterImageLayer() const {
    return _raster_layer;
}
uface::canvas2d::GenericLayer* MainViewer::shapeLayer() const {
    return _shape_layer;
}

void MainViewer::clearRasterImageLayer() {
    _raster_layer->reset();
}
void MainViewer::clearShapeLayer() {
    _shape_layer->removeAllItems();
}

QToolBar *MainViewer::drawToolBar() const {
    return _draw_toolbar;
}

void MainViewer::clearViewer() {
    this->clearRasterImageLayer();
    this->clearShapeLayer();
}
void MainViewer::onOptimateSlot() {
    QVector<ParkSpace> park_space_vec;
    for (auto item : _shape_layer->items()) {
        auto eitem = dynamic_cast<uface::canvas2d::PolygonItem*>(item);
        if (!eitem) {continue;}
        auto polygon = eitem->scenePolygon();
        ParkSpace park_space;
        park_space.points = UFQ(polygon);
        auto length1 = park_space.getLine(0).getLength();
        auto length2 = park_space.getLine(1).getLength();
        auto min_length = std::min(length1, length2);
        auto max_length = std::max(length1, length2);
        if (max_length / min_length > 4.0) {continue;}
        park_space_vec.push_back(std::move(park_space));
    }
    // 计算出Group
    _park_space_group_vec.clear();
    while(!park_space_vec.empty()) {
        if (_park_space_group_vec.empty()) {
            ParkSpaceGroup park_space_group;
            park_space_group.addParkSpace(park_space_vec.front());
            park_space_vec.pop_front();
            _park_space_group_vec.push_back(std::move(park_space_group));
            continue;
        }
        QVector<int> valid_ids;
        auto& park_space_group = _park_space_group_vec.back();
        for (int i = 0; i < park_space_vec.size(); ++i) {
            const auto& park_space = park_space_vec.at(i);
            if (park_space_group.isIntersect(park_space)) {
                park_space_group.addParkSpace(park_space);
                valid_ids.append(i);
            }
        }
        if (valid_ids.empty()) {
            ParkSpaceGroup park_space_group;
            park_space_group.addParkSpace(park_space_vec.front());
            park_space_vec.pop_front();
            _park_space_group_vec.push_back(std::move(park_space_group));
            continue;
        }
        for (int i = valid_ids.size() - 1; i >= 0; --i) {
            park_space_vec.removeAt(valid_ids.at(i));
        }
    }
    // 计算长线方向和库位线
    for (auto& group : _park_space_group_vec) {
        group.computeLongSideDirection();
        group.computeParkLines();
        // 计算设计尺寸
        for (auto& ps : group.parkspaces) {
            ps.computeDesignSize();
        }
    }
    // 拟合
    auto dom_folder = _project->getDOMFolder();
    cv::Mat image;
    common::Point2d origin;
    double padding = 0.8; // 即允许转换之后存在80cm的误差
    int index = 0;
    for (auto& group : _park_space_group_vec) {
        ++index;
        common::Rectd rect = group.getBoudingRect();
        rect.addPadding(padding, padding);
        if (!dom_folder->extractImageByRect(image, rect)) {
            group.valid = false;
            continue;
        }
        origin = rect.getTopLeftPoint();
        // 拟合优化
        if (group.fit(image, origin, 0.4, 0.6) == -1) {
            group.valid = false;
            continue;
        }
        group.valid = true;
        rect = group.getBoudingRect();
        rect.addPadding(padding, padding);
        if (!dom_folder->extractImageByRect(image, rect)) {continue;}
        origin = rect.getTopLeftPoint();
        AINFO << index << "/" << _park_space_group_vec.size();
        // 边缘检测
        group.edgeDetect(image, origin, 60, 100);
        // 改正库位
        group.retifyParkSpace();
    }
    _shape_layer->removeAllItems();
    // 重新设置polygon
    for (const auto& group : _park_space_group_vec) {
        for (const auto& ps : group.parkspaces) {
            QPolygonF pg = UTQ(ps.points);
            _shape_layer->addPolygon(pg);
        }
    }

    // for (auto& group : _park_space_group_vec) {
    //     for (auto& pl : group.parklines) {
    //         // if (!pl.fit_success) {continue;}
    //         QLineF pl1 = UTQ(pl.border_line1);
    //         QLineF pl2 = UTQ(pl.border_line2);
    //         QLineF pl3 = UTQ(pl.center_line);
    //         _shape_layer->addLine(pl1);
    //         _shape_layer->addLine(pl2);
    //         _shape_layer->addLine(pl3);
    //     }
    // }
}
void MainViewer::onAddPolygonSlot() {
    auto polygon_draw_tool = _draw_toolbar->getPolygonDrawTool();
    _shape_layer->addPolygon(polygon_draw_tool->polygon());
}

void MainViewer::onAddRectSlot() {
    auto rect_draw_tool = _draw_toolbar->getRectDrawTool();
    QPolygonF polygon(rect_draw_tool->rect());
    polygon.pop_back(); // 最后一个点与第一个点重合，删除
    _shape_layer->addPolygon(polygon);
}
void MainViewer::createBaseMapLayer() {
    // 底图图层
    _raster_layer = new uface::canvas2d::RasterImageLayer(this);
    _raster_layer->setName("raster");
    _raster_layer->setText(tr("Raster"));
    _raster_layer->setZValue(-1.0);
    this->addLayer(_raster_layer);

    // 设置画笔
    QPen pen;
    pen.setColor(Qt::green);
    pen.setWidthF(2.0);
    pen.setCosmetic(true);
    QBrush brush(Qt::NoBrush);
    _shape_layer = new uface::canvas2d::GenericLayer(this);
    _shape_layer->setName("shape");
    _shape_layer->setText(tr("Shape"));
    _shape_layer->setPen(pen);
    _shape_layer->setBrush(brush);
    this->addLayer(_shape_layer);
}
void MainViewer::createDrawToolBar() {
    // 增加绘制工具栏
    _draw_toolbar = new uface::canvas2d::DrawToolBar(this);
    _draw_toolbar->setViewer(this);
    // // 将线绘制、成组和解组按钮隐藏
    // _draw_toolbar->setActionVisible(uface::canvas2d::LineDrawKey, false);
    // _draw_toolbar->setActionVisible(uface::canvas2d::GroupKey, false);
    // _draw_toolbar->setActionVisible(uface::canvas2d::UnGroupKey, false);
    // _draw_toolbar->setActionActived(uface::canvas2d::SelectKey, true);
    QStringList keys;
    keys << uface::canvas2d::RectDrawKey << uface::canvas2d::PolygonDrawKey
        << uface::canvas2d::SelectKey << uface::canvas2d::MoveKey
        << uface::canvas2d::RotateKey << uface::canvas2d::ScaleKey
        << uface::canvas2d::EditKey << uface::canvas2d::DeleteKey;
    _draw_toolbar->setActionsVisible(keys);
    _draw_toolbar->setActionActived(uface::canvas2d::SelectKey, true);

    // 建立绘制工具的信号-槽
    auto polygon_draw_tool = _draw_toolbar->getPolygonDrawTool();
    connect(polygon_draw_tool, SIGNAL(finished()), this, SLOT(onAddPolygonSlot()));
    auto rect_draw_tool = _draw_toolbar->getRectDrawTool();
    connect(rect_draw_tool, SIGNAL(finished()), this, SLOT(onAddRectSlot()));
}
}
