#include "select_pillar_dialog.h"
#include <uface/base/uconverter.h>
#include <uface/canvas2d/item.hpp>
#include <uface/canvas2d/tool.hpp>

namespace welkin::bamboo {
SelectPillarDialog::SelectPillarDialog(QWidget* parent) : ImagePairDialog(parent) {
    this->setWindowTitle(tr("Pillar Selector"));
    QList<QString> keys;
    keys << uface::canvas2d::PolygonDrawKey << uface::canvas2d::ViewKey
         << uface::canvas2d::MoveKey << uface::canvas2d::EditKey;
    _draw_toolbar_1->setActionsVisible(keys);
    _draw_toolbar_1->setActionActived(uface::canvas2d::PolygonDrawKey, true);

    keys.clear();
    keys << uface::canvas2d::RectDrawKey << uface::canvas2d::ViewKey
         << uface::canvas2d::MoveKey << uface::canvas2d::EditKey;
    _draw_toolbar_2->setActionsVisible(keys);
    _draw_toolbar_2->setActionActived(uface::canvas2d::RectDrawKey, true);

    QPen pen(Qt::cyan, 2.0); pen.setCosmetic(true);
    _rect_layer = _viewer_2->addLayer(
        "rect", tr("Rect"), 10.0, pen, Qt::NoBrush);
    _rect_item = _rect_layer->addRect(QRectF());
    _polygon_layer = _viewer_1->addLayer(
        "polygon", tr("Polygon"), 10.0, pen, Qt::NoBrush);
    _polygon_item = _polygon_layer->addPolygon(QPolygonF());

    _rect_draw_tool = _draw_toolbar_2->getRectDrawTool();
    _polygon_draw_tool = _draw_toolbar_1->getPolygonDrawTool();
    connect(_rect_draw_tool, SIGNAL(started()), this, SLOT(hideRectItem()));
    connect(_rect_draw_tool, SIGNAL(finished()), this, SLOT(onUpdateRect()));
    connect(_polygon_draw_tool, SIGNAL(started()), this, SLOT(hidePolygonItem()));
    connect(_polygon_draw_tool, SIGNAL(finished()), this, SLOT(onUpdatePolygon()));
}

SelectPillarDialog::~SelectPillarDialog() {}

common::Polygon2d SelectPillarDialog::getPolygon() const {
    if (!_polygon_item->isVisible()) {
        return common::Polygon2d();
    }
    QPolygonF polygon = _polygon_item->mapToScene(_polygon_item->polygon());
    common::Polygon2d polygon2d;
    polygon2d.points = UFQ(polygon);
    return polygon2d;
}
common::Ranged SelectPillarDialog::getRangeZ() const {
    if (!_rect_item->isVisible()) {
        return common::Ranged();
    }
    QRectF rect = _rect_item->mapRectToScene(_rect_item->rect());
    return UFQ(rect).range_y;
}
void SelectPillarDialog::onUpdatePolygon() {
    setPolygonHelper(_polygon_draw_tool->polygon());
}
void SelectPillarDialog::onUpdateRect() {
    setRectHelper(_rect_draw_tool->rect());
}

void SelectPillarDialog::hideRectItem() {
    _rect_item->setPos(0.0, 0.0);
    _rect_item->setRotation(0.0);
    _rect_item->setRect(QRectF());
    _rect_item->hide();
}

void SelectPillarDialog::hidePolygonItem() {
    _polygon_item->setPos(0.0, 0.0);
    _polygon_item->setRotation(0.0);
    _polygon_item->setPolygon(QPolygonF());
    _polygon_item->hide();
}
void SelectPillarDialog::clearData() {
    hideRectItem();
    hidePolygonItem();
}

void SelectPillarDialog::setRectHelper(const QRectF &rect) {
    if (!_rect_item->isVisible()) {
        _rect_item->setVisible(true);
    }
    _rect_item->setPos(QPointF(0.0, 0.0));
    _rect_item->setRotation(0.0);
    _rect_item->setRect(rect);
}

void SelectPillarDialog::setPolygonHelper(const QPolygonF &polygon) {
    if (!_polygon_item->isVisible()) {
        _polygon_item->setVisible(true);
    }
    _polygon_item->setPos(QPointF(0.0, 0.0));
    _polygon_item->setRotation(0.0);
    _polygon_item->setPolygon(polygon);
}
}
