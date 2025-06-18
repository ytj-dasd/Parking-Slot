#include "select_polygon_dialog.h"
#include <uface/base/uconverter.h>
#include <uface/canvas2d/item.hpp>
#include <uface/canvas2d/tool.hpp>

namespace welkin::bamboo {
SelectPolygonDialog::SelectPolygonDialog(QWidget* parent) : RectImageDialog(parent) {
    this->setWindowTitle(tr("Polygon Selector"));

    QList<QString> keys;
    keys << uface::canvas2d::RectDrawKey << uface::canvas2d::PolygonDrawKey
        << uface::canvas2d::ViewKey << uface::canvas2d::MoveKey 
        << uface::canvas2d::EditKey;
    _draw_toolbar->setActionsVisible(keys);
    _draw_toolbar->setActionActived(uface::canvas2d::RectDrawKey, true);

    QPen pen(Qt::cyan, 2.0); pen.setCosmetic(true);
    _polygon_layer = _image_viewer->addLayer(
        "polygon", tr("Polygon"), 10.0, pen, Qt::NoBrush);
    _polygon_item = _polygon_layer->addPolygon(QPolygonF());

    _rect_draw_tool = _draw_toolbar->getRectDrawTool();
    connect(_rect_draw_tool, SIGNAL(started()), this, SLOT(hidePolygonItem()));
    connect(_rect_draw_tool, SIGNAL(finished()), this, SLOT(onUpdateRect()));
    _polygon_draw_tool = _draw_toolbar->getPolygonDrawTool();
    connect(_polygon_draw_tool, SIGNAL(started()), this, SLOT(hidePolygonItem()));
    connect(_polygon_draw_tool, SIGNAL(finished()), this, SLOT(onUpdatePolygon()));
}

SelectPolygonDialog::~SelectPolygonDialog() {

}
common::Polygon2d SelectPolygonDialog::getPolygon() const {
    if (!_polygon_item->isVisible()) {
        return common::Polygon2d();
    }
    QPolygonF polygon = _polygon_item->mapToScene(_polygon_item->polygon());
    common::Polygon2d polygon2d;
    polygon2d.points = UFQ(polygon);
    return polygon2d;
}

void SelectPolygonDialog::onUpdateRect() {
    QPolygonF polygon(_rect_draw_tool->rect());
    polygon.pop_back();
    setPolygonHelper(polygon);
}

void SelectPolygonDialog::onUpdatePolygon() {
    setPolygonHelper(_polygon_draw_tool->polygon());
}

void SelectPolygonDialog::hidePolygonItem() {
    _polygon_item->setPos(0.0, 0.0);
    _polygon_item->setRotation(0.0);
    _polygon_item->setPolygon(QPolygonF());
    _polygon_item->hide();
}

void SelectPolygonDialog::clearData() {
    hidePolygonItem();
}

void SelectPolygonDialog::setPolygonHelper(const QPolygonF &polygon) {
    if (!_polygon_item->isVisible()) {
        _polygon_item->setVisible(true);
    }
    _polygon_item->setPos(QPointF(0.0, 0.0));
    _polygon_item->setRotation(0.0);
    _polygon_item->setPolygon(polygon);
}
}
