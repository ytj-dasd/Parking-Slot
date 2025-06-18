#include "draw_shape_layer.h"
#include <uface/canvas2d/command.hpp>
#include <uface/canvas2d/view.hpp>

namespace welkin::bamboo {
/// DrawShapeLayer
#define NameRole Qt::UserRole + 10
const int PointTypeKey = uface::canvas2d::PointItem::Type;
const int PolylineTypeKey = uface::canvas2d::PolylineItem::Type;
const int PolygonTypeKey = uface::canvas2d::PolygonItem::Type;
DrawShapeLayer::DrawShapeLayer(QObject *parent) : IdMarkShapeLayer(parent) {}
DrawShapeLayer::~DrawShapeLayer() {}

bool DrawShapeLayer::hasPoint(const QString &id) const {
    return hasShape(id) && shapeType(id) == PointTypeKey;
}
QPointF DrawShapeLayer::getPoint(const QString &id) const {
    auto item = getPointItem(id);
    return item ? item->mapToScene(item->point()) : QPointF();
}
QMap<QString, QPointF> DrawShapeLayer::getPointMap() const {
    QMap<QString, QPointF> point_map;
    auto ids = _id_item_map.keys();
    for (auto& id : ids) {
        if (!hasPoint(id)) {continue;}
        point_map.insert(id, getPoint(id));
    }
    return point_map;
}

uface::canvas2d::PointItem *DrawShapeLayer::getPointItem(const QString& id) const {
    if (!hasPoint(id)) {return nullptr;}
    return dynamic_cast<uface::canvas2d::PointItem*>(_id_item_map[id]);
}

QMap<QString, uface::canvas2d::PointItem *> DrawShapeLayer::getPointItemMap() const {
    QMap<QString, uface::canvas2d::PointItem*> item_map;
    auto ids = _id_item_map.keys();
    for (auto& id : ids) {
        if (!hasPoint(id)) {continue;}
        item_map.insert(id, getPointItem(id));
    }
    return item_map;
}

uface::canvas2d::PointItem* DrawShapeLayer::addPoint(
        const QPointF &point, const QString &id) {
    if (hasShape(id)) {return nullptr;}
    auto item = new uface::canvas2d::PointItem(point, 0);
    item->setPenByLayer(true);
    item->setBrushByLayer(true);
    // 如果id不为空，直接设置
    if (!id.isEmpty()) {
        item->setData(int(NameRole), id);
    }
    // 先设置id，再将item加入图层
    this->addItem(item);
    return item;
}

bool DrawShapeLayer::updatePoint(const QString &id, const QPointF &point) {
    if (!hasPoint(id)) {return false;}
    auto item = dynamic_cast<uface::canvas2d::PointItem*>(_id_item_map[id]);
    // 创建备忘
    uface::canvas2d::EntityItemMemento memento;
    item->createMemento(memento);
    item->setPos(0.0, 0.0);
    item->setRotation(0.0);
    item->setPoint(point);
    // 发送改变信号
    if (_viewer) {
        emit _viewer->itemChanged(item);
        emit _viewer->itemContentHasChanged(item, memento);
    }
    return true;
}

bool DrawShapeLayer::hasPolyline(const QString &id) const {
    return hasShape(id) && shapeType(id) == PolylineTypeKey;
}

QPolygonF DrawShapeLayer::getPolyline(const QString &id) const {
    auto item = getPolylineItem(id);
    return item ? item->mapToScene(item->polyline()) : QPolygonF();
}

QMap<QString, QPolygonF> DrawShapeLayer::getPolylineMap() const {
    auto ids = _id_item_map.keys();
    QMap<QString, QPolygonF> polyline_map;
    for (auto& id : ids) {
        if (!hasPolyline(id)) {continue;}
        polyline_map.insert(id, getPolyline(id));
    }
    return polyline_map;
}

uface::canvas2d::PolylineItem *DrawShapeLayer::getPolylineItem(const QString &id) const {
    if (!hasPolyline(id)) {return nullptr;}
    return dynamic_cast<uface::canvas2d::PolylineItem*>(_id_item_map[id]);
}

QMap<QString, uface::canvas2d::PolylineItem *> DrawShapeLayer::getPolylineItemMap() const {
    auto ids = _id_item_map.keys();
    QMap<QString, uface::canvas2d::PolylineItem*> item_map;
    for (auto& id : ids) {
        if (!hasPolyline(id)) {continue;}
        item_map.insert(id, getPolylineItem(id));
    }
    return item_map;
}

uface::canvas2d::PolylineItem* DrawShapeLayer::addPolyline(
        const QPolygonF &polyline, const QString &id) {
    if (hasShape(id)) {return nullptr;}
    auto item = new uface::canvas2d::PolylineItem(polyline, 0);
    item->setPenByLayer(true);
    item->setBrushByLayer(true);
    if (!id.isEmpty()) { // 如果id不为空，直接设置
        item->setData(int(NameRole), id);
    }
    this->addItem(item);
    return item;
}

bool DrawShapeLayer::updatePolyline(
        const QString &id, const QPolygonF &polyline) {
    if (!hasPolyline(id)) {return false;}
    auto item = dynamic_cast<uface::canvas2d::PolylineItem*>(_id_item_map[id]);
    // 创建备忘
    uface::canvas2d::EntityItemMemento memento;
    item->createMemento(memento);
    item->setPos(0.0, 0.0);
    item->setRotation(0.0);
    item->setPolyline(polyline);
    // 发送改变信号
    if (_viewer) {
        emit _viewer->itemChanged(item);
        emit _viewer->itemContentHasChanged(item, memento);
    }
    return true;
}

bool DrawShapeLayer::hasPolygon(const QString &id) const {
    return hasShape(id) && shapeType(id) == PolygonTypeKey;
}

QPolygonF DrawShapeLayer::getPolygon(const QString &id) const {
    auto item = getPolygonItem(id);
    return item ? item->mapToScene(item->polygon()) : QPolygonF();
}

QMap<QString, QPolygonF> DrawShapeLayer::getPolygonMap() const {
    auto ids = _id_item_map.keys();
    QMap<QString, QPolygonF> polygon_map;
    for (auto& id : ids) {
        if (!hasPolygon(id)) {continue;}
        polygon_map.insert(id, getPolygon(id));
    }
    return polygon_map;
}

uface::canvas2d::PolygonItem *DrawShapeLayer::getPolygonItem(const QString &id) const {
    if (!hasPolygon(id)) {return nullptr;}
    return dynamic_cast<uface::canvas2d::PolygonItem*>(_id_item_map[id]);
}

QMap<QString, uface::canvas2d::PolygonItem *> DrawShapeLayer::getPolygonItemMap() const {
    auto ids = _id_item_map.keys();
    QMap<QString, uface::canvas2d::PolygonItem*> item_map;
    for (auto& id : ids) {
        if (!hasPolygon(id)) {continue;}
        item_map.insert(id, getPolygonItem(id));
    }
    return item_map;
}

uface::canvas2d::PolygonItem* DrawShapeLayer::addPolygon(
        const QPolygonF &polygon, const QString &id) {
    if (hasShape(id)) {return nullptr;}
    auto item = new uface::canvas2d::PolygonItem(polygon, 0);
    item->setPenByLayer(true);
    item->setBrushByLayer(true);
    if (!id.isEmpty()) { // 如果id不为空，直接设置
        item->setData(int(NameRole), id);
    }
    this->addItem(item);
    return item;
}

bool DrawShapeLayer::updatePolygon(
        const QString &id, const QPolygonF &polygon) {
    if (!hasPolygon(id)) {return false;}
    auto item = dynamic_cast<uface::canvas2d::PolygonItem*>(_id_item_map[id]);
    // 创建备忘
    uface::canvas2d::EntityItemMemento memento;
    item->createMemento(memento);

    item->setPos(0.0, 0.0);
    item->setRotation(0.0);
    item->setPolygon(polygon);
    // 发送改变信号
    if (_viewer) {
        emit _viewer->itemChanged(item);
        emit _viewer->itemContentHasChanged(item, memento);
    }
    return true;
}
}
