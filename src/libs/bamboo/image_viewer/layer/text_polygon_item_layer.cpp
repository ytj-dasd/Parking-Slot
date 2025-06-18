#include "text_polygon_item_layer.h"
#include <uface/canvas2d/command.hpp>
#include <uface/canvas2d/view.hpp>

namespace welkin::bamboo {
#define NameRole Qt::UserRole + 10
TextPolygonItemLayer::TextPolygonItemLayer(QObject *parent)
    : IdMarkShapeLayer(parent) {

}

TextPolygonItemLayer::~TextPolygonItemLayer() {

}

bool TextPolygonItemLayer::hasPolygon(const QString &id) const {
    return hasShape(id);
}

QPolygonF TextPolygonItemLayer::getPolygon(const QString &id) const {
    auto item = getPolygonItem(id);
    return item ? item->mapToScene(item->polygon()) : QPolygonF();
}

QMap<QString, QPolygonF> TextPolygonItemLayer::getPolygonMap() const {
    auto ids = _id_item_map.keys();
    QMap<QString, QPolygonF> polygon_map;
    for (auto& id : ids) {
        if (!hasPolygon(id)) {continue;}
        polygon_map.insert(id, getPolygon(id));
    }
    return polygon_map;
}

GraphicsTextPolygonItem *TextPolygonItemLayer::getPolygonItem(const QString &id) const {
    if (!hasPolygon(id)) {return nullptr;}
    return dynamic_cast<GraphicsTextPolygonItem*>(_id_item_map[id]);
}

QMap<QString, GraphicsTextPolygonItem *> TextPolygonItemLayer::getPolygonItemMap() const {
    auto ids = _id_item_map.keys();
    QMap<QString, GraphicsTextPolygonItem*> item_map;
    for (auto& id : ids) {
        if (!hasPolygon(id)) {continue;}
        item_map.insert(id, getPolygonItem(id));
    }
    return item_map;
}

GraphicsTextPolygonItem* TextPolygonItemLayer::addPolygon(
        const QPolygonF &polygon, const QString &id, const QString& text) {
    if (hasShape(id)) {return nullptr;}
    GraphicsTextPolygonItem* item = new GraphicsTextPolygonItem(0);
    item->setPenByLayer(true);
    item->setBrushByLayer(true);
    item->setPolygon(polygon);
    // 如果id不为空，直接设置
    if (!id.isEmpty()) {item->setData(int(NameRole), id);}
    // 设置text
    item->setText(text.isEmpty() ? getItemId(item) : text);
    this->addItem(item);
    return item;
}

bool TextPolygonItemLayer::updatePolygon(
        const QString& id, const QPolygonF& polygon, const QString& text) {
    if (!hasPolygon(id)) {return false;}
    auto item = dynamic_cast<GraphicsTextPolygonItem*>(_id_item_map[id]);
    // 创建备忘
    uface::canvas2d::EntityItemMemento memento;
    item->createMemento(memento);

    item->setPos(0.0, 0.0);
    item->setRotation(0.0);
    item->setPolygon(polygon);
    item->setText(text);
    // 发送改变信号
    if (_viewer) {
        emit _viewer->itemChanged(item);
        emit _viewer->itemContentHasChanged(item, memento);
    }
    return true;
}
}
