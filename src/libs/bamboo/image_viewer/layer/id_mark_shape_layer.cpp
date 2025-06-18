#include "id_mark_shape_layer.h"
#include <QUuid>
#include <uface/canvas2d/command.hpp>
#include <uface/canvas2d/view.hpp>
#include <QDebug>
namespace welkin::bamboo {
/// IdMarkShapeLayer
#define NameRole Qt::UserRole + 10
IdMarkShapeLayer::IdMarkShapeLayer(QObject *parent)
    : uface::canvas2d::GenericLayer(parent) {
}
IdMarkShapeLayer::~IdMarkShapeLayer() {
    // 派生类没有图元的所有权，清空map即可（所有权在基类中）
    _id_item_map.clear();
    _item_id_map.clear();
}
bool IdMarkShapeLayer::hasShape(const QString &id) const {
    return _id_item_map.contains(id);
}

int IdMarkShapeLayer::shapeType(const QString &id) const {
    auto item = shapeItem(id);
    return item ? item->type() : -1;
}

uface::canvas2d::EntityItem *IdMarkShapeLayer::shapeItem(const QString &id) const {
    return hasShape(id) ? _id_item_map[id] : nullptr;
}

void IdMarkShapeLayer::removeShape(const QString &id) {
    if (!hasShape(id)) {return;}
    auto item = _id_item_map[id];
    this->removeItem(item);
}

void IdMarkShapeLayer::clearShapes() {
    this->removeAllItems();
}
void IdMarkShapeLayer::onAddItemSlot(
        const QString &layer_name, uface::canvas2d::EntityItem *item) {
    if (layer_name != name()) {return;}
    // 无效或已经存在
    auto titem = dynamic_cast<uface::canvas2d::EntityItem*>(item);
    if (!titem || _item_id_map.contains(titem)) {return;}
    auto id = getItemId(titem);
    _id_item_map.insert(id, titem);
    _item_id_map.insert(titem, id);
    emit shapeAdded(id);
}

void IdMarkShapeLayer::onAddItemsSlot(
        const QString &layer_name, const QList<uface::canvas2d::EntityItem *> &items) {
    if (layer_name != name()) {return;}

    QList<uface::canvas2d::EntityItem*> valid_items;
    for (auto& item : items) {
        auto titem = dynamic_cast<uface::canvas2d::EntityItem*>(item);
        if (!titem || _item_id_map.contains(titem)) {continue;}
        valid_items.append(titem);
    }
    if (valid_items.empty()) {return;}
    QList<QString> ids;
    for (auto& item : valid_items) {
        auto id = getItemId(item);
        _id_item_map.insert(id, item);
        _item_id_map.insert(item, id);
        ids.append(id);
    }
    emit shapesAdded(ids);
}

void IdMarkShapeLayer::onRemoveItemSlot(
        const QString &layer_name, uface::canvas2d::EntityItem *item) {
    if (layer_name != name()) {return;}
    // 无效或已经存在
    auto titem = dynamic_cast<uface::canvas2d::EntityItem*>(item);
    if (!titem || !_item_id_map.contains(titem)) {return;}
    auto id = _item_id_map[titem];
    _id_item_map.remove(id);
    _item_id_map.remove(titem);
    emit shapeRemoved(id);
}

void IdMarkShapeLayer::onRemoveItemsSlot(
        const QString &layer_name, const QList<uface::canvas2d::EntityItem *> &items) {
    if (layer_name != name()) {return;}
    // 无效或已经存在
    QList<QString> valid_ids;
    QList<uface::canvas2d::EntityItem*> valid_items;
    for (auto& item : items) {
        auto titem = dynamic_cast<uface::canvas2d::EntityItem*>(item);
        if (!titem || !_item_id_map.contains(titem)) {continue;}
        auto id = _item_id_map[titem];
        valid_ids.append(id);
        valid_items.append(titem);
    }
    if (valid_items.empty()) {return;}
    for (int i = 0; i < valid_ids.size(); ++i) {
        auto id = valid_ids[i];
        auto item = valid_items[i];
        _id_item_map.remove(id);
        _item_id_map.remove(item);
    }
    emit shapesRemoved(valid_ids);
}

void IdMarkShapeLayer::onClearItemsSlot(const QString &layer_name) {
    if (layer_name != name()) {return;}
    _id_item_map.clear();
    _item_id_map.clear();
    emit shapesCleared();
}

void IdMarkShapeLayer::onChangeItemSlot(uface::canvas2d::EntityItem *item) {
    if (!this->contains(item)) {return;}
    auto titem = dynamic_cast<uface::canvas2d::EntityItem*>(item);
    if (!titem || !_item_id_map.contains(titem)) {return;}
    auto id = _item_id_map[titem];
    emit shapeChanged(id);
}

void IdMarkShapeLayer::setViewer(uface::canvas2d::Viewer *viewer) {
    if (_viewer == viewer) {return;}
    if (_viewer) {
        disconnect(_viewer, SIGNAL(itemAdded(const QString&, uface::canvas2d::EntityItem*)),
            this, SLOT(onAddItemSlot(const QString&, uface::canvas2d::EntityItem*)));
        disconnect(_viewer, SIGNAL(itemsAdded(const QString&, const QList<uface::canvas2d::EntityItem*>&)),
            this, SLOT(onAddItemsSlot(const QString&, const QList<uface::canvas2d::EntityItem*>&)));
        disconnect(_viewer, SIGNAL(itemRemoved(const QString&, uface::canvas2d::EntityItem*)),
            this, SLOT(onRemoveItemSlot(const QString&, uface::canvas2d::EntityItem*)));
        disconnect(_viewer, SIGNAL(itemsRemoved(const QString&, const QList<uface::canvas2d::EntityItem*>&)),
            this, SLOT(onRemoveItemsSlot(const QString&, const QList<uface::canvas2d::EntityItem*>&)));
        disconnect(_viewer, SIGNAL(itemsCleared(const QString&)),
            this, SLOT(onClearItemsSlot(const QString&)));
        disconnect(_viewer, SIGNAL(itemChanged(uface::canvas2d::EntityItem*)),
            this, SLOT(onChangeItemSlot(uface::canvas2d::EntityItem*)));
    }
    uface::canvas2d::GenericLayer::setViewer(viewer);
    if (_viewer) {
        connect(_viewer, SIGNAL(itemAdded(const QString&, uface::canvas2d::EntityItem*)),
            this, SLOT(onAddItemSlot(const QString&, uface::canvas2d::EntityItem*)));
        connect(_viewer, SIGNAL(itemsAdded(const QString&, const QList<uface::canvas2d::EntityItem*>&)),
            this, SLOT(onAddItemsSlot(const QString&, const QList<uface::canvas2d::EntityItem*>&)));
        connect(_viewer, SIGNAL(itemRemoved(const QString&, uface::canvas2d::EntityItem*)),
            this, SLOT(onRemoveItemSlot(const QString&, uface::canvas2d::EntityItem*)));
        connect(_viewer, SIGNAL(itemsRemoved(const QString&, const QList<uface::canvas2d::EntityItem*>&)),
            this, SLOT(onRemoveItemsSlot(const QString&, const QList<uface::canvas2d::EntityItem*>&)));
        connect(_viewer, SIGNAL(itemsCleared(const QString&)),
            this, SLOT(onClearItemsSlot(const QString&)));
        connect(_viewer, SIGNAL(itemChanged(uface::canvas2d::EntityItem*)),
            this, SLOT(onChangeItemSlot(uface::canvas2d::EntityItem*)));
    }
}

QString IdMarkShapeLayer::getNewId() const {
    return QUuid::createUuid().toString();
}

QString IdMarkShapeLayer::getItemId(uface::canvas2d::EntityItem *item) const {
    // 如果设置了id，采用设置的；如果未设置，则随机生成
    auto data = item->data(int(NameRole));
    bool has_id = data.isValid()
        && data.canConvert<QString>()
        && !data.toString().isEmpty();
    return has_id ? data.toString() : getNewId();
}

// void IdMarkShapeLayer::createItemContentChangeCommand(
//         uface::canvas2d::EntityItem* item, const uface::canvas2d::EntityItemMemento &memento) {
//     if (_viewer && item) {
//         QUndoCommand* cmd = new uface::canvas2d::EntityItemContentChangeCommand(item, memento);
//         _viewer->getUndoStack()->push(cmd);
//     }
// }
}
