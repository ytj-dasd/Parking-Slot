#pragma once
#include <QMap>
#include <uface/canvas2d/layer/genericlayer.h>
#include "uface/canvas2d/item.hpp"
#include "bamboo/base/macro.h"

namespace welkin::bamboo {
// 使用Id标记的图形图层
class BAMBOO_EXPORT IdMarkShapeLayer : public uface::canvas2d::GenericLayer {
    Q_OBJECT
public:
    IdMarkShapeLayer(QObject* parent = nullptr);
    virtual ~IdMarkShapeLayer();

    bool hasShape(const QString& id) const;
    int shapeType(const QString& id) const;
    uface::canvas2d::EntityItem* shapeItem(const QString& id) const;
    void removeShape(const QString& id);
    void clearShapes();

Q_SIGNALS:
    // 增加图形
    void shapeAdded(const QString& id);
    void shapesAdded(const QList<QString>& ids);
    // 改变图形
    void shapeChanged(const QString& id);
    // 删除图形
    void shapeRemoved(const QString& id);
    void shapesRemoved(const QList<QString>& ids);
    // 清除
    void shapesCleared();

protected Q_SLOTS:
    void onAddItemSlot(const QString& layer_name, uface::canvas2d::EntityItem* item);
    void onAddItemsSlot(const QString& layer_name, const QList<uface::canvas2d::EntityItem*>& items);
    void onRemoveItemSlot(const QString& layer_name, uface::canvas2d::EntityItem* item);
    void onRemoveItemsSlot(const QString& layer_name, const QList<uface::canvas2d::EntityItem*>& items);
    void onClearItemsSlot(const QString& layer_name);
    void onChangeItemSlot(uface::canvas2d::EntityItem* item);

protected:
    void setViewer(uface::canvas2d::Viewer* viewer) override;
    QString getNewId() const;
    QString getItemId(uface::canvas2d::EntityItem* item) const;
    // void createItemContentChangeCommand(uface::canvas2d::EntityItem* item,
    //     const uface::canvas2d::EntityItemMemento& memento);
protected:
    QMap<QString, uface::canvas2d::EntityItem*> _id_item_map;
    QMap<uface::canvas2d::EntityItem*, QString> _item_id_map;
};
}
