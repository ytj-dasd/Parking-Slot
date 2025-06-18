#pragma once
#include "bamboo/image_viewer/layer/id_mark_shape_layer.h"
#include "bamboo/image_viewer/item/graphics_text_polygon_item.h"

namespace welkin::bamboo {
class BAMBOO_EXPORT TextPolygonItemLayer : public IdMarkShapeLayer {
    Q_OBJECT
public:
    TextPolygonItemLayer(QObject* parent = nullptr);
    virtual ~TextPolygonItemLayer();

    bool hasPolygon(const QString& id) const;
    QPolygonF getPolygon(const QString& id) const;
    QMap<QString, QPolygonF> getPolygonMap() const;
    GraphicsTextPolygonItem* getPolygonItem(const QString& id) const;
    QMap<QString, GraphicsTextPolygonItem*> getPolygonItemMap() const;
    // 如果id存在，不在添加且返回空指针
    GraphicsTextPolygonItem* addPolygon(
        const QPolygonF& polygon, const QString& id = QString(),
        const QString& text = QString());
    // 如果不存在，不更新且返回false
    bool updatePolygon(
        const QString& id, const QPolygonF& polygon, const QString& text);
};
}
