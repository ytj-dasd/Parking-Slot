#pragma once
#include <uface/canvas2d/item/polygonitem.h>
#include "bamboo/base/macro.h"

namespace welkin::bamboo {
class BAMBOO_EXPORT GraphicsTextPolygonItem : public uface::canvas2d::PolygonItem {
public:
    GraphicsTextPolygonItem(QGraphicsItem* parent = nullptr);
    virtual ~GraphicsTextPolygonItem();
    
    QString text() const;
    void setText(const QString& text);
protected:
    void updateTextPoint();
protected:
    QGraphicsTextItem* _item_text = nullptr;
};
}
