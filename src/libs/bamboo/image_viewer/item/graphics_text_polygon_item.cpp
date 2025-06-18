#include "graphics_text_polygon_item.h"
#include <QFont>

namespace welkin::bamboo {
GraphicsTextPolygonItem::GraphicsTextPolygonItem(QGraphicsItem* parent) 
        : uface::canvas2d::PolygonItem(parent) {}
GraphicsTextPolygonItem::~GraphicsTextPolygonItem() {}

QString GraphicsTextPolygonItem::text() const {
    return _item_text ? _item_text->toPlainText() : QString();
}

void GraphicsTextPolygonItem::setText(const QString& text) {
    if (text.isEmpty() && !_item_text) {return;}
    if (!_item_text) {
        _item_text = new QGraphicsTextItem(this);
        _item_text->setDefaultTextColor(QColor(255, 128, 0));
        QFont font = _item_text->font();
        font.setPointSize(10);
        _item_text->setFont(font);
        _item_text->setFlag(ItemIgnoresParentOpacity, true);
        _item_text->setFlag(ItemIgnoresTransformations, true);
    }
    _item_text->setPlainText(text);
    this->updateTextPoint();
}

void GraphicsTextPolygonItem::updateTextPoint() {
    if (!_item_text) {return;}
    auto polygon = this->polygon();
    _item_text->setVisible(polygon.size() > 2);
    _item_text->setPos(polygon.boundingRect().center());
}
}
