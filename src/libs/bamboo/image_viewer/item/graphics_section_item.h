#pragma once
#include <uface/canvas2d/item/entityitem.h>
#include <common/geometry/section.h>
#include "bamboo/base/macro.h"

namespace welkin::bamboo {
class BAMBOO_EXPORT GraphicsSectionItem : public uface::canvas2d::EntityItem {
public:
    GraphicsSectionItem(QGraphicsItem* parent = nullptr);
    virtual ~GraphicsSectionItem();
    enum {Type = QGraphicsItem::UserType + 102};
    int type() const override {return Type;}

    const common::Sectiond& section() const;
    common::Sectiond sceneSection() const;
    void setSection(const common::Sectiond& section);
    QString text() const;
    void setText(const QString& text);
    void setMidLinePen(const QPen& pen);
protected:
    QPainterPath contentShape() const override;
    void contentTransform(const QTransform& matrix) override;
    void contentPaint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

    bool aboutToBeEditing(const QPointF &view_pos) const override;
    bool beginEdit(const QPointF& view_pos) override;
    bool bufferEdit(const QPointF& view_pos) override;
    bool endEdit(const QPointF& view_pos) override;

    void drawControlHandles(QPainter* painter) override;
    QPainterPath controlHandlesPath() const override;

    void createPropertyMemento(uface::canvas2d::EntityItemMemento& memento) const override;
    void recoveryPropertyMemento(const uface::canvas2d::EntityItemMemento& memento) override;

    QPointF getControlHandleViewPos(int index) const;
    QRectF getControlHandleViewRect(int index) const;
    int collidesWithControlHandle(const QPointF& view_pos) const;
    void updateEditPos(const QPointF& pos);
    void updateTextPoint();
    QLineF getMidLine(const common::Sectiond& section) const;
    QLineF getMinRangeLine(const common::Sectiond& section) const;
    QLineF getMaxRangeLine(const common::Sectiond& section) const;
    QPolygonF getPolygon(const common::Sectiond& section) const;
protected:
    common::Sectiond _section;
    QPen _mid_line_pen = QPen();
    QGraphicsTextItem* _item_text = nullptr;
    int _edit_index = -1;
};
}
Q_DECLARE_METATYPE(welkin::common::Sectiond)
