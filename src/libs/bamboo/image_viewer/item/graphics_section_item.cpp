#include "graphics_section_item.h"
#include <QPainter>
#include <uface/base/uconverter.h>
#include <QDebug>
namespace welkin::bamboo {
GraphicsSectionItem::GraphicsSectionItem(QGraphicsItem *parent)
        : uface::canvas2d::EntityItem(parent) {}

GraphicsSectionItem::~GraphicsSectionItem() {

}

const common::Sectiond &GraphicsSectionItem::section() const {
    return _section;
}

common::Sectiond GraphicsSectionItem::sceneSection() const {
    common::Sectiond section = this->section();
    QLineF line = getMidLine(section);
    line.setP1(this->mapToScene(line.p1()));
    line.setP2(this->mapToScene(line.p2()));
    section.line = UFQ(line);
    return section;
}

void GraphicsSectionItem::setSection(const common::Sectiond &section) {
    if (_section.isEqualTo(section)) {return;}
    prepareGeometryChange();
    _section = section;
    update();
    resetTransformOriginPoint();
    // 更新文本位置
    updateTextPoint();
}

QString GraphicsSectionItem::text() const {
    return _item_text ? _item_text->toPlainText() : QString();
}

void GraphicsSectionItem::setText(const QString& text) {
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

void GraphicsSectionItem::setMidLinePen(const QPen &pen) {
    if (_mid_line_pen == pen) {return;}
    prepareGeometryChange();
    _mid_line_pen = pen;
    update();
}

QPainterPath GraphicsSectionItem::contentShape() const {
    QPainterPath path;
    path.addPolygon(getPolygon(_section));
    return path;
}

void GraphicsSectionItem::contentTransform(const QTransform &matrix) {
    prepareGeometryChange();
    QLineF line = getMidLine(_section);
    line = matrix.map(line);
    _section.line = UFQ(line);
    update();
}

void GraphicsSectionItem::contentPaint(
        QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    Q_UNUSED(option)
    Q_UNUSED(widget)
    {
        painter->save();
        painter->setPen(pen());
        painter->setBrush(brush());
        painter->drawPolygon(getPolygon(_section));
        painter->restore();
    }
    {
        painter->save();
        painter->setPen(_mid_line_pen);
        painter->drawLine(getMidLine(_section));
        painter->restore();
    }
//    {
//        painter->save();
//        QPen pen = this->pen();
//        pen.setWidthF(1.5 * pen.widthF());
//        painter->setPen(pen);
//        painter->setBrush(brush());
//        painter->drawLine(getMinRangeLine(_section));
//        painter->restore();
    //    }
}

bool GraphicsSectionItem::aboutToBeEditing(const QPointF &view_pos) const {
    return collidesWithControlHandle(view_pos) != -1;
}

bool GraphicsSectionItem::beginEdit(const QPointF &view_pos) {
    _edit_index = collidesWithControlHandle(view_pos);
    return _edit_index != -1;
}

bool GraphicsSectionItem::bufferEdit(const QPointF &view_pos) {
    updateEditPos(mapFromView(view_pos));
    this->updateTextPoint();
    return true;
}

bool GraphicsSectionItem::endEdit(const QPointF &view_pos) {
    updateEditPos(mapFromView(view_pos));
    this->resetTransformOriginPoint();
    this->updateTextPoint();
    _edit_index = -1;
    uface::canvas2d::EntityItem::endEdit(view_pos);
    return true;
}

void GraphicsSectionItem::drawControlHandles(QPainter *painter) {
    painter->save();
    painter->setPen(_control_handle_pen);
    painter->setBrush(_control_handle_brush);
    painter->resetTransform();
    for (int i = 0; i < 4; ++i) {
        painter->drawRect(getControlHandleViewRect(i));
    }
    painter->restore();
}

QPainterPath GraphicsSectionItem::controlHandlesPath() const {
    QPainterPath path;
    for (int i = 0; i < 4; ++i) {
        path.addPolygon(mapFromView(getControlHandleViewRect(i)));
    }
    return path;
}

void GraphicsSectionItem::createPropertyMemento(uface::canvas2d::EntityItemMemento &memento) const {
    memento.shape().setValue(this->section());
}

void GraphicsSectionItem::recoveryPropertyMemento(const uface::canvas2d::EntityItemMemento &memento) {
    if (memento.shape().canConvert<common::Sectiond>()) {
        this->setSection(memento.shape().value<common::Sectiond>());
    }
}

QPointF GraphicsSectionItem::getControlHandleViewPos(int index) const {
    if (index < 0 || index >= 4) {return QPointF();}
    QPointF pos;
    if (index == 0) {
        pos = getMidLine(_section).p1();
    } else if (index == 1) {
        pos = getMidLine(_section).p2();
    } else if (index == 2) {
        pos = getMinRangeLine(_section).center();
    } else if (index == 3) {
        pos = getMaxRangeLine(_section).center();
    }
    return mapToView(pos);
}

QRectF GraphicsSectionItem::getControlHandleViewRect(int index) const {
    return _control_handle_rect.translated(getControlHandleViewPos(index));
}

int GraphicsSectionItem::collidesWithControlHandle(const QPointF &view_pos) const {
    if (!controlHandleIsVisible()) {return -1;}
    for (int i = 0; i < 4; ++i) {
        auto rect = getControlHandleViewRect(i);
        if (rect.contains(view_pos)) {return i;}
    }
    return -1;
}

void GraphicsSectionItem::updateEditPos(const QPointF &pos) {
    if (_edit_index == -1) {return;}
    prepareGeometryChange();
    if (_edit_index == 0) {
        _section.line.begin_point = UFQ(pos);
    } else if (_edit_index == 1) {
        _section.line.end_point = UFQ(pos);
    } else if (_edit_index == 2) {
        auto dist = _section.line.distanceTo(pos.x(), pos.y());
        _section.range.min_v = -dist;
    } else if (_edit_index == 3) {
        auto dist = _section.line.distanceTo(pos.x(), pos.y());
        _section.range.max_v = dist;
    }
    update();
}

void GraphicsSectionItem::updateTextPoint() {
    if (!_item_text) {return;}
    auto line = this->getMidLine(_section);
    _item_text->setVisible(!line.isNull());
    _item_text->setPos(line.center());
}

QLineF GraphicsSectionItem::getMidLine(const common::Sectiond &section) const {
    return UTQ(section.line);
}

QLineF GraphicsSectionItem::getMinRangeLine(const common::Sectiond &section) const {
    return UTQ(section.getMinRangeLine());
}

QLineF GraphicsSectionItem::getMaxRangeLine(const common::Sectiond &section) const {
    return UTQ(section.getMaxRangeLine());
}

QPolygonF GraphicsSectionItem::getPolygon(const common::Sectiond &section) const {
    return UTQ(section.getCorners());
}
}
