#include "section_item_layer.h"
#include <uface/canvas2d/command.hpp>
#include <uface/canvas2d/view.hpp>

namespace welkin::bamboo {
#define NameRole Qt::UserRole + 10
SectionItemLayer::SectionItemLayer(QObject *parent) : IdMarkShapeLayer(parent) {
    connect(this, SIGNAL(midLinePenChanged()), this, SLOT(onChangeMidLinePen()));
    // 设置Section样式
    QPen pen(QColor(255, 0, 0), 1.0);
    pen.setCosmetic(true);
    QBrush brush(QColor(0, 255, 0, 75));
    QPen mid_line_pen(QColor(255, 255, 0), 2.0);
    mid_line_pen.setCosmetic(true);
    this->setPen(pen);
    this->setBrush(brush);
    this->setMidLinePen(mid_line_pen);
}

SectionItemLayer::~SectionItemLayer() {

}

bool SectionItemLayer::hasSection(const QString &id) const {
    return hasShape(id);
}

common::Sectiond SectionItemLayer::getSection(const QString &id) const {
    auto item = getSectionItem(id);
    return item ? item->sceneSection() : common::Sectiond();
}

QMap<QString, common::Sectiond> SectionItemLayer::getSectionMap() const {
    auto ids = _id_item_map.keys();
    QMap<QString, common::Sectiond> section_map;
    for (auto& id : ids) {
        if (!hasSection(id)) {continue;}
        section_map.insert(id, getSection(id));
    }
    return section_map;
}

GraphicsSectionItem *SectionItemLayer::getSectionItem(const QString &id) const {
    if (!hasSection(id)) {return nullptr;}
    return dynamic_cast<GraphicsSectionItem*>(_id_item_map[id]);
}

QMap<QString, GraphicsSectionItem *> SectionItemLayer::getSectionItemMap() const {
    auto ids = _id_item_map.keys();
    QMap<QString, GraphicsSectionItem*> item_map;
    for (auto& id : ids) {
        if (!hasSection(id)) {continue;}
        item_map.insert(id, getSectionItem(id));
    }
    return item_map;
}

GraphicsSectionItem* SectionItemLayer::addSection(
        const common::Sectiond& section, const QString &id, const QString& text) {
    if (hasShape(id)) {return nullptr;}
    GraphicsSectionItem* item = new GraphicsSectionItem(0);
    item->setPenByLayer(true);
    item->setBrushByLayer(true);
    item->setMidLinePen(_mid_line_pen);
    item->setSection(section);
    // 如果id不为空，直接设置
    if (!id.isEmpty()) {item->setData(int(NameRole), id);}
    // 设置text
    item->setText(text.isEmpty() ? getItemId(item) : text);
    this->addItem(item);
    return item;
}

bool SectionItemLayer::updateSection(
        const QString& id, const common::Sectiond& section, const QString& text) {
    if (!hasSection(id)) {return false;}
    auto item = dynamic_cast<GraphicsSectionItem*>(_id_item_map[id]);
    // 创建备忘
    uface::canvas2d::EntityItemMemento memento;
    item->createMemento(memento);

    item->setPos(0.0, 0.0);
    item->setRotation(0.0);
    item->setSection(section);
    item->setText(text);
    // 发送改变信号
    if (_viewer) {
        emit _viewer->itemChanged(item);
        emit _viewer->itemContentHasChanged(item, memento);    
    }
    return true;
}

void SectionItemLayer::setMidLinePen(const QPen &pen) {
    if (_mid_line_pen == pen) {return;}
    _mid_line_pen = pen;
    emit midLinePenChanged();
}

void SectionItemLayer::onChangeMidLinePen() {
    auto items = this->items();
    for (auto& item : items) {
        auto titem = dynamic_cast<GraphicsSectionItem*>(item);
        if (!titem) {continue;}
        titem->setMidLinePen(_mid_line_pen);
    }
}
}
