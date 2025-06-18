#pragma once
#include "bamboo/image_viewer/layer/id_mark_shape_layer.h"
#include "bamboo/image_viewer/item/graphics_section_item.h"

namespace welkin::bamboo {
class BAMBOO_EXPORT SectionItemLayer : public IdMarkShapeLayer {
    Q_OBJECT
public:
    SectionItemLayer(QObject* parent = nullptr);
    virtual ~SectionItemLayer();

    bool hasSection(const QString& id) const;
    common::Sectiond getSection(const QString& id) const;
    QMap<QString, common::Sectiond> getSectionMap() const;
    GraphicsSectionItem* getSectionItem(const QString& id) const;
    QMap<QString, GraphicsSectionItem*> getSectionItemMap() const;
    // 如果id存在，不在添加且返回空指针
    GraphicsSectionItem* addSection(
        const common::Sectiond& section, const QString& id = QString(),
        const QString& text = QString());
    // 如果不存在，不更新且返回false
    bool updateSection(
        const QString& id, const common::Sectiond& section, const QString& text);

    void setMidLinePen(const QPen& pen);
Q_SIGNALS:
    void midLinePenChanged();
protected Q_SLOTS:
    void onChangeMidLinePen();
protected:
    QPen _mid_line_pen;
};
}
