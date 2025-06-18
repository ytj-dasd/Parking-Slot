#pragma once
#include "bamboo/image_viewer/layer/id_mark_shape_layer.h"
#include <common/geometry/line2.h>
#include <common/geometry/polyline2.h>
#include <common/geometry/polygon2.h>

namespace welkin::bamboo {
class BAMBOO_EXPORT DrawShapeLayer : public IdMarkShapeLayer {
    Q_OBJECT
public:
    DrawShapeLayer(QObject* parent = nullptr);
    virtual ~DrawShapeLayer();

    bool hasPoint(const QString& id) const;
    QPointF getPoint(const QString& id) const;
    QMap<QString, QPointF> getPointMap() const;
    uface::canvas2d::PointItem* getPointItem(const QString& id) const;
    QMap<QString, uface::canvas2d::PointItem*> getPointItemMap() const;
    // 如果id存在，不在添加且返回空指针
    uface::canvas2d::PointItem* addPoint(const QPointF& point,
        const QString& id = QString());
    // 如果不存在，不更新且返回false
    bool updatePoint(const QString& id, const QPointF& point);

    bool hasPolyline(const QString& id) const;
    QPolygonF getPolyline(const QString& id) const;
    QMap<QString, QPolygonF> getPolylineMap() const;
    uface::canvas2d::PolylineItem* getPolylineItem(const QString& id) const;
    QMap<QString, uface::canvas2d::PolylineItem*> getPolylineItemMap() const;
    // 如果id存在，不在添加且返回空指针
    uface::canvas2d::PolylineItem* addPolyline(const QPolygonF& polyline,
        const QString& id = QString());
    // 如果不存在，不更新且返回false
    bool updatePolyline(const QString& id, const QPolygonF& polyline);

    bool hasPolygon(const QString& id) const;
    QPolygonF getPolygon(const QString& id) const;
    QMap<QString, QPolygonF> getPolygonMap() const;
    uface::canvas2d::PolygonItem* getPolygonItem(const QString& id) const;
    QMap<QString, uface::canvas2d::PolygonItem*> getPolygonItemMap() const;
    // 如果id存在，不在添加且返回空指针
    uface::canvas2d::PolygonItem* addPolygon(const QPolygonF& polygon,
        const QString& id = QString());
    // 如果不存在，不更新且返回false
    bool updatePolygon(const QString& id, const QPolygonF& polygon);
};
}
