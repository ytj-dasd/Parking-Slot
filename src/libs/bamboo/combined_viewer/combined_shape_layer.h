#pragma once
#include <QObject>
#include "bamboo/image_viewer/layer/draw_shape_layer.h"
#include "bamboo/osg_viewer/layer/osg_shape_layer.h"
namespace welkin::bamboo {
using ShapeLayer2d = DrawShapeLayer;
using ShapeLayer3d = OSGShapeLayer;
// 二维三维联合图层（二维主动层，三维被动层）
class BAMBOO_EXPORT CombinedShapeLayer : public QObject {
    Q_OBJECT
public:
    CombinedShapeLayer(const QString& name, QObject* parent = nullptr);
    virtual ~CombinedShapeLayer();

    const common::Pose3d& projectPose() const;
    // 设置投影坐标
    void setProjectPose(const common::Pose3d& pose);

    // Note: 二维图层为主动层；三维图层为被动显示层
    ShapeLayer2d* layer2d() const;
    ShapeLayer3d* layer3d() const;

    const QString& name() const;
    QString text() const;
    bool isVisible() const;
    bool isLocked() const;

    QPen pen() const;
    void setPen(const QPen& pen);
    QBrush brush() const;
    void setBrush(const QBrush& brush);
    qreal zValue() const;
    void setZValue(const qreal zvalue);

    bool hasShape(const QString& id) const;
    void removeShape(const QString& id);
    void clearShapes();

    bool hasPoint(const QString& id) const;
    common::Point3d getPoint(const QString& id) const;
    QMap<QString, common::Point3d> getPointMap() const;
    bool addPoint(const common::Point3d& point,
        const QString& id = QString());
    bool addPoint(const common::Point2d& point, double z = 0.0,
        const QString& id = QString());
    bool updatePoint(const QString& id,
        const common::Point3d& point);
    bool updatePoint(const QString& id,
        const common::Point2d& point, double z = 0.0);

    bool hasPolyline(const QString& id) const;
    common::Polyline3d getPolyline(const QString& id) const;
    QMap<QString, common::Polyline3d> getPolylineMap() const;
    // 要求各点高度相同
    bool addPolyline(const common::Polyline2d& polyline, double z = 0.0,
        const QString& id = QString());
    bool updatePolyline(const QString& id,
        const common::Polyline2d& polyline, double z = 0.0);

    bool hasPolygon(const QString& id) const;
    common::Polygon3d getPolygon(const QString& id) const;
    QMap<QString, common::Polygon3d> getPolygonMap() const;
    bool addPolygon(const common::Polygon2d& polygon, double z = 0.0,
        const QString& id = QString());
    bool updatePolygon(const QString& id,
        const common::Polygon2d& polygon, double z = 0.0);
Q_SIGNALS:
    void nameChanged();
    void textChanged();
    void visibleChanged(bool visible);
    void lockedChanged(bool locked);

public Q_SLOTS:
    void setName(const QString& name);
    void setText(const QString& text);
    void setVisible(bool visible);
    void setLocked(bool locked);

protected Q_SLOTS:
    void changeLayer3dColor();
    void changeLayer3dVisible();
    virtual void updateLayer3dShape(const QString& id);
    virtual void updateLayer3dShapes(const QList<QString>& ids);
    virtual void removeLayer3dShape(const QString& id);
    virtual void removeLayer3dShapes(const QList<QString>& ids);
    virtual void clearLayer3dShapes();
protected:
    // 获取高度值
    double getHeight(QGraphicsItem* item) const;
protected:
    QString _name = QString();
    ShapeLayer2d* _layer2d = nullptr;
    ShapeLayer3d* _layer3d = nullptr;
    common::Pose3d _proj_pose;
};
}
