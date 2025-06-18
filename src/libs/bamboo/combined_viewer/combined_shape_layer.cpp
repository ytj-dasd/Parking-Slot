#include "combined_shape_layer.h"
#include <uface/base/uconverter.h>

namespace welkin::bamboo {
#define HeightRole Qt::UserRole + 11
CombinedShapeLayer::CombinedShapeLayer(const QString &name, QObject *parent)
        : QObject(parent) {
    _layer2d = new ShapeLayer2d(this);
    _layer3d = new ShapeLayer3d(this);
    setName(name);
    connect(_layer2d, SIGNAL(shapeAdded(const QString&)),
        this, SLOT(updateLayer3dShape(const QString&)));
    connect(_layer2d, SIGNAL(shapesAdded(const QList<QString>&)),
        this, SLOT(updateLayer3dShapes(const QList<QString>&)));
    connect(_layer2d, SIGNAL(shapeChanged(const QString&)),
        this, SLOT(updateLayer3dShape(const QString&)));
    connect(_layer2d, SIGNAL(shapeRemoved(const QString&)),
        this, SLOT(removeLayer3dShape(const QString&)));
    connect(_layer2d, SIGNAL(shapesRemoved(const QList<QString>&)),
        this, SLOT(removeLayer3dShapes(const QList<QString>&)));
    connect(_layer2d, SIGNAL(shapesCleared()), this, SLOT(clearLayer3dShapes()));
    connect(_layer2d, SIGNAL(penChanged()), this, SLOT(changeLayer3dColor()));
    changeLayer3dColor();
}
CombinedShapeLayer::~CombinedShapeLayer() {
    disconnect(_layer2d, SIGNAL(shapeAdded(const QString&)),
        this, SLOT(updateLayer3dShape(const QString&)));
    disconnect(_layer2d, SIGNAL(shapesAdded(const QList<QString>&)),
        this, SLOT(updateLayer3dShapes(const QList<QString>&)));
    disconnect(_layer2d, SIGNAL(shapeChanged(const QString&)),
               this, SLOT(updateLayer3dShape(const QString&)));
    disconnect(_layer2d, SIGNAL(shapeRemoved(const QString&)),
        this, SLOT(removeLayer3dShape(const QString&)));
    disconnect(_layer2d, SIGNAL(shapesRemoved(const QList<QString>&)),
        this, SLOT(removeLayer3dShapes(const QList<QString>&)));
    disconnect(_layer2d, SIGNAL(shapesCleared()), this, SLOT(clearLayer3dShapes()));
    disconnect(_layer2d, SIGNAL(penChanged()), this, SLOT(changeLayer3dColor()));
}

const common::Pose3d &CombinedShapeLayer::projectPose() const {
    return _proj_pose;
}

void CombinedShapeLayer::setProjectPose(const common::Pose3d &pose) {
    if (_proj_pose.isEqualTo(pose)) {return;}
    _proj_pose = pose;
}

ShapeLayer2d *CombinedShapeLayer::layer2d() const {
    return _layer2d;
}

ShapeLayer3d *CombinedShapeLayer::layer3d() const {
    return _layer3d;
}

const QString& CombinedShapeLayer::name() const {
    return _name;
}

void CombinedShapeLayer::setName(const QString &name) {
    if (name == this->name()) {return;}
    _name = name;
    _layer2d->setName(name);
    _layer3d->setName(UFQ(name));
    emit nameChanged();
}

QString CombinedShapeLayer::text() const {
    return _layer2d->text();
}

void CombinedShapeLayer::setText(const QString &text) {
    if (this->text() == text) {return;}
    _layer2d->setText(text);
    emit textChanged();
}

QPen CombinedShapeLayer::pen() const {
    return _layer2d->pen();
}

void CombinedShapeLayer::setPen(const QPen &pen) {
    _layer2d->setPen(pen);
}

QBrush CombinedShapeLayer::brush() const {
    return _layer2d->brush();
}

void CombinedShapeLayer::setBrush(const QBrush &brush) {
    _layer2d->setBrush(brush);
}

qreal CombinedShapeLayer::zValue() const {
    return _layer2d->zValue();
}

void CombinedShapeLayer::setZValue(const qreal zvalue) {
    _layer2d->setZValue(zvalue);
}

bool CombinedShapeLayer::isVisible() const {
    return _layer2d->isVisible();
}

void CombinedShapeLayer::setVisible(bool visible) {
    if (isVisible() == visible) {return;}
    _layer2d->setVisible(visible);
    _layer3d->setVisible(visible);
    emit visibleChanged(visible);
}

bool CombinedShapeLayer::isLocked() const {
    return _layer2d->isLocked();
}

void CombinedShapeLayer::setLocked(bool locked) {
    if (isLocked() == locked) {return;}
    _layer2d->setLocked(locked);
    emit lockedChanged(locked);
}

bool CombinedShapeLayer::hasShape(const QString &id) const {
    return _layer2d->hasShape(id);
}

void CombinedShapeLayer::removeShape(const QString &id) {
    _layer2d->removeShape(id);
}

void CombinedShapeLayer::clearShapes() {
    _layer2d->clearShapes();
}

bool CombinedShapeLayer::hasPoint(const QString &id) const {
    return _layer2d->hasPoint(id);
}

common::Point3d CombinedShapeLayer::getPoint(const QString &id) const {
    auto item = _layer2d->getPointItem(id);
    if (!item) {return common::Point3d();}
    double z = getHeight(item);
    auto point = _layer2d->getPoint(id);
    return common::Point3d(point.x(), point.y(), z);
}

QMap<QString, common::Point3d> CombinedShapeLayer::getPointMap() const {
    auto item_map = _layer2d->getPointItemMap();
    QMap<QString, common::Point3d> point_map;
    auto ids = item_map.keys();
    for (auto& id : ids) {
        point_map.insert(id, getPoint(id));
    }
    return point_map;
}

bool CombinedShapeLayer::addPoint(
        const common::Point3d &point, const QString &id) {
    auto item = _layer2d->addPoint(QPointF(point.x, point.y), id);
    if (!item) {return false;}
    item->setData(HeightRole, point.z);
    return true;
}

bool CombinedShapeLayer::addPoint(
        const common::Point2d &point, double z, const QString &id) {
    return addPoint(common::Point3d(point.x, point.y, z), id);
}

bool CombinedShapeLayer::updatePoint(
        const QString &id, const common::Point3d &point) {
    auto item = _layer2d->getPointItem(id);
    if (!item) {return false;}
    item->setData(HeightRole, point.z);
    return _layer2d->updatePoint(id, QPointF(point.x, point.y));
}

bool CombinedShapeLayer::updatePoint(
        const QString &id, const common::Point2d &point, double z) {
    return updatePoint(id, common::Point3d(point.x, point.y, z));
}

bool CombinedShapeLayer::hasPolyline(const QString &id) const {
    return _layer2d->hasPolyline(id);
}

common::Polyline3d CombinedShapeLayer::getPolyline(const QString &id) const {
    auto item = _layer2d->getPolylineItem(id);
    if (!item) {return common::Polyline3d();}
    double z = getHeight(item);
    auto polyline = _layer2d->getPolyline(id);
    common::Polyline3d polyline3d;
    polyline3d.fromPolyline2(UFQ(polyline), z);
    return polyline3d;
}

QMap<QString, common::Polyline3d> CombinedShapeLayer::getPolylineMap() const {
    auto item_map = _layer2d->getPolylineItemMap();
    QMap<QString, common::Polyline3d> polyline_map;
    auto ids = item_map.keys();
    for (auto& id : ids) {
        polyline_map.insert(id, getPolyline(id));
    }
    return polyline_map;
}

bool CombinedShapeLayer::addPolyline(
        const common::Polyline2d& polyline, double z, const QString &id) {
    auto item = _layer2d->addPolyline(UTQ(polyline.points), id);
    if (!item) {return false;}
    item->setData(HeightRole, z);
    return true;
}

bool CombinedShapeLayer::updatePolyline(
        const QString &id, const common::Polyline2d& polyline, double z) {
    auto item = _layer2d->getPolylineItem(id);
    if (!item) {return false;}
    item->setData(HeightRole, z);
    return _layer2d->updatePolyline(id, UTQ(polyline.points));
}

bool CombinedShapeLayer::hasPolygon(const QString &id) const {
    return _layer2d->hasPolygon(id);
}

common::Polygon3d CombinedShapeLayer::getPolygon(const QString &id) const {
    auto item = _layer2d->getPolygonItem(id);
    if (!item) {return common::Polygon3d();}
    double z = getHeight(item);
    auto polygon = _layer2d->getPolygon(id);
    common::Polygon3d polygon3d;
    polygon3d.fromPolygon2(UFQ(polygon), z);
    return polygon3d;
}

QMap<QString, common::Polygon3d> CombinedShapeLayer::getPolygonMap() const {
    auto item_map = _layer2d->getPolygonItemMap();
    QMap<QString, common::Polygon3d> polygon_map;
    auto ids = item_map.keys();
    for (auto& id : ids) {
        polygon_map.insert(id, getPolygon(id));
    }
    return polygon_map;
}

bool CombinedShapeLayer::addPolygon(
        const common::Polygon2d& polygon, double z, const QString &id) {
    auto item = _layer2d->addPolygon(UTQ(polygon.points), id);
    if (!item) {return false;}
    item->setData(HeightRole, z);
    return true;
}

bool CombinedShapeLayer::updatePolygon(
        const QString &id, const common::Polygon2d& polygon, double z) {
    auto item = _layer2d->getPolygonItem(id);
    if (!item) {return false;}
    item->setData(HeightRole, z);
    return _layer2d->updatePolygon(id, UTQ(polygon.points));
}

/// protected slots
void CombinedShapeLayer::changeLayer3dColor() {
    _layer3d->setColor(_layer2d->pen().color());
}

void CombinedShapeLayer::changeLayer3dVisible() {
    _layer3d->setVisible(_layer2d->isVisible());
}

void CombinedShapeLayer::updateLayer3dShape(const QString &id) {
    Eigen::Matrix4d prj_mat = _proj_pose.toMatrix();
    if (_layer2d->hasPoint(id)) {
        auto point = this->getPoint(id);
        _layer3d->updatePoint(UFQ(id), point, prj_mat, 0.1);
    } else if (_layer2d->hasPolyline(id)) {
        auto polyline = this->getPolyline(id);
        _layer3d->updatePolyline(UFQ(id), polyline.points, prj_mat, false);
    } else if (_layer2d->hasPolygon(id)) {
        auto polygon = this->getPolygon(id);
        _layer3d->updatePolyline(UFQ(id), polygon.points, prj_mat, true);
    }
}

void CombinedShapeLayer::updateLayer3dShapes(const QList<QString> &ids) {
    for (auto& id : ids) {updateLayer3dShape(id);}
}

void CombinedShapeLayer::removeLayer3dShape(const QString &id) {
    if (_layer3d->hasChild(UFQ(id))) {_layer3d->removeChild(UFQ(id));}
}

void CombinedShapeLayer::removeLayer3dShapes(const QList<QString> &ids) {
    for (auto& id : ids) {removeLayer3dShape(id);}
}

void CombinedShapeLayer::clearLayer3dShapes() {
    _layer3d->removeAllChildren();
}

/// protected functions
double CombinedShapeLayer::getHeight(QGraphicsItem *item) const {
    if (!item) {return 0.0;}
    auto data = item->data(HeightRole);
    auto has_data = data.isValid() && data.canConvert<double>();
    return has_data ? data.toDouble() : 0.0;
}
}
