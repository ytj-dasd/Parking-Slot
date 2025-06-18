#include "osg_shape_layer.h"

namespace welkin::bamboo {
OSGShapeLayer::OSGShapeLayer(QObject *parent) : OSGGeodeLayer(parent) {
    _color_vec = new osg::Vec4Array();
    this->setColor(QColor(255, 255, 255));
}

OSGShapeLayer::~OSGShapeLayer() {

}

void OSGShapeLayer::setColor(const QColor &color) {
    _color_vec->clear();
    _color_vec->push_back(osg::Vec4(color.redF(),
        color.greenF(), color.blueF(), 1.0f));
    _color_vec->setBinding(osg::Array::BIND_OVERALL);
}

bool OSGShapeLayer::updatePoint(const std::string &id,
        const common::Point3d &point, double radius) {
    osg::ref_ptr<osg::Vec3Array> point_vec = new osg::Vec3Array();
    point_vec->push_back(osg::Vec3(point.x - radius, point.y, point.z));
    point_vec->push_back(osg::Vec3(point.x + radius, point.y, point.z));
    point_vec->push_back(osg::Vec3(point.x, point.y, point.z));
    point_vec->push_back(osg::Vec3(point.x, point.y - radius, point.z));
    point_vec->push_back(osg::Vec3(point.x, point.y + radius, point.z));
    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
    geometry->setVertexArray(point_vec.get());
    geometry->setColorArray(_color_vec.get());
    geometry->addPrimitiveSet(new osg::DrawArrays(
        osg::PrimitiveSet::LINE_STRIP, 0, point_vec->size()));
        // osg::PrimitiveSet::LINE_LOOP, 0, point_vec->size()));
    this->addDrawable(id, geometry.get());
    return true;
}

bool OSGShapeLayer::updatePoint(const std::string &id,
        const common::Point2d &point, double z, double radius) {
    common::Point3d point3(point.x, point.y, z);
    return updatePoint(id, point3, radius);
}

bool OSGShapeLayer::updatePoint(const std::string &id,
        const common::Point3d &point, const Eigen::Matrix4d &proj_mat, double radius) {
    auto trans_point = point;
    trans_point.transform(proj_mat);
    return updatePoint(id, trans_point, radius);
}

bool OSGShapeLayer::updatePoint(const std::string &id,
        const common::Point2d &point, double z,
        const Eigen::Matrix4d &proj_mat, double radius) {
    common::Point3d point3(point.x, point.y, z);
    point3.transform(proj_mat);
    return updatePoint(id, point3, radius);
}

bool OSGShapeLayer::updatePolyline(const std::string &id,
        const std::vector<common::Point3d> &points, bool closed) {
    osg::ref_ptr<osg::Vec3Array> point_vec = new osg::Vec3Array();
    point_vec->reserve(points.size());
    for (auto& pt : points) {
        point_vec->push_back(osg::Vec3(pt.x, pt.y, pt.z));
    }
    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
    geometry->setVertexArray(point_vec.get());
    geometry->setColorArray(_color_vec.get());

    if (closed) {
        geometry->addPrimitiveSet(new osg::DrawArrays(
            osg::PrimitiveSet::LINE_LOOP, 0, point_vec->size()));
    } else {
        geometry->addPrimitiveSet(new osg::DrawArrays(
            osg::PrimitiveSet::LINE_STRIP, 0, point_vec->size()));
    }
    this->addDrawable(id, geometry.get());
    return true;
}

bool OSGShapeLayer::updatePolyline(const std::string &id,
        const std::vector<common::Point3d> &points,
        const Eigen::Matrix4d &proj_mat, bool closed) {
    std::vector<common::Point3d> point_vec;
    point_vec.resize(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        point_vec[i] = points[i];
        point_vec[i].transform(proj_mat);
    }
    return updatePolyline(id, point_vec, closed);
}

bool OSGShapeLayer::updatePolyline(const std::string &id,
        const std::vector<common::Point2d> &points, double z, bool closed) {
    std::vector<common::Point3d> point_vec;
    point_vec.resize(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        point_vec[i].x = points[i].x;
        point_vec[i].y = points[i].y;
        point_vec[i].z = z;
    }
    return updatePolyline(id, point_vec, closed);
}

bool OSGShapeLayer::updatePolyline(const std::string &id,
        const std::vector<common::Point2d> &points, double z,
        const Eigen::Matrix4d &proj_mat, bool closed) {
    std::vector<common::Point3d> point_vec;
    point_vec.resize(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        point_vec[i].x = points[i].x;
        point_vec[i].y = points[i].y;
        point_vec[i].z = z;
        point_vec[i].transform(proj_mat);
    }
    return updatePolyline(id, point_vec, closed);
}
}
