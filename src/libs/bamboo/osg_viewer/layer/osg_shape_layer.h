#pragma once
#include "bamboo/osg_viewer/layer/osg_geode_layer.h"
namespace welkin::bamboo {
class BAMBOO_EXPORT OSGShapeLayer : public OSGGeodeLayer {
    Q_OBJECT
public:
    OSGShapeLayer(QObject* parent = nullptr);
    virtual ~OSGShapeLayer();
    // 设置颜色
    void setColor(const QColor& color);

    // 添加点 (存在更新，不存在则添加)
    bool updatePoint(const std::string& id,
        const common::Point3d& point, double radius = 0.1f);
    bool updatePoint(const std::string& id,
        const common::Point2d& point, double z, double radius = 0.1f);
    bool updatePoint(const std::string& id,
        const common::Point3d& point,
        const Eigen::Matrix4d& proj_mat, double radius = 0.1f);
    bool updatePoint(const std::string& id,
        const common::Point2d& point, double z,
        const Eigen::Matrix4d& proj_mat, double radius = 0.1f);

    // 添加多线 (存在更新，不存在则添加)
    bool updatePolyline(const std::string& id,
        const std::vector<common::Point3d>& points, bool closed = false);
    bool updatePolyline(const std::string& id,
        const std::vector<common::Point3d>& points,
        const Eigen::Matrix4d& proj_mat, bool closed = false);
    bool updatePolyline(const std::string& id,
        const std::vector<common::Point2d>& points, double z, bool closed = false);
    bool updatePolyline(const std::string& id,
        const std::vector<common::Point2d>& points, double z,
        const Eigen::Matrix4d& proj_mat, bool closed = false);
protected:
    osg::ref_ptr<osg::Vec4Array> _color_vec;
};
}
