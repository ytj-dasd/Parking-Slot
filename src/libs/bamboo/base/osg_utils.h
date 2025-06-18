#pragma once
#include <vector>

#include <osg/Geode>
#include <osg/Geometry>
#include <uface/state/ustate.h>
#include <common/base/color.h>
#include <bamboo/base/macro.h>
#include <bamboo/base/types.h>
namespace welkin::bamboo::point_cloud {

class BAMBOO_EXPORT ColorMap {
public:
    using Ptr = std::shared_ptr<ColorMap>;
    ColorMap();
    virtual ~ColorMap();

    inline const osg::Vec4& getColor(const int idx) const {
        if (idx < 0) {return _color_table[0];}
        else if (idx >= _color_table.size()) {
            return _color_table[_color_table.size() - 1];}
        else {return _color_table[idx];}
    }
protected:
    float _line_a = 1.f;
    float _line_b = 0.f;
    std::vector<osg::Vec4> _color_table;
    friend class ColorRender;
};

class BAMBOO_EXPORT RainbowColorMap : public ColorMap {
public:
    RainbowColorMap();
    virtual ~RainbowColorMap();
};

class BAMBOO_EXPORT ColorMapManager {
public:
    virtual ~ColorMapManager();
    static ColorMapManager* Instance();
    
    enum Type {
        RAINBOW = 0
    };

    static ColorMap* Get(Type type = RAINBOW);
private:
    std::unordered_map<Type, ColorMap::Ptr> _color_map;
private:
    ColorMapManager();
};
class BAMBOO_EXPORT ColorRender {
public:
    using Ptr = std::shared_ptr<ColorRender>;
    explicit ColorRender(ColorMap* color_map);
    virtual ~ColorRender();

    enum Mode {
        MODE_ORIGIN = 0,
        MODE_FIELD_X = 1,
        MODE_FIELD_Y = 2,
        MODE_FIELD_Z = 3,
        MODE_FIELD_INTENSITY = 4,
        MODE_RANDOM = 5,
    };

    void setRange(float min_v, float max_v);
    void setRange(const common::Rangef& range);
    void setLineRatio(float line_a, float line_b);

    void setMinMaxRatio(float min_ratio, float max_ratio);

    inline const osg::Vec4& getColor(const float v) const {
        int idx = std::floor(v * _line_a + _line_b);
        return _color_map->getColor(idx);
    }
private:
    float _line_a = 1.f;
    float _line_b = 0.f;
    float _min_ratio = 0.05;
    float _max_ratio = 0.9;
    ColorMap* _color_map;
};

template<typename PointT>
osg::Geometry* CreateGeometryWithOrigin(
        const pcl::PointCloud<PointT>& cloud) {
    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
    int pt_num = cloud.size();
    osg::ref_ptr<osg::Vec3Array> point_vec = new osg::Vec3Array();
    osg::ref_ptr<osg::Vec4Array> color_vec = new osg::Vec4Array();
    point_vec->reserve(pt_num);
    color_vec->reserve(pt_num);
    for (auto& pt : cloud.points) {
        point_vec->push_back(osg::Vec3(pt.x, pt.y, pt.z));
        color_vec->push_back(osg::Vec4(pt.r / 255.f, pt.g / 255.f, pt.b / 255.f, 1.f));
    }
    // 设置顶点数组
    geometry->setVertexArray(point_vec.get());
    geometry->setColorArray(color_vec.get(), osg::Array::Binding::BIND_PER_VERTEX);
    // geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    // 设置关联方式
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, pt_num));
    return geometry.release();
}
#define CONVERT_TO_GEOMETRY_WIDTH_FILED_T(T, t)                                           \
template<typename PointT>                                                                 \
osg::Geometry* CreateGeometryWithField##T(                                                \
        const pcl::PointCloud<PointT>& cloud,                                             \
        const ColorRender& color_render) {                                                \
    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();                           \
    int pt_num = cloud.size();                                                            \
    osg::ref_ptr<osg::Vec3Array> point_vec = new osg::Vec3Array();                        \
    osg::ref_ptr<osg::Vec4Array> color_vec = new osg::Vec4Array();                        \
    point_vec->reserve(pt_num);                                                           \
    color_vec->reserve(pt_num);                                                           \
    for (auto& pt : cloud.points) {                                                       \
        point_vec->push_back(osg::Vec3(pt.x, pt.y, pt.z));                                \
        color_vec->push_back(color_render.getColor(pt.t));                                \
    }                                                                                     \
    geometry->setVertexArray(point_vec.get());                                            \
    geometry->setColorArray(color_vec.get(), osg::Array::Binding::BIND_PER_VERTEX);       \
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, pt_num)); \
    return geometry.release();                                                                      \
}

CONVERT_TO_GEOMETRY_WIDTH_FILED_T(X, x)
CONVERT_TO_GEOMETRY_WIDTH_FILED_T(Y, y)
CONVERT_TO_GEOMETRY_WIDTH_FILED_T(Z, z)
CONVERT_TO_GEOMETRY_WIDTH_FILED_T(I, intensity)

// 随机色
template<typename PointT>
osg::Geometry* CreateGeometryWithRandom(
        const pcl::PointCloud<PointT>& cloud) {
    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
    int pt_num = cloud.size();
    osg::ref_ptr<osg::Vec3Array> point_vec = new osg::Vec3Array();
    osg::ref_ptr<osg::Vec4Array> color_vec = new osg::Vec4Array();
    point_vec->reserve(pt_num);
    for (auto& pt : cloud.points) {
        point_vec->push_back(osg::Vec3(pt.x, pt.y, pt.z));
    }
    common::Color clr = common::Color::Rand();
    color_vec->push_back(osg::Vec4(clr.rF(), clr.gF(), clr.bF(), 1.0));
    // 设置顶点数组
    geometry->setVertexArray(point_vec.get());
    geometry->setColorArray(color_vec.get(), osg::Array::Binding::BIND_OVERALL);
    // 设置关联方式
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, pt_num));
    return geometry.release();
}

BAMBOO_EXPORT osg::Geode* CreateGeodeNode(
    const PointCloud& cloud,
    const ColorRender& color_render,
    ColorRender::Mode mode);

}