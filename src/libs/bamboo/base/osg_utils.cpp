#include "osg_utils.h"
namespace welkin::bamboo::point_cloud {
ColorMap::ColorMap() {
    _color_table.emplace_back(osg::Vec4(0.f, 0.f, 0.f, 1.f));
}
ColorMap::~ColorMap() {

}
//////////////
RainbowColorMap::RainbowColorMap() {
    _color_table.clear();
    _color_table.reserve(256 * 4);
    float step = 1.f / 255.f;
    for (float r = 0.0; r < 1.0; r += step) {
        _color_table.emplace_back(osg::Vec4(1.f, r, 0.f, 1.f));
    }
    for (float r = 0.0; r < 1.0; r += step) {
        _color_table.emplace_back(osg::Vec4(1.f - r, 1.f, 0.f, 1.f));
    }
    for (float r = 0.0; r < 1.0; r += step) {
        _color_table.emplace_back(osg::Vec4(0.f, 1.f, r, 1.f));
    }
    for (float r = 0.0; r < 1.0; r += step) {
        _color_table.emplace_back(osg::Vec4(0.f, 1.f - r, 1.f, 1.f));
    }
}
RainbowColorMap::~RainbowColorMap() {

}
//
ColorMapManager::ColorMapManager() {
    _color_map[RAINBOW].reset(new RainbowColorMap());
}
ColorMapManager::~ColorMapManager() {

}
ColorMapManager* ColorMapManager::Instance() {
    static ColorMapManager s_instance;
    return &s_instance;
}
ColorMap* ColorMapManager::Get(Type type) {
    auto iter = Instance()->_color_map.find(type);
    RETURN_NULL_IF(iter == Instance()->_color_map.end())
    return iter->second.get();
}
/////
ColorRender::ColorRender(ColorMap* color_map) : _color_map(color_map) {

}
ColorRender::~ColorRender() {

}
void ColorRender::setRange(float min_v, float max_v) {
    float min_vr = min_v + (max_v - min_v) * _min_ratio;
    float max_vr = min_v + (max_v - min_v) * _max_ratio;
    _line_a = std::max(_color_map->_color_table.size(), std::size_t(1)) 
        / std::max(max_vr - min_vr, 0.1f);
    _line_b = - min_vr * _line_a;
}
void ColorRender::setRange(const common::Rangef& range) {
    this->setRange(range.min_v, range.max_v);
}
void ColorRender::setLineRatio(float line_a, float line_b) {
    _line_a = line_a;
    _line_b = line_b;
}
void ColorRender::setMinMaxRatio(float min_ratio, float max_ratio) {
    _min_ratio = min_ratio;
    _max_ratio = max_ratio;
}

////////////////////
osg::Geode* CreateGeodeNode(
        const PointCloud& cloud, 
        const ColorRender& color_render,
        ColorRender::Mode mode) {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    switch (mode) {
    case ColorRender::Mode::MODE_ORIGIN:
        geode->addDrawable(CreateGeometryWithOrigin(cloud));
        break;
    case ColorRender::Mode::MODE_FIELD_X:
        geode->addDrawable(CreateGeometryWithFieldX(cloud, color_render));
        break;
    case ColorRender::Mode::MODE_FIELD_Y:
        geode->addDrawable(CreateGeometryWithFieldY(cloud, color_render));
        break;
    case ColorRender::Mode::MODE_FIELD_Z:
        geode->addDrawable(CreateGeometryWithFieldZ(cloud, color_render));
        break;
    case ColorRender::Mode::MODE_FIELD_INTENSITY:
        geode->addDrawable(CreateGeometryWithFieldI(cloud, color_render));
        break;
    case ColorRender::Mode::MODE_RANDOM:
        geode->addDrawable(CreateGeometryWithRandom(cloud));
        break;
    default:
        break;
    }
    return geode.release();
}
}