#include "osg_mono_cloud_layer.h"
#include <common/base/dir.h>
#include <common/base/file.h>
#include <common/base/color.h>
#include <common/time/tic_toc.h>
#include <common/point_cloud/utils.h>
#include "bamboo/osg_viewer/osg_viewer.h"
namespace welkin::bamboo {
OSGMonoCloudLayer::OSGMonoCloudLayer(QObject* parent) : OSGGeodeLayer(parent) {
    _color_map.reset(new point_cloud::RainbowColorMap());
    _color_render.reset(new point_cloud::ColorRender(_color_map.get()));

    connect(this, SIGNAL(cloudChanged()), this, SLOT(onCloudChanged()));
}
OSGMonoCloudLayer::~OSGMonoCloudLayer() {
    
}
bool OSGMonoCloudLayer::init() {
    connect(_viewer, SIGNAL(renderModeChanged(int)),
        this, SLOT(onColorModeChanged(int)));
    return true;
}
bool OSGMonoCloudLayer::updateCloud(const PointCloudPtr cloud_in, const common::Boxf& bbox) {
    if (!cloud_in.get() || cloud_in->empty()) {
        this->removeAllChildren();
        return true;
    }
    {
        std::lock_guard<std::mutex> lock(_cloud_mutex);
        _bbox = bbox;
        _cloud = cloud_in;
    }
    _geometry = nullptr;
    _point_vec = nullptr;
    _color_vec = nullptr;
    
    this->remapCloud();
    return true;
}
bool OSGMonoCloudLayer::updateCloud(const PointCloudPtr cloud_in) {
    if (!cloud_in.get() || cloud_in->empty()) {
        this->removeAllChildren();
        return true;
    }
    auto bbox = common::point_cloud::ComputeBoundingBox(*cloud_in);
    this->updateCloud(cloud_in, bbox);
    return true;
}
void OSGMonoCloudLayer::remapCloud() {
    UStateProcessBar([&]() {   
        std::lock_guard<std::mutex> lock(_cloud_mutex);   
        std::size_t pt_num = _cloud->size();
        std::size_t step = pt_num / 25;
        int progress = 0;
        // if (!_geometry.get()) {
            _geometry = new osg::Geometry();
        // }
        if (!_point_vec.get() || _point_vec->size() != pt_num) {
            _point_vec = new osg::Vec3Array();
            _point_vec->resize(pt_num);
            for (std::size_t i = 0; i < pt_num; ++i) {
                if (i % step == 0) {
                    progress += 2;
                    UProgressTextValue("", progress, 100);
                }
                auto& pt = _cloud->points[i];
                (*_point_vec)[i][0] = pt.x;
                (*_point_vec)[i][1] = pt.y;
                (*_point_vec)[i][2] = pt.z;
            }
            _geometry->setVertexArray(_point_vec.get());
            _geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, pt_num));
        }
        UProgressTextValue("", 50, 100);
        if (!_color_vec.get() || _color_vec->size() != pt_num) {
            _color_vec = new osg::Vec4Array();
            _color_vec->resize(pt_num);
            _geometry->setColorArray(_color_vec.get());
        }
        if (_render_mode == ColorMode::MODE_ORIGIN) {
            for (std::size_t i = 0; i < pt_num; ++i) {
                if (i % step == 0) {
                    progress += 2;
                    UProgressTextValue("", progress, 100);
                }
                auto& pt = _cloud->points[i];
                (*_color_vec)[i] = osg::Vec4(pt.r / 255.f, pt.g / 255.f, pt.b / 255.f, 1.f);
            }
            _color_vec->setBinding(osg::Array::Binding::BIND_PER_VERTEX);
        } else if (_render_mode == ColorMode::MODE_FIELD_Z) {
            _color_render->setRange(_bbox.range_z);
            for (std::size_t i = 0; i < pt_num; ++i) {
                if (i % step == 0) {
                    progress += 2;
                    UProgressTextValue("", progress, 100);
                }
                auto& pt = _cloud->points[i];
                (*_color_vec)[i] = _color_render->getColor(pt.z);
            }
            _color_vec->setBinding(osg::Array::Binding::BIND_PER_VERTEX);
        } else if (_render_mode == ColorMode::MODE_FIELD_X) {
            _color_render->setRange(_bbox.range_x);
            for (std::size_t i = 0; i < pt_num; ++i) {
                if (i % step == 0) {
                    progress += 2;
                    UProgressTextValue("", progress, 100);
                }
                auto& pt = _cloud->points[i];
                (*_color_vec)[i] = _color_render->getColor(pt.x);
            }
            _color_vec->setBinding(osg::Array::Binding::BIND_PER_VERTEX);
        } else if (_render_mode == ColorMode::MODE_FIELD_Y) {
            _color_render->setRange(_bbox.range_y);
            for (std::size_t i = 0; i < pt_num; ++i) {
                if (i % step == 0) {
                    progress += 2;
                    UProgressTextValue("", progress, 100);
                }
                auto& pt = _cloud->points[i];
                (*_color_vec)[i] = _color_render->getColor(pt.y);
            }
            _color_vec->setBinding(osg::Array::Binding::BIND_PER_VERTEX);
        } else if (_render_mode == ColorMode::MODE_FIELD_INTENSITY) {
            auto range_i = common::point_cloud::ComputeRangeI(*_cloud);
            _color_render->setRange(range_i);
            for (std::size_t i = 0; i < pt_num; ++i) {
                if (i % step == 0) {
                    progress += 2;
                    UProgressTextValue("", progress, 100);
                }
                auto& pt = _cloud->points[i];
                (*_color_vec)[i] = _color_render->getColor(pt.intensity);
            }
            _color_vec->setBinding(osg::Array::Binding::BIND_PER_VERTEX);
        } else if (_render_mode == ColorMode::MODE_RANDOM) {
            common::Color clr = common::Color::Rand();
            (*_color_vec)[0] = osg::Vec4(clr.rF(), clr.gF(), clr.bF(), 1.0);
            _color_vec->setBinding(osg::Array::Binding::BIND_OVERALL);
        } else {
            // DO NOTHING   
            return;
        }
        UProgressTextValue("", 99, 100);
        emit this->cloudChanged();
    });
}
void OSGMonoCloudLayer::onCloudChanged() {
    std::lock_guard<std::mutex> lock(_cloud_mutex);
    this->addDrawable("points", _geometry.get());
}
void OSGMonoCloudLayer::onColorModeChanged(int mode) {
    _render_mode = ColorMode(mode);
    {
        std::lock_guard<std::mutex> lock(_cloud_mutex);
        if (!_cloud.get() || _cloud->empty()) {
            this->removeAllChildren();
            return;
        }
    }
    this->remapCloud();
}
void OSGMonoCloudLayer::clearLayerData() {
    _cloud = nullptr;

    _geometry = nullptr;
    _point_vec = nullptr;
    _color_vec = nullptr;
}
}