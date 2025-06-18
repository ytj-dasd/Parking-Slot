#pragma once
#include <osg/LOD>
#include <QComboBox>
#include "bamboo/base/osg_utils.h"
#include <bamboo/file_system/types.h>
#include "bamboo/osg_viewer/layer/osg_geode_layer.h"

namespace welkin::bamboo {
class BAMBOO_EXPORT OSGMonoCloudLayer : public OSGGeodeLayer {
    Q_OBJECT
public:
    OSGMonoCloudLayer(QObject* parent = nullptr);
    virtual ~OSGMonoCloudLayer();
    
    bool init() override;

    bool updateCloud(const PointCloudPtr cloud_in);
    bool updateCloud(const PointCloudPtr cloud_in, const common::Boxf& bbox);

signals:
    void cloudChanged();

private slots:
    void onCloudChanged();

    void onColorModeChanged(int mode);
protected:
    void remapCloud();
    void clearLayerData() override;
private:
    std::mutex _cloud_mutex;
    common::Boxf _bbox;
    PointCloudPtr _cloud = nullptr;
    osg::ref_ptr<osg::Vec3Array> _point_vec = nullptr;
    osg::ref_ptr<osg::Vec4Array> _color_vec = nullptr;
    osg::ref_ptr<osg::Geometry> _geometry = nullptr;

    using ColorMode = point_cloud::ColorRender::Mode;
    ColorMode _render_mode = ColorMode::MODE_ORIGIN;
    point_cloud::ColorMap::Ptr _color_map;
    point_cloud::ColorRender::Ptr _color_render;
};
}