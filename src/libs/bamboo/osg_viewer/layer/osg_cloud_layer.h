#pragma once
#include <osg/LOD>
#include <QComboBox>
#include "bamboo/base/osg_utils.h"
#include <bamboo/file_system/types.h>
#include "bamboo/osg_viewer/layer/osg_group_layer.h"

namespace welkin::bamboo {
class BAMBOO_EXPORT OSGCloudLayer : public OSGGroupLayer {
    Q_OBJECT
public:
    OSGCloudLayer(QObject* parent = nullptr);
    virtual ~OSGCloudLayer();
    bool init() override;

    bool updateCloud(const std::string& id, const PointCloudPtr cloud_in);
    bool updateCloud(const std::string& id, const PointCloudPtr cloud_in, const common::Boxf& bbox);

private:
    osg::LOD* createLodNode(
        PointCloudPtr& cloud_in,
        const common::Boxf& bbox,
        const point_cloud::ColorRender& color_render,
        point_cloud::ColorRender::Mode mode);
};
}