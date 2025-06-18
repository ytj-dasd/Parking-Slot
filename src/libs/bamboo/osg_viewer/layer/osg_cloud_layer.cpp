#include "osg_cloud_layer.h"
#include <common/base/dir.h>
#include <common/base/file.h>
#include <common/base/color.h>

#include <common/point_cloud/utils.h>
#include "bamboo/osg_viewer/osg_viewer.h"
namespace welkin::bamboo {
OSGCloudLayer::OSGCloudLayer(QObject* parent) : OSGGroupLayer(parent) {
    
}
OSGCloudLayer::~OSGCloudLayer() {

}
bool OSGCloudLayer::init() {
    _viewer->insertComboBoxToToolBar();
    return true;
}
osg::LOD* OSGCloudLayer::createLodNode(
        PointCloudPtr& cloud_in,
        const common::Boxf& bbox,
        const point_cloud::ColorRender& color_render,
        point_cloud::ColorRender::Mode mode) {
    osg::ref_ptr<osg::LOD> lod_node = new osg::LOD();
    auto cnt_pt = bbox.getCenter();
    lod_node->setCenter(osg::Vec3(cnt_pt.x, cnt_pt.y, cnt_pt.z));
    lod_node->setRadius(bbox.getRadius());
    std::vector<float> leaf_size_vec = { 0.5, 0.25, 0.15, 0.1, 0.05};
    std::vector<float> max_dist_vec  = {1e10,  500,  300, 200,  100};

    PointCloudPtr cloud_left(new PointCloud());
    for (std::size_t i = 0; i < leaf_size_vec.size(); ++i) {
        float leaf_size = leaf_size_vec[i];
        float max_dist = max_dist_vec[i];
        if (cloud_in->empty()) {
            return lod_node.release();
        } else if (i > 0 && cloud_in->size() < 10000) {
            lod_node->addChild(point_cloud::CreateGeodeNode(*cloud_in, color_render, mode), 0, max_dist);
            return lod_node.release();
        } else {
            common::point_cloud::SpiltByVoxel(*cloud_in, bbox, *cloud_left, *cloud_in, leaf_size, leaf_size, leaf_size);
            lod_node->addChild(point_cloud::CreateGeodeNode(*cloud_left, color_render, mode), 0, max_dist);
        }
    }
    if (cloud_in->empty()) {
        // DO NOTHING
    } else if (cloud_in->size() < 20000) {
        lod_node->addChild(point_cloud::CreateGeodeNode(*cloud_in, color_render, mode), 0, 90);
    } else if (cloud_in->size() < 40000) {
        common::point_cloud::SplitByIndex(*cloud_in, *cloud_left, *cloud_in, 2);
        lod_node->addChild(point_cloud::CreateGeodeNode(*cloud_left, color_render, mode), 0, 90);
        lod_node->addChild(point_cloud::CreateGeodeNode(*cloud_in, color_render, mode), 0, 70);
    } else if (cloud_in->size() < 60000) {
        common::point_cloud::SplitByIndex(*cloud_in, *cloud_left, *cloud_in, 3);
        lod_node->addChild(point_cloud::CreateGeodeNode(*cloud_left, color_render, mode), 0, 90);
        common::point_cloud::SplitByIndex(*cloud_in, *cloud_left, *cloud_in, 2);
        lod_node->addChild(point_cloud::CreateGeodeNode(*cloud_left, color_render, mode), 0, 70);
        lod_node->addChild(point_cloud::CreateGeodeNode(*cloud_in, color_render, mode), 0, 50);
    } else {
        common::point_cloud::SplitByIndex(*cloud_in, *cloud_left, *cloud_in, 4);
        lod_node->addChild(point_cloud::CreateGeodeNode(*cloud_left, color_render, mode), 0, 90);
        common::point_cloud::SplitByIndex(*cloud_in, *cloud_left, *cloud_in, 3);
        lod_node->addChild(point_cloud::CreateGeodeNode(*cloud_left, color_render, mode), 0, 70);
        common::point_cloud::SplitByIndex(*cloud_in, *cloud_left, *cloud_in, 2);
        lod_node->addChild(point_cloud::CreateGeodeNode(*cloud_left, color_render, mode), 0, 50);
        lod_node->addChild(point_cloud::CreateGeodeNode(*cloud_in, color_render, mode), 0, 30);
    }
    return lod_node.release();
}

bool OSGCloudLayer::updateCloud(const std::string& id, const PointCloudPtr cloud_in, const common::Boxf& bbox) {
    using ColorMode = point_cloud::ColorRender::Mode;
    auto color_mode = _viewer->getColorMode();
    UStateProcessBar([&, color_mode, id, cloud_in, bbox](){
        point_cloud::ColorRender color_render(point_cloud::ColorMapManager::Get());
        if (color_mode == ColorMode::MODE_FIELD_Z) {
            color_render.setRange(bbox.range_z);
        } else if (color_mode == ColorMode::MODE_FIELD_X) {
            color_render.setRange(bbox.range_x);
        } else if (color_mode == ColorMode::MODE_FIELD_Y) {
            color_render.setRange(bbox.range_y);
        } else if (color_mode == ColorMode::MODE_FIELD_INTENSITY) {
            auto range_i = common::point_cloud::ComputeRangeI(*cloud_in);
            color_render.setRange(range_i);
        } else {
            // DO NOTHING
        }
        osg::ref_ptr<osg::Group> group_node = new osg::Group();
        float max_length = 10.f;
        std::vector<PointCloudPtr> sub_cloud_list;
        common::point_cloud::SplitToCloudListByVoxel(
            *cloud_in, sub_cloud_list, bbox, max_length, max_length, max_length);
        
        std::atomic<int> finished_num{0};
        int total_count = sub_cloud_list.size();
        int max_point_num = 100000;
        std::mutex group_mutex;
        #pragma omp parallel for num_threads(3)
        for (int i = 0; i < total_count; ++i) {
            auto sub_cloud = sub_cloud_list[i];
            sub_cloud_list[i] = nullptr;
            CONTINUE_IF(sub_cloud->empty())
            ++finished_num;
            UProgressTextValue("", finished_num.load(), total_count);
            auto bbox = common::point_cloud::ComputeBoundingBox(*sub_cloud);
            if (sub_cloud->size() > max_point_num) {
                std::vector<PointCloudPtr> kd_cloud_list;
                SplitToCloudListByKDTree(*sub_cloud, kd_cloud_list, bbox, max_length, max_point_num);
                sub_cloud = nullptr;
                for (auto& kd_cloud : kd_cloud_list) {
                    CONTINUE_IF(kd_cloud->empty())
                    auto kd_box = common::point_cloud::ComputeBoundingBox(*kd_cloud);
                    auto lod_node = createLodNode(kd_cloud, kd_box, color_render, color_mode);
                    kd_cloud = nullptr;
                    std::lock_guard<std::mutex> lock(group_mutex);
                    group_node->addChild(lod_node);
                }
            } else {
                auto lod_node = createLodNode(sub_cloud, bbox, color_render, color_mode);
                std::lock_guard<std::mutex> lock(group_mutex);
                group_node->addChild(lod_node);
            }
        }

        //
        this->removeChildToBuffer(id);
        this->addChildToBuffer(id, group_node);
        emit this->childChanged();
    });
    return true;
}
bool OSGCloudLayer::updateCloud(const std::string& id, const PointCloudPtr cloud_in) {
    if (cloud_in->empty()) {return false;}
    auto bbox = common::point_cloud::ComputeBoundingBox(*cloud_in);
    return updateCloud(id, cloud_in, bbox);
}
}