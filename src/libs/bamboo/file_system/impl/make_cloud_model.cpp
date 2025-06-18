#include "bamboo/file_system/origin_data_folder.h"
#include "bamboo/base/osg_utils.h"
#include <common/base/dir.h>
#include <osg/PagedLOD>
#include <osgDB/WriteFile>

namespace welkin::bamboo {

namespace impl {
bool WriteCloudToOSGNode(const std::string& filename_out,
        const PointCloudPtr& cloud_in,
        const point_cloud::ColorRender& color_render,
        point_cloud::ColorRender::Mode mode) {
    return osgDB::writeNodeFile(
        *(point_cloud::CreateGeodeNode(*cloud_in, color_render, mode)), filename_out);
}

void AddToPagedNode(osg::ref_ptr<osg::PagedLOD>& paged_node,
        const std::string& save_path, int& index,
        PointCloudPtr& cloud_in,
        float min_dist, float max_dist,
        const point_cloud::ColorRender& color_render,
        point_cloud::ColorRender::Mode mode) {
    RETURN_IF(cloud_in->empty())
    std::stringstream ss; ss << "_l_" << index << ".osgb";
    std::string save_file = save_path + "/" + ss.str();
    RETURN_IF(!WriteCloudToOSGNode(save_file, cloud_in, color_render, mode))
    paged_node->setFileName(index, ss.str());
    paged_node->setRange(index, min_dist, max_dist);
    paged_node->setMinimumExpiryTime(index, 10);
    paged_node->setMinimumExpiryFrames(index, 30);
    ++index;
}
void AddChildToPagedNode(osg::ref_ptr<osg::PagedLOD>& paged_node,
        const std::string& save_path, int& index,
        const common::Boxf& bbox,
        PointCloudPtr& cloud_in,
        const point_cloud::ColorRender& color_render,
        point_cloud::ColorRender::Mode mode) {
    RETURN_IF(cloud_in->empty())
    if (bbox.getMaxLength() > 10 || cloud_in->size() > 100000) {
        common::Boxf bbox_left, bbox_right;
        {
            PointCloudPtr cloud_left(new PointCloud());
            common::point_cloud::SplitByMaxAxis(*cloud_in, *cloud_left, *cloud_in, bbox);
            bbox.splitByMaxAxis(bbox_left, bbox_right);
            AddChildToPagedNode(paged_node, save_path, index, bbox_left, cloud_left, color_render, mode);
        }
        AddChildToPagedNode(paged_node, save_path, index, bbox_right, cloud_in, color_render, mode);
        return;
    }
    osg::ref_ptr<osg::LOD> child_node = new osg::LOD();
    auto cnt_pt = bbox.getCenter();
    child_node->setCenter(osg::Vec3(cnt_pt.x, cnt_pt.y, cnt_pt.z));
    child_node->setRadius(bbox.getRadius());
    {
        PointCloudPtr cloud_1(new PointCloud());
        PointCloudPtr cloud_2(new PointCloud());
        PointCloudPtr cloud_3(new PointCloud());
        common::point_cloud::SplitByIndex(*cloud_in, *cloud_1, *cloud_in, 6);
        common::point_cloud::SplitByIndex(*cloud_in, *cloud_2, *cloud_in, 4);
        common::point_cloud::SplitByIndex(*cloud_in, *cloud_3, *cloud_in, 3);
        child_node->addChild(point_cloud::CreateGeodeNode(*cloud_1, color_render, mode), 0, 90);
        child_node->addChild(point_cloud::CreateGeodeNode(*cloud_2, color_render, mode), 0, 70);
        child_node->addChild(point_cloud::CreateGeodeNode(*cloud_3, color_render, mode), 0, 50);
        child_node->addChild(point_cloud::CreateGeodeNode(*cloud_in, color_render, mode), 0, 30);
    }

    std::stringstream ss; ss << "_l_" << index << ".osgb";
    std::string save_file = save_path + "/" + ss.str();
    osgDB::writeNodeFile(*child_node, save_file);
    paged_node->setFileName(index, ss.str());
    paged_node->setRange(index, 0, 100);
    paged_node->setMinimumExpiryTime(index, 10);
    paged_node->setMinimumExpiryFrames(index, 30);
    ++index;
}
bool WriteCloudToPagedNode(const std::string& save_path,
        const common::Boxf& bbox,
        PointCloudPtr& cloud_in,
        const point_cloud::ColorRender& color_render,
        point_cloud::ColorRender::Mode mode) {
    osg::ref_ptr<osg::PagedLOD> paged_node = new osg::PagedLOD();
    auto cnt_pt = bbox.getCenter();
    paged_node->setCenter(osg::Vec3(cnt_pt.x, cnt_pt.y, cnt_pt.z));
    paged_node->setRadius(bbox.getRadius());
    int index = 0;
    PointCloudPtr cloud_left(new PointCloud());
    common::point_cloud::SpiltByVoxel(
        *cloud_in, bbox, *cloud_left, *cloud_in, 0.5f, 0.5f, 0.5f);
    AddToPagedNode(paged_node, save_path, index, cloud_left, 0, 1e30, color_render, mode);

    std::vector<float> max_dist_vec;
    std::vector<float> leaf_size_vec;
    for (int i = 200; i < 1000; i += 300) {
        max_dist_vec.push_back(i);
        leaf_size_vec.push_back(0.2);
    }
    for (int i = 100; i < 200; i += 50) {
        max_dist_vec.push_back(i);
        leaf_size_vec.push_back(0.1);
    }
    for (std::size_t i = 0; i < max_dist_vec.size(); ++i) {
        float ls = leaf_size_vec[i];
        common::point_cloud::SpiltByVoxel(*cloud_in, bbox,
            *cloud_left, *cloud_in, ls, ls, ls);
        AddToPagedNode(paged_node, save_path, index,
            cloud_left, 0, max_dist_vec[i], color_render, mode);
    }
    AddChildToPagedNode(paged_node, save_path, index, bbox, cloud_in, color_render, mode);
    osgDB::writeNodeFile(*paged_node, save_path + "/_l.osgb");
    // osgDB::writeNodeFile(*paged_node, save_path + "/_l.osg");
    return true;
}
}

void OriginDataFolder::makeCloudModel(int color_mode, double min_ratio, double max_ratio) {
    auto model_list_folder = this->getModelListFolder();
    // 先清除
    model_list_folder->removeAllChildren();
    std::string save_path = model_list_folder->getPath();
    
    UInfo(QObject::tr("Save path: %1").arg(save_path.c_str()));

    auto cloud_list_folder = this->getCloudListFolder();
    
    using ColorMode = point_cloud::ColorRender::Mode;
    ColorMode mode = ColorMode(color_mode);
    point_cloud::ColorRender color_render(point_cloud::ColorMapManager::Get());
    color_render.setMinMaxRatio(min_ratio, max_ratio);
    if (mode == ColorMode::MODE_FIELD_Z) {
        auto bbox = cloud_list_folder->getBoundingBox();
        color_render.setRange(bbox.range_z.cast<float>());
    } else if (mode == ColorMode::MODE_FIELD_X) {
        auto bbox = cloud_list_folder->getBoundingBox();
        color_render.setRange(bbox.range_x.cast<float>());
    } else if (mode == ColorMode::MODE_FIELD_Y) {
        auto bbox = cloud_list_folder->getBoundingBox();
        color_render.setRange(bbox.range_y.cast<float>());
    } else if (mode == ColorMode::MODE_FIELD_INTENSITY) {
        color_render.setRange(common::Rangef(0, 255));
    } else {
        // DO NOTHING   
    }

    std::atomic<int> finished_num{0};
    int child_count = cloud_list_folder->getChildSize();
    #pragma omp parallel for num_threads(6)
    for (int i = 0; i < child_count; ++i) {
        auto cloud_folder = cloud_list_folder->getChild(i);
        auto cloud = cloud_folder->getSourceCloud();
        auto bbox = cloud_folder->getBoundingBox();
        auto name = cloud_folder->getName();
        cloud_folder->reset();
        std::string sub_path = save_path + "/_" + name;
        common::Dir(sub_path).mkdir();
        impl::WriteCloudToPagedNode(sub_path, bbox.cast<float>(), cloud, color_render, mode);
        ++finished_num;
        UProgressTextValue(QObject::tr("Make Model"), finished_num.load(), child_count);
    }
    // 写索引
    std::ofstream outfile;
    outfile.open(save_path + "/all.desc");
    for (int i = 0; i < child_count; ++i) {
        auto cloud_folder = cloud_list_folder->getChild(i);
        auto name = cloud_folder->getName();
        outfile << "_" << name << "/_l.osgb" << std::endl;
        ++finished_num;
        UProgressTextValue(QObject::tr("%1").arg(i), finished_num.load(), child_count);
    }
    outfile.close();
}
}