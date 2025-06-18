#include "building_folder.h"
#include "origin_data_folder.h"
#include "uface/base/uconverter.h"

namespace welkin::bamboo {
void BuildingSectionHorizontalFolder::setSection(const common::Sectiond& section) {
    this->getSectionFiler()->store(section);
    auto abc = section.line.getABC();
    this->getPlaneFiler()->store(common::Planed(abc[0], 0, abc[1], abc[2]));
    Eigen::Matrix4d swap_mat; swap_mat.setIdentity();
    swap_mat(1, 1) = 0; swap_mat(2, 2) = 0;
    swap_mat(1, 2) = 1; swap_mat(2, 1) = -1;
    Eigen::Matrix4d mat_line = section.line.toMatrix();
    // x 偏移回去，y不偏移，坐标Z归化到0平面
    mat_line(0, 3) += section.line.getCenter().x;
    // 级联的目的是为了达到点云绕投影面中心点旋转
    Eigen::Matrix4d mat_pose = swap_mat.inverse() * mat_line * swap_mat;
    common::Pose3d pose; pose.fromMatrix(mat_pose);
    this->getPose3dFiler()->store(pose.inverse());
}

void BuildingSectionVerticalFolder::setSection(const common::Sectiond& section) {
    this->getSectionFiler()->store(section);
    auto abc = section.line.getABC();
    this->getPlaneFiler()->store(common::Planed(abc[0], abc[1], 0, abc[2]));
    Eigen::Matrix4d mat_line = section.line.toMatrix();
    Eigen::Matrix4d swap_mat; swap_mat.setIdentity();
    swap_mat(1, 1) = 0; swap_mat(2, 2) = 0;
    swap_mat(1, 2) = 1; swap_mat(2, 1) = -1;
    Eigen::Matrix4d mat_pose = swap_mat * mat_line;
    common::Pose3d pose; pose.fromMatrix(mat_pose);
    this->getPose3dFiler()->store(pose.inverse());
}

////////////////////////////////
void BuildingFolder::extractCloudFromOriginCloudList(
        OriginDataCloudListFolder* cloud_list) {
    auto contour = this->getContour();
    std::string name = this->getName();
    if (!contour.isValid()) {
        UWarn(QObject::tr("Contour of building %1 is unvalid!").arg(UTQ(name)));
        return;
    }
    auto cloud_ptr = cloud_list->extractCloudInPolygon(contour);
    if (cloud_ptr->empty()) {
        UWarn(QObject::tr("No cloud int building contour: %1").arg(UTQ(name)));
    }
    this->setCloud(cloud_ptr);
}
void BuildingFolder::projectToBirdView(double res_x, double res_y, 
        ProjectType type, double min_value, double max_value, int color_map) {
    auto contour = this->getContour();
    if (!contour.isValid()) {
        UWarn(QObject::tr("contour of building %1 is unvalid!").arg(UTQ(this->getName())));
        return;
    }
    auto cloud = this->getCloud();
    if (!cloud.get() || cloud->empty()) {
        UWarn(QObject::tr("Cloud of building %1 is unvalid!").arg(UTQ(this->getName())));
        return;
    }
    auto bbox = this->getBoundingBox();
    cv::Mat image_out;
    common::point_cloud::ImageProjector<PointX>(
        bbox.cast<float>(), *cloud, image_out, res_x, res_y, 
        type, min_value, max_value, color_map);
    // algorithm::ImageMaxValueScale(image_gray);
    auto bird_view_folder = this->getBirdViewFolder();
    double origin_x = bbox.getMinX();
    double origin_y = bbox.getMinY();
    bird_view_folder->setData(image_out, origin_x, origin_y, res_x, res_y);
}
void BuildingFolder::projectToFrontView(double res_x, double res_y, 
        ProjectType type, double min_value, double max_value, int color_map) {
    Eigen::Matrix4d swap_mat; swap_mat.setIdentity();
    swap_mat(1, 1) = 0; swap_mat(2, 2) = 0;
    swap_mat(1, 2) = 1; swap_mat(2, 1) = -1;
    // 将Y/Z对调
    auto cloud = this->getCloudFolder()->getTransformCloud(swap_mat);
    if (!cloud.get() || cloud->empty()) {
        UWarn(QObject::tr("Cloud of building %1 is unvalid!").arg(UTQ(this->getName())));
        return;
    }
    auto bbox = common::point_cloud::ComputeBoundingBox(*cloud);
    // auto bbox = this->getBoundingBox();
    // 这里Y/Z对调，同时Z取反号
    // common::Boxd swap_box = bbox;
    // swap_box.range_y.min_v = bbox.range_z.min_v;
    // swap_box.range_y.max_v = bbox.range_z.max_v;
    // swap_box.range_z.min_v = -bbox.range_y.max_v;
    // swap_box.range_z.max_v = -bbox.range_y.min_v;
    cv::Mat image_out;
    common::point_cloud::ImageProjector<PointX>(
        bbox, *cloud, image_out, res_x, res_y, 
        type, min_value, max_value, color_map);
    auto front_view_folder = this->getFrontViewFolder();
    double origin_x = bbox.getMinX();
    double origin_y = bbox.getMinY();
    front_view_folder->setData(image_out, origin_x, origin_y, res_x, res_y);
}

void BuildingFolder::extractSectionCloud(BuildingSectionHorizontalFolder* section_folder) {
    auto name = this->getName();
    auto section_name = section_folder->getName();
    auto section = section_folder->getSection();
    if (!section.isValid()) {
        UWarn(QObject::tr("Building %1 section in %2").arg(UTQ(name)).arg(UTQ(section_name)));
        return;
    }
    auto cloud_ptr = this->getCloud();
    Eigen::Matrix4d swap_mat; swap_mat.setIdentity();
    swap_mat(1, 1) = 0; swap_mat(2, 2) = 0;
    swap_mat(1, 2) = 1; swap_mat(2, 1) = -1;
    PointCloudPtr cloud_out(new PointCloud());
    common::point_cloud::SectionFilter<PointX>(
        *cloud_ptr, *cloud_out, section.cast<float>(), swap_mat.cast<float>());
    section_folder->setCloud(cloud_out);
}
void BuildingFolder::extractSectionCloud(BuildingSectionVerticalFolder* section_folder) {
    auto name = this->getName();
    auto section_name = section_folder->getName();
    auto section = section_folder->getSection();
    if (!section.isValid()) {
        UWarn(QObject::tr("Building %1 section in %2").arg(UTQ(name)).arg(UTQ(section_name)));
        return;
    }
    auto cloud_ptr = this->getCloud();
    PointCloudPtr cloud_out(new PointCloud());
    common::point_cloud::SectionFilter<PointX>(
        *cloud_ptr, *cloud_out, section.cast<float>());
    if (cloud_out->empty()) {
        UWarn(QObject::tr("Building %1 section in %2 has no point cloud!!").arg(UTQ(name)).arg(UTQ(section_name)));
    } else {
        section_folder->setCloud(cloud_out);
    }
}
void BuildingFolder::projectSectionImage(BuildingSectionHorizontalFolder* section_folder,
        double res_x, double res_y, ProjectType type,
        double min_value, double max_value, int color_map) {
    // 水平的采用竖直投影，这里假设的是所有水平的是严格与地面水平的，所有投影图大小相同
    auto bbox = this->getBoundingBox();
    auto section_cloud = section_folder->getSectionCloud();
    // 注意Z值范围,section的Z值是相对值
    auto section = section_folder->getSection();
    bbox.range_z = section.range;
    cv::Mat image_out;
    common::point_cloud::ImageProjector<PointX>(
        bbox.cast<float>(), *section_cloud, image_out, res_x, res_y, 
        type, min_value, max_value, color_map);
    auto image_folder = section_folder->getImageFolder();
    double origin_x = bbox.getMinX();
    double origin_y = bbox.getMinY();
    image_folder->setData(image_out, origin_x, origin_y, res_x, res_y);
}
void BuildingFolder::projectSectionImage(BuildingSectionVerticalFolder* section_folder,
        double res_x, double res_y, ProjectType type,
        double min_value, double max_value, int color_map) {
    //采用自身坐标系投影
    cv::Mat image_out;
    auto section_cloud = section_folder->getSectionCloud();
    auto bbox = common::point_cloud::ComputeBoundingBox(*section_cloud);
    common::point_cloud::ImageProjector<PointX>(
        bbox, *section_cloud, image_out, res_x, res_y, 
        type, min_value, max_value, color_map);
    auto image_folder = section_folder->getImageFolder();
    double origin_x = bbox.getMinX();
    double origin_y = bbox.getMinY();
    image_folder->setData(image_out, origin_x, origin_y, res_x, res_y);
}
////////////
std::list<BuildingSectionHorizontalFolder*>
BuildingListFolder::getAllHorizontalSections() {
    std::list<BuildingSectionHorizontalFolder*> section_list;
    int building_size = this->getChildSize();
    for (int i = 0; i < building_size; ++i) {
        auto building_folder = this->getChild(i);
        auto sections_folder = building_folder->getSectionHorizontalListFolder();
        int section_size = sections_folder->getChildSize();
        for (int j = 0; j < section_size; ++j) {
            auto section_folder = sections_folder->getChild(j);
            section_list.push_back(section_folder);
        }
    }
    return section_list;
}
std::list<BuildingSectionVerticalFolder*>
BuildingListFolder::getAllVerticalSections() {
    std::list<BuildingSectionVerticalFolder*> section_list;
    int building_size = this->getChildSize();
    for (int i = 0; i < building_size; ++i) {
        auto building_folder = this->getChild(i);
        auto sections_folder = building_folder->getSectionVerticalListFolder();
        int section_size = sections_folder->getChildSize();
        for (int j = 0; j < section_size; ++j) {
            auto section_folder = sections_folder->getChild(j);
            section_list.push_back(section_folder);
        }
    }
    return section_list;
}
}
