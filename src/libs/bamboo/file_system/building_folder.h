#pragma once
#include <bamboo/file_system/types.h>
#include <common/file_system/filer/point_cloud_filer.h>
#include <common/file_system/filer/proto_filer.h>

#include <common/file_system/group.h>

#include <common/file_system/folder/rect_image_folder.h>
#include <common/file_system/folder/plane_cloud_folder.h>
#include <common/file_system/folder/box_cloud_folder.h>
#include <uface/base/uconverter.h>
#include "uface/logger/ulogger.h"
namespace welkin::bamboo {

using BuildingContourFiler =
    common::file_system::TypePolygon2dFiler<ITEM_BUILDING_CONTOUR_FILER>;

using BuildingCloudFolder =
    common::file_system::TypeBoxCloudFolder<ITEM_BUILDING_CLOUD_FOLDER, PointX>;

using BuildingBirdViewFolder =
    common::file_system::TypeRectImageFolder<ITEM_BUILDING_BIRD_VIEW_FOLDER>;

using BuildingFrontViewFolder =
    common::file_system::TypeRectImageFolder<ITEM_BUILDING_FRONT_VIEW_FOLDER>;

// using BuildingWallCloudFolder =
//     common::file_system::TypePlaneCloudFolder<ITEM_BUILDING_WALL_CLOUD_FOLDER, PointX>;
// 
// class BuildingWallCloudListFolder :
//     public common::file_system::TypeGroup<ITEM_BUILDING_WALL_CLOUD_LIST_FOLDER, BuildingWallCloudFolder> {
// public:
//     BuildingWallCloudListFolder(const std::string& name, Folder* parent = nullptr)
//         : common::file_system::TypeGroup<ITEM_BUILDING_WALL_CLOUD_LIST_FOLDER, BuildingWallCloudFolder>(name, parent) {}
//     virtual ~BuildingWallCloudListFolder() {}
// 
// };
//////////////
template <int32_t item_type,
    typename PlaneFilerT, typename Pose3dFilerT,
    typename SectionFilerT, typename LayerFilerT,
    typename CloudFolderT, typename ImageFolderT>
class TypeBuildingSectionFolder :
    public common::file_system::TypeFolder<item_type,
    common::file_system::TypeFilerFinder<
        PlaneFilerT, Pose3dFilerT,
        SectionFilerT, LayerFilerT>,
    common::file_system::TypeFolderFinder<
        CloudFolderT, ImageFolderT>> {
public:
    TypeBuildingSectionFolder(const std::string& name, Folder* parent = nullptr)
        : common::file_system::TypeFolder<item_type,
            common::file_system::TypeFilerFinder<
                PlaneFilerT, Pose3dFilerT,
                SectionFilerT, LayerFilerT>,
            common::file_system::TypeFolderFinder<
                CloudFolderT, ImageFolderT>>(name, parent) {
        this->initFilers("pb.plane", "pb.pose", "pb.section", "pb.layer");
        this->initFolders("cloud", "image");
    }
    virtual ~TypeBuildingSectionFolder() {}

    const common::Planed& getPlane() {
        return this->getPlaneFiler()->load();
    }
    const common::Pose3d& getPose3d() {
        return this->getPose3dFiler()->load();
    }
    const common::Sectiond& getSection() {
        return this->getSectionFiler()->load();
    }

    const common::Layerd& getLayer() {
        return this->getLayerFiler()->load();
    }
    void setLayer(const common::Layerd& layer) {
        this->getLayerFiler()->store(layer);
    }
    void setCloud(const PointCloudPtr& cloud) {
        this->getCloudFolder()->setSourceCloud(cloud);
        this->getCloudFolder()->recomputeBoundingBox();
    }
    PointCloudPtr getCloud() {
        return this->getCloudFolder()->getSourceCloud();
    }
    // 获取Section坐标系下的点云
    PointCloudPtr getSectionCloud() {
        common::Pose3d pose = this->getPose3d().inverse();
        auto section_cloud_folder = this->getCloudFolder();
        return section_cloud_folder->getTransformCloud(pose.toMatrix());
    }
    common::Boxd getBoundingBox() {
        return this->getCloudFolder()->getBoundingBox();
    }
    PlaneFilerT* getPlaneFiler() {
        return this->template getTypeFiler<0>();
    }
    const PlaneFilerT* getPlaneFiler() const {
        return this->template getTypeFiler<0>();
    }
    Pose3dFilerT* getPose3dFiler() {
        return this->template getTypeFiler<1>();
    }
    const Pose3dFilerT* getPose3dFiler() const {
        return this->template getTypeFiler<1>();
    }
    SectionFilerT* getSectionFiler() {
        return this->template getTypeFiler<2>();
    }
    const SectionFilerT* getSectionFiler() const {
        return this->template getTypeFiler<2>();
    }

    LayerFilerT* getLayerFiler() {
        return this->template getTypeFiler<3>();
    }
    const LayerFilerT* getLayerFiler() const {
        return this->template getTypeFiler<3>();
    }

    CloudFolderT* getCloudFolder() {
        return this->template getTypeFolder<0>();
    }
    const CloudFolderT* getCloudFolder() const {
        return this->template getTypeFolder<0>();
    }
    ImageFolderT* getImageFolder() {
        return this->template getTypeFolder<1>();
    }
    const ImageFolderT* getImageFolder() const {
        return this->template getTypeFolder<1>();
    }
};

using BaseBuildingSectionHorizontalFolder =
    TypeBuildingSectionFolder<ITEM_BUILDING_SECTION_HORIZONTAL_FOLDER,
    common::file_system::TypePlaneFiler<ITEM_BUILDING_SECTION_HORIZONTAL_PLANE_FILER>,
    common::file_system::TypePose3dFiler<ITEM_BUILDING_SECTION_HORIZONTAL_POSE3D_FILER>,
    common::file_system::TypeSectionFiler<ITEM_BUILDING_SECTION_HORIZONTAL_SECTION_FILER>,
    common::file_system::TypeLayerFiler<ITEM_BUILDING_SECTION_HORIZONTAL_LAYER_FILER>,
    common::file_system::TypeBoxCloudFolder<ITEM_BUILDING_SECTION_HORIZONTAL_CLOUD_FOLDER, PointX>,
    common::file_system::TypeRectImageFolder<ITEM_BUILDING_SECTION_HORIZONTAL_IMAGE_FOLDER>>;

class BAMBOO_EXPORT BuildingSectionHorizontalFolder : public BaseBuildingSectionHorizontalFolder {
public:
    BuildingSectionHorizontalFolder(const std::string& name, Folder* parent = nullptr)
        : BaseBuildingSectionHorizontalFolder(name, parent) {}
    virtual ~BuildingSectionHorizontalFolder() {}
    void setSection(const common::Sectiond& section);
};

using BuildingSectionHorizontalListFolder =
    common::file_system::TypeGroup<ITEM_BUILDING_SECTION_HORIZONTAL_LIST_FOLDER, BuildingSectionHorizontalFolder>;

using BaseBuildingSectionVerticalFolder =
    TypeBuildingSectionFolder<ITEM_BUILDING_SECTION_VERTICAL_FOLDER,
    common::file_system::TypePlaneFiler<ITEM_BUILDING_SECTION_VERTICAL_PLANE_FILER>,
    common::file_system::TypePose3dFiler<ITEM_BUILDING_SECTION_VERTICAL_POSE3D_FILER>,
    common::file_system::TypeSectionFiler<ITEM_BUILDING_SECTION_VERTICAL_SECTION_FILER>,
    common::file_system::TypeLayerFiler<ITEM_BUILDING_SECTION_VERTICAL_LAYER_FILER>,
    common::file_system::TypeBoxCloudFolder<ITEM_BUILDING_SECTION_VERTICAL_CLOUD_FOLDER, PointX>,
    common::file_system::TypeRectImageFolder<ITEM_BUILDING_SECTION_VERTICAL_IMAGE_FOLDER>>;
class BAMBOO_EXPORT BuildingSectionVerticalFolder : public BaseBuildingSectionVerticalFolder {
public:
    BuildingSectionVerticalFolder(const std::string& name, Folder* parent = nullptr)
        : BaseBuildingSectionVerticalFolder(name, parent) {}
    virtual ~BuildingSectionVerticalFolder() {}
    void setSection(const common::Sectiond& section);
};

using BuildingSectionVerticalListFolder =
    common::file_system::TypeGroup<ITEM_BUILDING_SECTION_VERTICAL_LIST_FOLDER, BuildingSectionVerticalFolder>;

///////////////
using BaseBuildingFolder =
    common::file_system::TypeFolder<
    ITEM_BUILDING_FOLDER,
    common::file_system::TypeFilerFinder<
        BuildingContourFiler>,
    common::file_system::TypeFolderFinder<
        BuildingCloudFolder,
        BuildingBirdViewFolder,
        BuildingFrontViewFolder,
        BuildingSectionHorizontalListFolder,
        BuildingSectionVerticalListFolder
        >
    >;

class OriginDataCloudListFolder;
class BAMBOO_EXPORT BuildingFolder : public BaseBuildingFolder {
public:
    BuildingFolder(const std::string& name, Folder* parent = nullptr)
        : BaseBuildingFolder(name, parent) {
        // this->initFilers("contour.polygon");
        // this->initFolders("cloud", "bird_view", "front_view",
        //     "horiontal_sections", "vertical_sections");
        this->initFilers(UFQ("轮廓.poly"));
        this->initFolders(UFQ("点云"), UFQ("俯视图"), UFQ("前视图"),
            UFQ("水平截面"), UFQ("垂直截面"));
    }
    virtual ~BuildingFolder() {}

    void setContour(const common::Polygon2d& polygon) {
        this->getContourFiler()->store(polygon);
    }
    const common::Polygon2d& getContour() {
        return this->getContourFiler()->load();
    }
    void setCloud(const PointCloudPtr& cloud) {
        this->getCloudFolder()->setSourceCloud(cloud);
        this->getCloudFolder()->recomputeBoundingBox();
    }
    PointCloudPtr getCloud() {
        return this->getCloudFolder()->getSourceCloud();
    }
    common::Boxd getBoundingBox() {
        return this->getCloudFolder()->getBoundingBox();
    }
    common::Rectd getBoundingRect() {
        return this->getCloudFolder()->getBoundingRect();
    }
    void extractCloudFromOriginCloudList(OriginDataCloudListFolder* cloud_list);
    void projectToBirdView(double res_x, double res_y, 
        ProjectType type = ProjectType::PROJECT_NUM,
        double min_value = 3, double max_value = 128, 
        int color_map = cv::COLORMAP_HOT);
    void projectToFrontView(double res_x, double res_y, 
        ProjectType type = ProjectType::PROJECT_RGB_NUM,
        double min_value = 0, double max_value = 128, 
        int color_map = -1);

    void extractSectionCloud(BuildingSectionHorizontalFolder* section_folder);
    void extractSectionCloud(BuildingSectionVerticalFolder* section_folder);
    void projectSectionImage(BuildingSectionHorizontalFolder* section_folder,
        double res_x = 0.01, double res_y = 0.01,
        ProjectType type = ProjectType::PROJECT_NUM,
        double min_value = 3, double max_value = 192, 
        int color_map = -1);
    void projectSectionImage(BuildingSectionVerticalFolder* section_folder,
        double res_x = 0.01, double res_y = 0.01,
        ProjectType type = ProjectType::PROJECT_RGB_NUM,
        double min_value = 3, double max_value = 192,
        int color_map = -1);
    
    /////
    BuildingContourFiler* getContourFiler() {
        return this->template getTypeFiler<0>();
    }
    const BuildingContourFiler* getContourFiler() const {
        return this->template getTypeFiler<0>();
    }
    BuildingCloudFolder* getCloudFolder() {
        return this->template getTypeFolder<0>();
    }
    const BuildingCloudFolder* getCloudFolder() const {
        return this->template getTypeFolder<0>();
    }

    BuildingBirdViewFolder* getBirdViewFolder() {
        return this->template getTypeFolder<1>();
    }
    const BuildingBirdViewFolder* getBirdViewFolder() const {
        return this->template getTypeFolder<1>();
    }
    BuildingFrontViewFolder* getFrontViewFolder() {
        return this->template getTypeFolder<2>();
    }
    const BuildingFrontViewFolder* getFrontViewFolder() const {
        return this->template getTypeFolder<2>();
    }
    BuildingSectionHorizontalListFolder* getSectionHorizontalListFolder() {
        return this->template getTypeFolder<3>();
    }
    const BuildingSectionHorizontalListFolder* getSectionHorizontalListFolder() const {
        return this->template getTypeFolder<3>();
    }
    BuildingSectionVerticalListFolder* getSectionVerticalListFolder() {
        return this->template getTypeFolder<4>();
    }
    const BuildingSectionVerticalListFolder* getSectionVerticalListFolder() const {
        return this->template getTypeFolder<4>();
    }
};

class BAMBOO_EXPORT BuildingListFolder :
    public common::file_system::TypeGroup<ITEM_BUILDING_LIST_FOLDER, BuildingFolder> {
public:
    BuildingListFolder(const std::string& name, Folder* parent = nullptr)
        : common::file_system::TypeGroup<ITEM_BUILDING_LIST_FOLDER, BuildingFolder>(name, parent) {
    }
    virtual ~BuildingListFolder() {}

    std::list<BuildingSectionHorizontalFolder*>
    getAllHorizontalSections();
    std::list<BuildingSectionVerticalFolder*>
    getAllVerticalSections();
};
}
