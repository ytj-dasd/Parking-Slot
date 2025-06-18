#pragma once
#include <bamboo/file_system/types.h>
#include <uface/base/uconverter.h>
#include <uface/logger/ulogger.h>
#include <uface/state/ustate.h>
#include <common/geometry/polygon2.h>
#include <common/file_system/folder/box_cloud_folder.h>
#include <common/file_system/folder/rect_image_folder.h>
#include <common/file_system/folder/raster_image_folder.h>
#include <common/file_system/folder/point3d_pair_folder.h>

namespace welkin::bamboo {
using OriginDataLocalPoseFiler =
    common::file_system::TypePose3dFiler<ITEM_ORIGIN_DATA_LOCAL_POSE_FILER>;
using OriginDataGlobalPoseFiler =
    common::file_system::TypePose3dFiler<ITEM_ORIGIN_DATA_GLOBAL_POSE_FILER>;

using OriginDataBirdViewFolder =
    common::file_system::TypeRectImageFolder<ITEM_ORIGIN_DATA_BIRD_VIEW_FOLDER>;

using OriginDataFrontViewFolder =
    common::file_system::TypeRectImageFolder<ITEM_ORIGIN_DATA_FRONT_VIEW_FOLDER>;

using OriginDataFullCloudFolder = common::file_system::TypeBoxCloudFolder<
        ITEM_ORIGIN_DATA_FULL_CLOUD_FOLDER, PointX>;

using OriginDataCloudFolder =
    common::file_system::TypeBoxCloudFolder<ITEM_ORIGIN_DATA_CLOUD_FOLDER, PointX>;

class BAMBOO_EXPORT OriginDataCloudListFolder :
    public common::file_system::TypeGroup<ITEM_ORIGIN_DATA_CLOUD_LIST_FOLDER, OriginDataCloudFolder> {
public:
    OriginDataCloudListFolder(const std::string& name, Folder* parent = nullptr)
        : common::file_system::TypeGroup<ITEM_ORIGIN_DATA_CLOUD_LIST_FOLDER, OriginDataCloudFolder>(name, parent) {}
    virtual ~OriginDataCloudListFolder() {}

    // 左乘
    void applyPose(const common::Pose3d& delta_pose);
    common::Boxd getBoundingBox();
    common::Rectd getBoundingRect() {
        common::Boxd bbox = this->getBoundingBox();
        return common::Rectd(bbox.range_x, bbox.range_y);
    }
    PointCloudPtr stitchAndFilter(float leaf_x, float leaf_y, float leaf_z);
    PointCloudPtr extractCloudInPolygon(const common::Polygon2d& polygon);
    PointCloudPtr extractCloudInCircle(const common::Circled& circle);

    void projectToImages(cv::Mat& image_rgb,
        cv::Mat& image_d, cv::Mat& image_i,
        PointCloud& cloud_out,
        const common::Pose3d& pose,
        float cx, float cy, float fx, float fy,
        int width, int height, 
        float min_depth = 1.f,
        float max_depth = 100.f,
        float cloud_res = 0.02f);
};

using OriginDataOsgFiler =
    common::file_system::CommonFiler<ITEM_ORIGIN_DATA_OSG_FILER>;
using BaseOriginDataModelFolder =
    common::file_system::TypeOnlyFilerFolder<
    ITEM_ORIGIN_DATA_MODEL_FOLDER, OriginDataOsgFiler>;
class OriginDataModelFolder : public BaseOriginDataModelFolder {
public:
    OriginDataModelFolder(const std::string& name, Folder* parent = nullptr)
            : BaseOriginDataModelFolder(name, parent) {
        this->initFilers("_l.osgb");
    }
    virtual ~OriginDataModelFolder() {}

    std::string getOsgFilename() const {
        return this->template getTypeFiler<0>()->getFileName();
    }
};
class BAMBOO_EXPORT OriginDataModelListFolder :
    public common::file_system::TypeGroup<ITEM_ORIGIN_DATA_MODEL_LIST_FOLDER, OriginDataModelFolder> {
public:
    OriginDataModelListFolder(const std::string& name, Folder* parent = nullptr)
        : common::file_system::TypeGroup<ITEM_ORIGIN_DATA_MODEL_LIST_FOLDER, OriginDataModelFolder>(name, parent) {}
    virtual ~OriginDataModelListFolder() {}

    std::vector<std::string> getOsgFileList() const {
        int child_count = this->getChildSize();
        std::vector<std::string> file_list(child_count);
        for (int i = 0; i < child_count; ++i) {
            file_list[i] = this->getChild(i)->getOsgFilename();
        }
        return file_list;
    }
};

/////
// using OriginDataDOMFolder =
//     common::file_system::TypeRasterImageFolder<ITEM_ORIGIN_DATA_DOM_FOLDER>;
class BAMBOO_EXPORT OriginDataDOMFolder :
    public common::file_system::TypeRasterImageFolder<ITEM_ORIGIN_DATA_DOM_FOLDER> {
public:
    OriginDataDOMFolder(const std::string& name, Folder* parent = nullptr)
        : common::file_system::TypeRasterImageFolder<ITEM_ORIGIN_DATA_DOM_FOLDER>(name, parent) {}
    virtual ~OriginDataDOMFolder() {}
    bool extractImageByRect(cv::Mat& image, const common::Rectd& rect);
private:
    common::Rectd getIntersection(const common::Rectd& r1, const common::Rectd& r2) const;
    cv::Rect getCvRect(const common::Point2d& origin, const common::Rectd& rect, 
        double res_x = 0.02, double res_y = 0.02) const;
};

/////
using OriginDataControlPointFolder =
    common::file_system::TypePoint3dPairFolder<ITEM_ORIGIN_DATA_CONTROL_POINT_FOLDER>;

using BaseOriginDataControlPointListFolder =
    common::file_system::TypeGroup<ITEM_ORIGIN_DATA_CONTROL_POINT_LIST_FOLDER, 
        OriginDataControlPointFolder>;
class BAMBOO_EXPORT OriginDataControlPointListFolder 
    : public BaseOriginDataControlPointListFolder {
public:
    OriginDataControlPointListFolder(const std::string& name, Folder* parent = nullptr)
        : BaseOriginDataControlPointListFolder(name, parent) {}
    virtual ~OriginDataControlPointListFolder() {}

    void toPointPairList(common::Point3dList& src_points,
        common::Point3dList& tgt_points);
    void toPointPairList(common::Point2dList& src_points,
        common::Point2dList& tgt_points);
    bool solvePose3d(common::Pose3d& pose_out, std::vector<double>& error_list);
    bool solvePose2d(common::Pose2d& pose_out, std::vector<double>& error_list);
};

/////////////////////////
using BaseOriginDataFolder =
    common::file_system::TypeFolder<
    ITEM_ORIGIN_DATA_FOLDER,
    common::file_system::TypeFilerFinder<
        OriginDataLocalPoseFiler,
        OriginDataGlobalPoseFiler>,
    common::file_system::TypeFolderFinder<
        OriginDataBirdViewFolder,
        OriginDataFrontViewFolder,
        OriginDataFullCloudFolder,
        OriginDataModelListFolder,
        OriginDataDOMFolder,
        OriginDataCloudListFolder,
        OriginDataControlPointListFolder>
    >;

class BAMBOO_EXPORT OriginDataFolder : public BaseOriginDataFolder {
public:
    OriginDataFolder(const std::string& name, Folder* parent = nullptr)
        : BaseOriginDataFolder(name, parent) {
        this->initFilers(UFQ("局部坐标.pose"), UFQ("全局坐标.pose"));
        this->initFolders(UFQ("俯视图"), UFQ("前视图"), UFQ("预览点云"), 
            UFQ("模型点云"), UFQ("正射影像"), UFQ("分块点云"), UFQ("控制点"));
    }
    virtual ~OriginDataFolder() {}

    OriginDataLocalPoseFiler* getLocalPoseFiler() {
        return this->template getTypeFiler<0>();
    }
    const OriginDataLocalPoseFiler* getLocalPoseFiler() const {
        return this->template getTypeFiler<0>();
    }
    OriginDataGlobalPoseFiler* getGlobalPoseFiler() {
        return this->template getTypeFiler<1>();
    }
    const OriginDataGlobalPoseFiler* getGlobalPoseFiler() const {
        return this->template getTypeFiler<1>();
    }
    OriginDataBirdViewFolder* getBirdViewFolder() {
        return this->template getTypeFolder<0>();
    }
    const OriginDataBirdViewFolder* getBirdViewFolder() const {
        return this->template getTypeFolder<0>();
    }
    OriginDataFrontViewFolder* getFrontViewFolder() {
        return this->template getTypeFolder<1>();
    }
    const OriginDataFrontViewFolder* getFrontViewFolder() const {
        return this->template getTypeFolder<1>();
    }
    OriginDataFullCloudFolder* getFullCloudFolder() {
        return this->template getTypeFolder<2>();
    }
    const OriginDataFullCloudFolder* getFullCloudFolder() const {
        return this->template getTypeFolder<2>();
    }
    OriginDataModelListFolder* getModelListFolder() {
        return this->template getTypeFolder<3>();
    }
    const OriginDataModelListFolder* getModelListFolder() const {
        return this->template getTypeFolder<3>();
    }
    OriginDataDOMFolder* getDOMFolder() {
        return this->template getTypeFolder<4>();
    }
    const OriginDataDOMFolder* getDOMFolder() const {
        return this->template getTypeFolder<4>();
    }
    OriginDataCloudListFolder* getCloudListFolder() {
        return this->template getTypeFolder<5>();
    }
    const OriginDataCloudListFolder* getCloudListFolder() const {
        return this->template getTypeFolder<5>();
    }
    OriginDataControlPointListFolder* getControlPointListFolder() {
        return this->template getTypeFolder<6>();
    }
    const OriginDataControlPointListFolder* getControlPointListFolder() const {
        return this->template getTypeFolder<6>();
    }
    void projectToBirdView(double res_x = 0.1, double res_y = 0.1,
        ProjectType type = ProjectType::PROJECT_NUM,
        double min_value = 3, double max_value = 192,
        int color_map = cv::COLORMAP_HOT);
    void projectToFrontView(double res_x = 0.1, double res_y = 0.1,
        ProjectType type = ProjectType::PROJECT_RGB,
        double min_value = 3, double max_value = 192,
        int color_map = cv::COLORMAP_HOT);

    void projectToDOM(double min_z, double max_z, 
        double res_x = 0.01, double res_y = 0.01, 
        ProjectType type = ProjectType::PROJECT_RGB,
        double min_value = 0, double max_value = 128, 
        int color_map = -1);

    void makeCloudModel(int color_mode, double min_ratio, double max_ratio);
};
}
