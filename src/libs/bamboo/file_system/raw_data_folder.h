#pragma once
#include <bamboo/file_system/types.h>
#include "uface/logger/ulogger.h"
#include "uface/state/ustate.h"

#include "common/file_system/folder/pose_cloud_folder.h"
#include "common/file_system/folder/rect_image_folder.h"

namespace welkin::bamboo {

using RawDataCloudFolder = 
    common::file_system::TypePoseCloudFolder<ITEM_RAW_DATA_CLOUD_FOLDER, PointX>;

class BAMBOO_EXPORT RawDataCloudListFolder :
    public common::file_system::TypeGroup<ITEM_RAW_DATA_CLOUD_LIST_FOLDER, RawDataCloudFolder> {
public:
    RawDataCloudListFolder(const std::string& name, Folder* parent = nullptr)
        : common::file_system::TypeGroup<ITEM_RAW_DATA_CLOUD_LIST_FOLDER, RawDataCloudFolder>(name, parent) {}
    virtual ~RawDataCloudListFolder() {}
   
    common::Boxd getBoundingBox();
    common::Rectd getBoundingRect();
    // typename PointCloud::Ptr stitchAndFilter(float leaf_x, float leaf_y, float leaf_z);
};

/////////////////////////
class BAMBOO_EXPORT RawDataFolder : public common::file_system::TypeGroup<ITEM_RAW_DATA_FOLDER, RawDataCloudListFolder> {
public:
    RawDataFolder(const std::string& name, Folder* parent = nullptr)
        : common::file_system::TypeGroup<ITEM_RAW_DATA_FOLDER, RawDataCloudListFolder>(name, parent) {
    }
    virtual ~RawDataFolder() {}

    common::Boxd getBoundingBox();
    // 将二维重组为一维
    std::vector<RawDataCloudFolder*> getScanFolderList();
};
}
