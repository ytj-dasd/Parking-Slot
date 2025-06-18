#include "raw_data_folder.h"

namespace welkin::bamboo {
common::Boxd RawDataCloudListFolder::getBoundingBox() {
    std::size_t child_count = this->getChildSize();
    common::Boxd bbox;
    for (std::size_t i = 0; i < child_count; ++i) {
        auto child = this->getChild(i);
        auto sub_box = child->getBoundingBox();
        CONTINUE_IF(!sub_box.isValid())
        if (bbox.isValid()) {
            bbox.fitBox(sub_box);
        } else {
            bbox = sub_box;
        }
    }
    return bbox;
}
common::Rectd RawDataCloudListFolder::getBoundingRect() {
    common::Boxd bbox = this->getBoundingBox();
    return common::Rectd(bbox.range_x, bbox.range_y);
}
// typename RawDataCloudListFolder::PointCloudT::Ptr 
// RawDataCloudListFolder::stitchAndFilter(float leaf_x, float leaf_y, float leaf_z) {
//     int child_size = this->getChildSize();
//     if (child_size < 1) {
//         UWarn(QObject::tr("No child!"));
//         typename PointCloudT::Ptr cloud_empty(new PointCloudT());
//         return cloud_empty;
//     }
//     algorithm::PointCloudVoxelFilter<PointT> vf;
//     common::Boxd bbox = this->getBoundingBox();
//     vf.init(bbox.cast<float>(), leaf_x, leaf_y, leaf_z);
//     for (int i = 0; i < child_size; ++i) {
//         UProgressTextValue(QObject::tr("Process..."), i, child_size);
//         auto child = this->getChild(i);
//         auto sub_cloud = child->getSourceCloud();
//         vf.filter(sub_cloud);
//         child->reset();
//     }
//     return vf.getCloud();
// }

////////////
common::Boxd RawDataFolder::getBoundingBox() {
    std::size_t child_count = this->getChildSize();
    common::Boxd bbox;
    for (std::size_t i = 0; i < child_count; ++i) {
        auto child = this->getChild(i);
        auto sub_box = child->getBoundingBox();
        CONTINUE_IF(!sub_box.isValid())
        if (bbox.isValid()) {
            bbox.fitBox(sub_box);
        } else {
            bbox = sub_box;
        }
    }
    return bbox;
}
std::vector<RawDataCloudFolder*> RawDataFolder::getScanFolderList() {
    int total_size = 0;
    auto data_size = this->getChildSize();
    for (int i = 0; i < data_size; ++i) {
        auto scans_folder = this->getChild(i);
        total_size += scans_folder->getChildSize();
    }
    std::vector<RawDataCloudFolder*> scan_folder_list;
    scan_folder_list.reserve(total_size);
    for (int i = 0; i < data_size; ++i) {
        auto scans_folder = this->getChild(i);
        int scans_size = scans_folder->getChildSize();
        for (int j = 0; j < scans_size; ++j) {
            auto scan_folder = scans_folder->getChild(j);
            scan_folder_list.push_back(scan_folder);
        }
    }
    return scan_folder_list;
}
}