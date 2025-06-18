#pragma once
#include <memory>
#include <common/file_system/types.h>
#include <common/file_system/folder.h>
#include <common/file_system/filer/image_filer.h>
#include <common/file_system/filer/point_cloud_filer.h>
#include <common/file_system/filer/proto_filer.h>

#include "bamboo/base/macro.h"
#include "bamboo/base/types.h"

namespace welkin::bamboo {
// constexpr int32_t ITEM_FILER_USER = 1500;

const int32_t ITEM_DXF_FILER = 1651;

// constexpr int32_t ITEM_FILER_END = 1999;

// constexpr int32_t ITEM_FOLDER_USER = 2500;

constexpr int32_t ITEM_BAMBOO_PROJECT = 2521;

constexpr int32_t ITEM_BOX_CLOUD_FOLDER
    = common::file_system::ITEM_BOX_CLOUD_FOLDER;
constexpr int32_t ITEM_POSE_CLOUD_FOLDER
    = common::file_system::ITEM_POSE_CLOUD_FOLDER;

// 原始数据
constexpr int32_t ITEM_RAW_DATA_FOLDER = 2550;
// 原始点云
constexpr int32_t ITEM_RAW_DATA_CLOUD_FOLDER = 2551;
constexpr int32_t ITEM_RAW_DATA_CLOUD_LIST_FOLDER = 2552;

///////////////////////////////////////////////////
// 处理后的数据
constexpr int32_t ITEM_ORIGIN_DATA_FOLDER = 2560;
// 原始数据的LocalPose,相对于RawData
constexpr int32_t ITEM_ORIGIN_DATA_LOCAL_POSE_FILER = 2561;
// 原始数据的GlobalPose
constexpr int32_t ITEM_ORIGIN_DATA_GLOBAL_POSE_FILER = 2562;
// 原始数据预览图
constexpr int32_t ITEM_ORIGIN_DATA_BIRD_VIEW_FOLDER = 2563;
// 原始数据预览图
constexpr int32_t ITEM_ORIGIN_DATA_FRONT_VIEW_FOLDER = 2564;
// 原始数据预览点云
constexpr int32_t ITEM_ORIGIN_DATA_FULL_CLOUD_FOLDER = 2565;

constexpr int32_t ITEM_ORIGIN_DATA_CLOUD_FOLDER = 2566;
// 原始数据点云列表
constexpr int32_t ITEM_ORIGIN_DATA_CLOUD_LIST_FOLDER = 2567;

constexpr int32_t ITEM_ORIGIN_DATA_OSG_FILER = 2568;
constexpr int32_t ITEM_ORIGIN_DATA_MODEL_FOLDER = 2569;
// 原始数据点云列表
constexpr int32_t ITEM_ORIGIN_DATA_MODEL_LIST_FOLDER = 2570;
// 正射影像
constexpr int32_t ITEM_ORIGIN_DATA_DOM_FOLDER = 2571;
// 控制点
constexpr int32_t ITEM_ORIGIN_DATA_CONTROL_POINT_FOLDER = 2572;
constexpr int32_t ITEM_ORIGIN_DATA_CONTROL_POINT_LIST_FOLDER = 2573;

//////
constexpr int32_t ITEM_BUILDING_LIST_FOLDER = 2580;
constexpr int32_t ITEM_BUILDING_FOLDER = 2581;

constexpr int32_t ITEM_BUILDING_CONTOUR_FILER = 2582;
constexpr int32_t ITEM_BUILDING_CLOUD_FOLDER = 2583;

constexpr int32_t ITEM_BUILDING_BIRD_VIEW_FOLDER = 2584;
constexpr int32_t ITEM_BUILDING_FRONT_VIEW_FOLDER = 2585;
// 墙面
// constexpr int32_t ITEM_BUILDING_WALL_CLOUD_FOLDER = 2586;
// constexpr int32_t ITEM_BUILDING_WALL_CLOUD_LIST_FOLDER = 2587;

// 截面
// constexpr int32_t ITEM_BUILDING_SECTIONS_FOLDER = 2601;
// 水平截面
constexpr int32_t ITEM_BUILDING_SECTION_HORIZONTAL_LIST_FOLDER = 2610;
constexpr int32_t ITEM_BUILDING_SECTION_HORIZONTAL_FOLDER = 2611;

constexpr int32_t ITEM_BUILDING_SECTION_HORIZONTAL_PLANE_FILER = 2612;
constexpr int32_t ITEM_BUILDING_SECTION_HORIZONTAL_POSE3D_FILER = 2613;
constexpr int32_t ITEM_BUILDING_SECTION_HORIZONTAL_SECTION_FILER = 2614;
constexpr int32_t ITEM_BUILDING_SECTION_HORIZONTAL_LAYER_FILER = 2615;
constexpr int32_t ITEM_BUILDING_SECTION_HORIZONTAL_CLOUD_FOLDER = 2616;
constexpr int32_t ITEM_BUILDING_SECTION_HORIZONTAL_IMAGE_FOLDER = 2617;

// 垂直截面
constexpr int32_t ITEM_BUILDING_SECTION_VERTICAL_LIST_FOLDER = 2620;
constexpr int32_t ITEM_BUILDING_SECTION_VERTICAL_FOLDER = 2621;

constexpr int32_t ITEM_BUILDING_SECTION_VERTICAL_PLANE_FILER = 2622;
constexpr int32_t ITEM_BUILDING_SECTION_VERTICAL_POSE3D_FILER = 2623;
constexpr int32_t ITEM_BUILDING_SECTION_VERTICAL_SECTION_FILER = 2624;
constexpr int32_t ITEM_BUILDING_SECTION_VERTICAL_LAYER_FILER = 2625;
constexpr int32_t ITEM_BUILDING_SECTION_VERTICAL_CLOUD_FOLDER = 2626;
constexpr int32_t ITEM_BUILDING_SECTION_VERTICAL_IMAGE_FOLDER = 2627;

// 地形
constexpr int32_t ITEM_TERRAIN_FEATURE_LIST_FOLDER = 2641;

constexpr int32_t ITEM_TERRAIN_LAYER_FILER = 2642;
constexpr int32_t ITEM_TERRAIN_LAYER_LIST_FOLDER = 2643;
// constexpr int32_t ITEM_TERRAIN_CONTROL_POINT_FOLDER = 2642;
// constexpr int32_t ITEM_TERRAIN_CONTROL_POINT_LIST_FOLDER = 2643;

// constexpr int32_t ITEM_TERRAIN_ROAD_EDGE_FILER = 2644;
// constexpr int32_t ITEM_TERRAIN_ROAD_EDGE_LIST_FOLDER = 2645;


// constexpr int32_t ITEM_FOLDER_END = 2999;

//
using Filer = common::file_system::Filer;
using Folder = common::file_system::Folder;

using Pose3dFiler = common::file_system::Pose3dFiler;
using BoxFiler = common::file_system::BoxFiler;
using PlaneFiler = common::file_system::PlaneFiler;

using Polyline2dFiler = common::file_system::Polyline2dFiler;
using Polygon2dFiler = common::file_system::Polygon2dFiler;

template<typename PointT>
using PointCloudFiler = common::file_system::PointCloudFiler<PointT>;
}
