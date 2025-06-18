#pragma once
#include "common/point_cloud/filter.h"
#include "common/point_cloud/projector.h"
#include "common/point_cloud/segmentation.h"
namespace welkin::bamboo {

using PointX = common::point_cloud::PointXYZRGBI;
using PointCloud = pcl::PointCloud<PointX>;
using PointCloudPtr = PointCloud::Ptr;

//
using ProjectType = common::point_cloud::ProjectType;

}