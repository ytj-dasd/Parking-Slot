#pragma once
#include "bamboo/base/types.h"
#include "bamboo/base/macro.h"
namespace welkin::bamboo {
void BAMBOO_EXPORT FilterYellowLaneCloud(
    const PointCloudPtr cloud_in, PointCloudPtr& cloud_out);
void BAMBOO_EXPORT FilterWhiteLaneCloud(
    const PointCloudPtr cloud_in, PointCloudPtr& cloud_out);
}