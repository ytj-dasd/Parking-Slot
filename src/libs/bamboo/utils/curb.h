// 低路沿相关算法
#pragma once
#include "bamboo/base/types.h"
#include "bamboo/base/macro.h"

namespace welkin::bamboo {
void BAMBOO_EXPORT FilterCurbCloud(
        const PointCloudPtr cloud_in, PointCloudPtr& cloud_out,
        int min_cell_nums = 3, int max_cell_nums = 100);

void BAMBOO_EXPORT FitCurbWithCloud(
        const PointCloudPtr cloud_in,
        std::vector<common::Point2d>& point_list,
        double max_dist, double sac_dist);
}