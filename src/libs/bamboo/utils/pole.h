#pragma once
// 杆状物算法
#include <common/geometry/circle.h>
#include "bamboo/base/types.h"
#include "bamboo/base/macro.h"

namespace welkin::bamboo {
// 获取地面高度
bool BAMBOO_EXPORT GetGroundHeight(
    const PointCloudPtr cloud_in, double& height,
    int min_point_nums = 100, double res_z = 0.05);
// 杆状物点云过滤
void BAMBOO_EXPORT FilterPoleCloud(
        const PointCloudPtr cloud_in, PointCloudPtr& cloud_out,
        int min_cell_nums = 10, int max_cell_nums = 100,
        double res_x = 0.05, double res_y = 0.05, double res_z = 0.05);

// 杆状物点云拟合
// scale - (x, y)放大缩小多少倍
// radius_limits - 半径限制
bool BAMBOO_EXPORT FitPoleCloud(
        const PointCloudPtr cloud_in, common::Circled& circle,
        const common::Ranged& radius_limits, double scale = 1.0,
        double min_inlier_ratio = 0.5, double sac_dist = 0.01, int max_iter = 200);
}