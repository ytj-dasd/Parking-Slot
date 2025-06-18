#pragma once
#include "bamboo/base/macro.h"
#include "bamboo/base/types.h"

namespace welkin::bamboo {
// 停车位分割线拟合
bool BAMBOO_EXPORT FitParkLotLine(
        const PointCloud::Ptr& cloud_in, const common::Line2d& init_line,
        common::Line2d& line1, common::Line2d& line2);
bool BAMBOO_EXPORT FitParkLot(
        const PointCloud::Ptr& cloud_in, const common::Polygon2d& border,
        std::vector<common::Polygon2d>& lots);
}