#pragma once
#include <bamboo/base/types.h>
#include <common/geometry/point2.h>
namespace welkin::bamboo {
// 多段线拟合
void PiecewiseLineFit(const PointCloud::Ptr& cloud, std::vector<common::Point2d>& points);
}