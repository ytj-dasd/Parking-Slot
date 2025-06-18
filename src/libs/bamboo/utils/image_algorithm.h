#pragma once
#include <opencv2/core.hpp>
#include "bamboo/base/macro.h"
namespace welkin::bamboo {
// 图片旋转(超出范围不裁减)
// 返回值 - 旋转矩阵
cv::Mat BAMBOO_EXPORT Rotate(const cv::Mat& src, cv::Mat& dst, const cv::Point2f& center, float angle);
}