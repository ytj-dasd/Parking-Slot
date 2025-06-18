#pragma once
#include <opencv2/core.hpp>
#include "bamboo/base/macro.h"
namespace welkin::bamboo {
// 地面箭头匹配
void BAMBOO_EXPORT MatchRoadMarking(const cv::Mat& image, const std::vector<cv::Mat>& models, 
    int& match_index, float& match_angle, float& max_score, float angle_step = 1.0);
}