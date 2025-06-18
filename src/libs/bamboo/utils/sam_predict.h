#pragma once
#include <opencv2/core.hpp>
#include "bamboo/file_system/origin_data_folder.h"
#include "bamboo/base/macro.h"

namespace welkin::bamboo {
bool BAMBOO_EXPORT PromptPredict(OriginDataDOMFolder* dom_folder, 
    const common::Point2d& prompt_pt, cv::Mat& mask, common::Rectd& rect);

struct BAMBOO_EXPORT RectMask {
    cv::Mat mask;
    common::Rectd rect;
    bool computeContours(std::vector<common::Point2d>& points);
};
bool BAMBOO_EXPORT PromptPredict(OriginDataDOMFolder* dom_folder, 
    const std::vector<common::Point2d>& prompt_pts, std::vector<RectMask>& rect_masks);
}