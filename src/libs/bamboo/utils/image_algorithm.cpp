#include "image_algorithm.h"
#include <opencv2/imgproc.hpp>
namespace welkin::bamboo {
cv::Mat Rotate(const cv::Mat& src, cv::Mat& dst, const cv::Point2f& center, float angle) {
    double scale = 1.0;
    cv::Mat rotate = cv::getRotationMatrix2D(center, angle, scale);
    auto size = cv::Size(scale * src.cols, scale * src.rows);
    cv::Rect bbox = cv::RotatedRect(center, size, angle).boundingRect();
    rotate.at<double>(0, 2) += bbox.width * 0.5 - center.x;
    rotate.at<double>(1, 2) += bbox.height * 0.5 - center.y;
    cv::warpAffine(src, dst, rotate, bbox.size());
    return rotate;
}
}