#include "road_arrow.h"
#include <opencv2/imgproc.hpp>
#include "image_algorithm.h"
namespace welkin::bamboo {
void MatchRoadMarking(const cv::Mat& image, const std::vector<cv::Mat>& models, 
        int& match_index, float& match_angle, float& max_score, float angle_step) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
        match_index = -1;
        match_angle = 0.0;
        max_score = 0.0;
        return;
    }
    int best_index = -1; float best_angle = 0.0; float best_score = 0.0;

    cv::Rect brect = cv::boundingRect(cv::Mat(contours[0]));
    cv::Mat image_arrow = image(brect);

    cv::Mat result(1, 1, CV_32FC1);
    cv::Mat image_resize;
    for (int i = 0; i < models.size(); ++i) {
        auto& image_model = models[i];
        cv::resize(image_arrow, image_resize, image_model.size());
        cv::matchTemplate(image_resize, image_model, result, cv::TM_CCORR_NORMED);
        if (best_score < result.at<float>(0, 0)) {
            best_score = result.at<float>(0, 0);
            best_index = i;
            best_angle = 0.0;
        }
    }
    cv::Mat image_rotate, image_rotate_roi;
    cv::Point2f center(image_arrow.cols * 0.5, image_arrow.rows * 0.5);
    for (float angle = angle_step; angle < 360.0; angle += angle_step) {
        Rotate(image_arrow, image_rotate, center, angle);
        cv::findContours(image_rotate, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        brect = cv::boundingRect(cv::Mat(contours[0]));
        image_rotate_roi = image_rotate(brect);
        // cv::imwrite("/home/scene/data/test/" + std::to_string(angle) + ".png", image_rotate_roi);
        // 保证合适的宽高比
        if (image_rotate_roi.rows < image_rotate_roi.cols * 1.8) {continue;}
        for (int i = 0; i < models.size(); ++i) {
            auto& image_model = models[i];
            cv::resize(image_rotate_roi, image_resize, image_model.size());
            cv::matchTemplate(image_resize, image_model, result, cv::TM_CCORR_NORMED);
            if (best_score < result.at<float>(0, 0)) {
                best_score = result.at<float>(0, 0);
                best_index = i;
                best_angle = angle;
            }
        }
    }
    if (best_index == -1) {
        match_index = -1;
        match_angle = 0.0;
        max_score = 0.0;
    } else {
        match_index = best_index;
        match_angle = best_angle;
        max_score = best_score;
    }
}
}