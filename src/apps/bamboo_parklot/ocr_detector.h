#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>
#include "ocr_model.h"

namespace welkin::bamboo {
struct DetectionBox {
    std::vector<cv::Point2f> points; 
    float score;                      
    cv::Mat cropped_img;
};

class OCRDetector
{
private:
    std::unique_ptr<OCRModel> modle; 
    std::vector<DetectionBox> detectOne(const cv::Mat& img, const cv::Size& resized_shape,
                       bool preserve_aspect_ratio=true, float box_score_thresh=0.3,
                       int min_box_size=4);
    std::vector<DetectionBox> detected_boxes;

    std::vector<cv::Point2f> orderPointsClockwise(const std::vector<cv::Point2f>& pts);
    std::vector<cv::Point2f> clipDetRes(const std::vector<cv::Point2f>& points, int img_height, int img_width);
    std::vector<DetectionBox> filterTagDetRes(const std::vector<DetectionBox>& dt_boxes, const cv::Size& image_shape, int min_box_size=4);
    void sortedBoxes(std::vector<DetectionBox>& boxes);
    cv::Mat getRotateCropImage(const cv::Mat& img, const DetectionBox& box);
    std::vector<DetectionBox> postProcess(const cv::Mat& pred, float bin_thresh, 
                                                  float box_thresh, int max_candidates, 
                                                  float unclip_ratio, int min_size, 
                                                  bool use_dilation = false);
    float boxScoreFast(const cv::Mat& pred, const std::vector<cv::Point2f>& box);
    cv::Mat resizeWithAspectRatio(const cv::Mat& src, const cv::Size& target_size, cv::Mat& padded_image);
public:
    OCRDetector(std::string onnx_path);
    std::vector<std::vector<DetectionBox>> detect(const std::string& path);
    std::vector<std::vector<DetectionBox>> detect(const std::vector<cv::Mat>& imgs);
    ~OCRDetector();
};

}