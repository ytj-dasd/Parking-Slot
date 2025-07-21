#include "ocr_detector.h"
#include <iostream>
#include <filesystem>

namespace welkin::bamboo {
OCRDetector::OCRDetector(std::string onnx_path)
{
    // std::cout << "Load OCRDetector Model" << std::endl;
    this->modle = std::unique_ptr<OCRModel>(new OCRModel(onnx_path, USE_DEVICE::CPU));
}

// 析构函数实现
OCRDetector::~OCRDetector()
{
}

std::vector<cv::Point2f> OCRDetector::orderPointsClockwise(const std::vector<cv::Point2f>& pts) {
    std::vector<cv::Point2f> sorted_pts = pts;

    std::sort(sorted_pts.begin(), sorted_pts.end(), [](const cv::Point2f& a, const cv::Point2f& b) -> bool{
        return a.x < b.x;
    });

    std::vector<cv::Point2f> leftMost = { sorted_pts[0], sorted_pts[1] };
    std::vector<cv::Point2f> rightMost = { sorted_pts[2], sorted_pts[3] };

    std::sort(leftMost.begin(), leftMost.end(), [](const cv::Point2f& a, const cv::Point2f& b) -> bool{
        return a.y < b.y;
    });
    cv::Point2f tl = leftMost[0]; 
    cv::Point2f bl = leftMost[1];

    std::sort(rightMost.begin(), rightMost.end(), [](const cv::Point2f& a, const cv::Point2f& b) -> bool{
        return a.y < b.y;
    });
    cv::Point2f tr = rightMost[0];
    cv::Point2f br = rightMost[1];

    std::vector<cv::Point2f> rect = { tl, tr, br, bl };
    return rect;
}

std::vector<cv::Point2f> OCRDetector::clipDetRes(const std::vector<cv::Point2f>& points, int img_height, int img_width) {
    std::vector<cv::Point2f> clipped_points = points;

    for(auto& p : clipped_points){
        p.x = std::min(std::max(p.x, 0.0f), static_cast<float>(img_width - 1));
        p.y = std::min(std::max(p.y, 0.0f), static_cast<float>(img_height - 1));
    }

    return clipped_points;
}

std::vector<DetectionBox> OCRDetector::filterTagDetRes(const std::vector<DetectionBox>& dt_boxes, const cv::Size& image_shape, int min_box_size) {
    int img_height = image_shape.height;
    int img_width = image_shape.width;
    std::vector<DetectionBox> dt_boxes_new;

    for(const auto& box_score_pair : dt_boxes){
        std::vector<cv::Point2f> box = box_score_pair.points;
        float score = box_score_pair.score;

        std::vector<cv::Point2f> ordered_box = orderPointsClockwise(box);

        std::vector<cv::Point2f> clipped_box = clipDetRes(ordered_box, img_height, img_width);

        float rect_width = cv::norm(clipped_box[0] - clipped_box[1]);
        float rect_height = cv::norm(clipped_box[0] - clipped_box[3]);

        if(std::min(rect_width, rect_height) < min_box_size){
            continue; 
        }

        DetectionBox det_box;
        det_box.points = clipped_box;
        det_box.score = score;
        dt_boxes_new.push_back(det_box);
    }

    // std::cout << "过滤后检测到 " << dt_boxes_new.size() << " 个文本区域。" << std::endl;
    return dt_boxes_new;
}

void OCRDetector::sortedBoxes(std::vector<DetectionBox>& boxes){
    std::sort(boxes.begin(), boxes.end(),
        [](const DetectionBox& a, const DetectionBox& b) -> bool{
            if(a.points[0].y < b.points[0].y)
                return true;
            if(a.points[0].y > b.points[0].y)
                return false;
            return a.points[0].x < b.points[0].x;
        });
}

cv::Mat OCRDetector::getRotateCropImage(const cv::Mat& img, const DetectionBox& box) {
    if(box.points.size() != 4){
        std::cerr << "Error: shape of points must be 4*2" << std::endl;
        return cv::Mat();
    }

    float width1 = cv::norm(box.points[0] - box.points[1]);
    float width2 = cv::norm(box.points[2] - box.points[3]);
    int img_crop_width = static_cast<int>(std::max(width1, width2));

    float height1 = cv::norm(box.points[0] - box.points[3]);
    float height2 = cv::norm(box.points[1] - box.points[2]);
    int img_crop_height = static_cast<int>(std::max(height1, height2));

    std::vector<cv::Point2f> pts_std = {
        cv::Point2f(0, 0),
        cv::Point2f(static_cast<float>(img_crop_width), 0),
        cv::Point2f(static_cast<float>(img_crop_width), static_cast<float>(img_crop_height)),
        cv::Point2f(0, static_cast<float>(img_crop_height))
    };

    std::vector<cv::Point2f> pts_src = box.points;

    cv::Mat M = cv::getPerspectiveTransform(pts_src, pts_std);

    cv::Mat dst_img;
    cv::warpPerspective(img, dst_img, M, cv::Size(img_crop_width, img_crop_height),
                       cv::INTER_CUBIC, cv::BORDER_REPLICATE);

    dst_img.convertTo(dst_img, CV_32FC3);
    cv::min(dst_img, 255.0, dst_img); 

    float aspect_ratio = static_cast<float>(dst_img.rows) / static_cast<float>(dst_img.cols);
    if(aspect_ratio >= 1.5){
        cv::rotate(dst_img, dst_img, cv::ROTATE_90_CLOCKWISE);
    }

    return dst_img;
}

// 后处理
std::vector<DetectionBox> OCRDetector::postProcess(const cv::Mat& pred, 
                                               float bin_thresh, 
                                               float box_thresh, 
                                               int max_candidates, 
                                               float unclip_ratio, 
                                               int min_size, 
                                               bool use_dilation) {
    cv::Mat binary;
    cv::threshold(pred, binary, bin_thresh, 255, cv::THRESH_BINARY);
    binary.convertTo(binary, CV_8UC1);

    if (use_dilation) {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
        cv::dilate(binary, binary, kernel);
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    std::vector<DetectionBox> boxes;
    for (size_t i = 0; i < contours.size() && boxes.size() < max_candidates; ++i) {
        const auto& contour = contours[i];
        if (contour.size() < 4) continue;

        cv::RotatedRect rect = cv::minAreaRect(contour);
        cv::Point2f rect_points[4];
        rect.points(rect_points); 

        std::vector<cv::Point2f> box = {
            rect_points[0],
            rect_points[1],
            rect_points[2],
            rect_points[3]
        };

        float min_side = std::min(rect.size.width, rect.size.height);
        if (min_side < min_size) continue;

        float score = boxScoreFast(pred, box);
        if (score < box_thresh) continue;

        std::vector<cv::Point2f> expanded_box;
        // float scale = unclip_ratio;
        cv::Point2f center(0, 0);
        for (const auto& p : box) {
            center += p;
        }
        center *= (1.0 / box.size());

        float x_min = std::numeric_limits<float>::max();
        float x_max = std::numeric_limits<float>::lowest();
        float y_min = std::numeric_limits<float>::max();
        float y_max = std::numeric_limits<float>::lowest();

        for (const auto& p : box) {
            if (p.x < x_min) x_min = p.x;
            if (p.x > x_max) x_max = p.x;
            if (p.y < y_min) y_min = p.y;
            if (p.y > y_max) y_max = p.y;
        }

        float x_range = x_max - x_min;
        float y_range = y_max - y_min;
        bool is_wider = x_range > y_range;
        float scale_long = 1.2f; 
        float scale_short = 3.0f; 

        expanded_box.reserve(4);
        for (const auto& p : box) {
            cv::Point2f direction = p - center;
            float dx = direction.x;
            float dy = direction.y;
            if (is_wider) {
                dy *= scale_short;
                dx *= scale_long;
            }
            else {
                dx *= scale_short;
                dy *= scale_long;
            }
            expanded_box.emplace_back(center + cv::Point2f(dx, dy));
        }

        // for (const auto& p : box) {
        //     cv::Point2f direction = p - center;
        //     expanded_box.emplace_back(p + direction * (scale - 1.0));
        // }

        float expanded_min_side = std::min(
            cv::norm(expanded_box[0] - expanded_box[1]),
            cv::norm(expanded_box[1] - expanded_box[2])
        );
        if (expanded_min_side < (min_size + 2)) continue;

        for (auto& p : expanded_box) {
            p.x = std::min(std::max(p.x, 0.0f), static_cast<float>(pred.cols - 1));
            p.y = std::min(std::max(p.y, 0.0f), static_cast<float>(pred.rows - 1));
        }

        box = orderPointsClockwise(expanded_box);

        DetectionBox det_box;
        det_box.points = box;
        det_box.score = score;

        boxes.emplace_back(det_box);
    }

    return boxes;
}

// 计算检测框得分（平均值）
float OCRDetector::boxScoreFast(const cv::Mat& pred, const std::vector<cv::Point2f>& box) {
    cv::Rect bbox = cv::boundingRect(box);
    bbox &= cv::Rect(0, 0, pred.cols, pred.rows); 

    if (bbox.area() == 0) return 0.0f;

    cv::Mat mask = cv::Mat::zeros(bbox.height, bbox.width, CV_8UC1);
    std::vector<cv::Point> box_int;
    for (const auto& p : box) {
        box_int.emplace_back(cv::Point(static_cast<int>(p.x - bbox.x), static_cast<int>(p.y - bbox.y)));
    }
    cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{box_int}, cv::Scalar(1));

    cv::Mat roi = pred(bbox);
    double mean_val = cv::mean(roi, mask)[0];
    return static_cast<float>(mean_val);
}

cv::Mat OCRDetector::resizeWithAspectRatio(const cv::Mat& src, const cv::Size& target_size, cv::Mat& padded_image) {
    int original_width = src.cols;
    int original_height = src.rows;
    float aspect_ratio = static_cast<float>(original_width) / original_height;
    float target_aspect_ratio = static_cast<float>(target_size.width) / target_size.height;

    cv::Mat resized;
    if(aspect_ratio > target_aspect_ratio){
        cv::resize(src, resized, cv::Size(target_size.width, static_cast<int>(target_size.width / aspect_ratio)));
        int top = (target_size.height - resized.rows) / 2;
        int bottom = target_size.height - resized.rows - top;
        cv::copyMakeBorder(resized, padded_image, top, bottom, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));
    }
    else{
        cv::resize(src, resized, cv::Size(static_cast<int>(target_size.height * aspect_ratio), target_size.height));
        int left = (target_size.width - resized.cols) / 2;
        int right = target_size.width - resized.cols - left;
        cv::copyMakeBorder(resized, padded_image, 0, 0, left, right, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));
    }
    return padded_image;
}



std::vector<std::vector<DetectionBox>> OCRDetector::detect(const std::string& path){
    cv::Mat inimg = cv::imread(path, cv::IMREAD_COLOR);
    std::vector<cv::Mat> vmat;
    if (!inimg.empty()) {
        vmat.push_back(inimg);
    } else {
        std::cerr << "Failed to read image: " << path << std::endl;
    }
    return this->detect(vmat);
}

// 检测函数，批量路径
std::vector<std::vector<DetectionBox>> OCRDetector::detect(const std::vector<cv::Mat>& imgs){
    cv::Size resized_shape(768, 768);
    bool preserve_aspect_ratio = true;
    float box_score_thresh = 0.6f;
    int min_box_size = 3;
    std::vector<std::vector<DetectionBox>> out;
    for (const auto& img : imgs){
        // img = self._preprocess_images(img) 
        out.push_back(this->detectOne(
            img, 
            resized_shape, 
            preserve_aspect_ratio, 
            box_score_thresh, 
            min_box_size));
    }
    return out;
}

// 检测单个图像
std::vector<DetectionBox> OCRDetector::detectOne(const cv::Mat& img, const cv::Size& resized_shape, bool preserve_aspect_ratio, float box_score_thresh, int min_box_size){
    // cv::Mat imgresize;
    // cv::resize(img, imgresize, resized_shape);
    cv::Mat padded_image;
    cv::Mat img_resized = resizeWithAspectRatio(img, resized_shape, padded_image);

    long long input_height = img_resized.rows;
    long long input_width = img_resized.cols;
    runreturn ret_data = this->modle->runDetect(
        input_height, 
        input_width, 
        input_height * input_width * 3, 
        img_resized.data
    );

    // int64_t length = ret_data.shape[0];
    // int64_t width = ret_data.shape[1];
    int64_t height_out = ret_data.shape[2];
    int64_t width_out = ret_data.shape[3];
    cv::Mat ncdata(height_out, width_out, CV_32FC1, static_cast<float*>(ret_data.data));

    float bin_thresh = 0.3f;
    float box_thresh = box_score_thresh;
    float unclip_ratio = 2.0f; 
    int max_candidates = 1000;
    bool use_dilation = false;

    std::vector<DetectionBox> boxes = postProcess(ncdata, bin_thresh, box_thresh, max_candidates, unclip_ratio, min_box_size, use_dilation);

    std::vector<DetectionBox> filtered_boxes = filterTagDetRes(boxes, img_resized.size());

    sortedBoxes(filtered_boxes);

    // std::string save_dir = "cropped_images";
    // if(!std::filesystem::exists(save_dir)){
    //     std::filesystem::create_directory(save_dir);
    // }
    std::vector<DetectionBox> detected_results;
    // int crop_count = 0;
    for(auto& box : filtered_boxes){
        cv::Mat img_crop = getRotateCropImage(img_resized, box);
        
        if(img_crop.channels() == 3){
            cv::cvtColor(img_crop, img_crop, cv::COLOR_BGR2RGB);
        } else if(img_crop.channels() == 1){
            cv::cvtColor(img_crop, img_crop, cv::COLOR_GRAY2RGB);
        }

        img_crop.convertTo(img_crop, CV_8UC3);

        box.cropped_img = img_crop;
        // std::string filename = save_dir + "/image_crop_" + std::to_string(crop_count) + ".png";
        // bool success = cv::imwrite(filename, img_crop);

        detected_results.push_back(box);
        // crop_count++;
    }

    return detected_results;
}
}
