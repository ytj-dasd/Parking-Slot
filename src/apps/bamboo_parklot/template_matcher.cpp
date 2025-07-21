#include "template_matcher.h"
#include "ocr_detector.h"
#include "ocr_recognizer.h"
#include "data_preprocessor.h"
#include "file_system.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <numeric>
#include <omp.h>
#include <chrono>
#include <common/base/dir.h>
#include <uface/uface.hpp>

namespace welkin::bamboo {
float MIN_X = -40.0f; float MAX_Y = 60.0f; // 小昆山
// float MIN_X = -30.0f; float MAX_Y = 40.0f; // 九里亭
// float MIN_X = -34.0f; float MAX_Y = 53.0f; // 金浩园_2
// float MIN_X = -50.0f; float MAX_Y = 100.0f; // 金浩园_3
// float MIN_X = -40.0f; float MAX_Y = 60.0f; // 金浩园_4
float PIXEL_SIZE = 0.02f;
TemplateMatcher::TemplateMatcher(ParklotFileSystem* fs) : _file_system(fs) {
    Q_ASSERT(fs);
}
TemplateMatcher::~TemplateMatcher() {}

void TemplateMatcher::setDetectOnnxModelPath(const std::string& model_path) {
    _ocr_detect_onnx_model_path = model_path;
}

void TemplateMatcher::setRecognizeOnnxModelPath(const std::string& model_path) {
    _ocr_recognize_onnx_model_path = model_path;
}

void TemplateMatcher::setDictPath(const std::string& dict_path) {
    _dict_path = dict_path;
}

std::vector<PointPair> TemplateMatcher::loadPairsFile(const std::string &filePath) {
    std::vector<PointPair> pairs;
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filePath << std::endl;
        return pairs;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        float x1, y1, x2 = -1, y2 = -1; 
        if (!(iss >> x1 >> y1)) {
            std::cerr << "Invalid line (less than 2 values): " << line << std::endl;
            continue; 
        }
        if (!(iss >> x2 >> y2)) {}
        pairs.emplace_back(PointPair{cv::Point2f(x1, y1), cv::Point2f(x2, y2)});
    }
    file.close();
    return pairs;
}

void TemplateMatcher::loadTemplateFile(const std::string &filePath, std::vector<PSlot>& parkslots, 
                                        const cv::Mat &mapImage, const cv::Mat &intensityMap) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filePath << std::endl;
        return;
    }

    std::string rgbDir = _file_system->getRgbTemplatesDir();
    std::string intensityDir = _file_system->getIntensityTemplatesDir();

    // 创建输出文件夹
    std::filesystem::create_directories(rgbDir);
    std::filesystem::create_directories(intensityDir);

    PSlot slot;
    float x, y;
    std::string line;
    int slot_index = 0;

    while (std::getline(file, line)) {
        if (line.empty()) {
            if (!slot.corners.empty()) {
                parkslots.push_back(slot);

                cv::Mat rgbTemplate = rotateAndExtractTemplate(mapImage, slot);
                cv::Mat intensityTemplate = rotateAndExtractTemplate(intensityMap, slot);

                if (!rgbTemplate.empty() && !intensityTemplate.empty()) {
                    _rgb_template_rois[slot_index] = rgbTemplate;
                    _intensity_template_rois[slot_index] = intensityTemplate;

                    std::string rgbFilePath = rgbDir + "/template_" + std::to_string(slot_index) + ".png";
                    cv::imwrite(rgbFilePath, rgbTemplate);

                    std::string intensityFilePath = intensityDir + "/template_" + std::to_string(slot_index) + ".png";
                    cv::imwrite(intensityFilePath, intensityTemplate);
                } else {
                    std::cerr << "Warning: Slot " << slot_index << " ROI is invalid, skipping template extraction." << std::endl;
                }

                slot.corners.clear();
                slot.is_inferred = false; 
                ++slot_index;
            }
            continue;
        }

        std::istringstream iss(line);
        if (iss >> x >> y) {
            int pixel_x = static_cast<int>((x - MIN_X) / PIXEL_SIZE);
            int pixel_y = static_cast<int>((MAX_Y - y) / PIXEL_SIZE);
            slot.corners.push_back(cv::Point2f(pixel_x, pixel_y));
        }
        else if (line == "0") {
            slot.is_inferred = false;
        }
        else if (line == "1") {
            slot.is_inferred = true;
        }
    }

    if (!slot.corners.empty()) {
        parkslots.push_back(slot);

        cv::Mat rgbTemplate = rotateAndExtractTemplate(mapImage, slot);
        cv::Mat intensityTemplate = rotateAndExtractTemplate(intensityMap, slot);

        if (!rgbTemplate.empty() && !intensityTemplate.empty()) {
            _rgb_template_rois[slot_index] = rgbTemplate;
            _intensity_template_rois[slot_index] = intensityTemplate;

            std::string rgbFilePath = rgbDir + "/template_" + std::to_string(slot_index) + ".png";
            cv::imwrite(rgbFilePath, rgbTemplate);

            std::string intensityFilePath = intensityDir + "/template_" + std::to_string(slot_index) + ".png";
            cv::imwrite(intensityFilePath, intensityTemplate);
        } else {
            std::cerr << "Warning: Slot " << slot_index << " ROI is invalid, skipping template extraction." << std::endl;
        }
    }

    file.close();

    std::cout << "Templates saved to " << rgbDir << " and " << intensityDir << std::endl;
}



cv::Mat TemplateMatcher::rotateAndExtractTemplate(const cv::Mat &image, const PSlot &slot, int offset) {
    if (slot.corners.size() != 4) {
        std::cerr << "Error: Parking slot does not have exactly 4 corners." << std::endl;
        return cv::Mat();
    }

    float length_01 = cv::norm(slot.corners[0] - slot.corners[1]);
    float length_12 = cv::norm(slot.corners[1] - slot.corners[2]);

    cv::Point2f shortEdgeStart, shortEdgeEnd;
    if (length_01 < length_12) {
        shortEdgeStart = slot.corners[0];
        shortEdgeEnd = slot.corners[1];
    } else {
        shortEdgeStart = slot.corners[1];
        shortEdgeEnd = slot.corners[2];
    }

    float angle = std::atan2(shortEdgeEnd.y - shortEdgeStart.y, shortEdgeEnd.x - shortEdgeStart.x) * 180.0f / CV_PI;

    std::vector<int> x_coords, y_coords;
    for (const auto &corner : slot.corners) {
        x_coords.push_back(static_cast<int>(corner.x));
        y_coords.push_back(static_cast<int>(corner.y));
    }
    int center_x = std::accumulate(x_coords.begin(), x_coords.end(), 0) / 4;
    int center_y = std::accumulate(y_coords.begin(), y_coords.end(), 0) / 4;

    cv::Mat rotationMatrix = cv::getRotationMatrix2D(cv::Point(center_x, center_y), angle, 1.0);
    cv::Mat rotatedImage;
    cv::warpAffine(image, rotatedImage, rotationMatrix, image.size());

    std::vector<int> x_coords_rot, y_coords_rot;
    for (const auto &corner : slot.corners) {
        int x_rot = static_cast<int>(rotationMatrix.at<double>(0, 0) * corner.x +
                                     rotationMatrix.at<double>(0, 1) * corner.y +
                                     rotationMatrix.at<double>(0, 2));
        int y_rot = static_cast<int>(rotationMatrix.at<double>(1, 0) * corner.x +
                                     rotationMatrix.at<double>(1, 1) * corner.y +
                                     rotationMatrix.at<double>(1, 2));
        x_coords_rot.push_back(x_rot);
        y_coords_rot.push_back(y_rot);
    }

    int x1 = std::max(0, *std::min_element(x_coords_rot.begin(), x_coords_rot.end()) - OFFSET);
    int y1 = std::max(0, *std::min_element(y_coords_rot.begin(), y_coords_rot.end()) - OFFSET);
    int x2 = std::min(rotatedImage.cols, *std::max_element(x_coords_rot.begin(), x_coords_rot.end()) + OFFSET);
    int y2 = std::min(rotatedImage.rows, *std::max_element(y_coords_rot.begin(), y_coords_rot.end()) + OFFSET);

    
    if (x1 >= x2 || y1 >= y2 || x1 < 0 || y1 < 0 || x2 > rotatedImage.cols || y2 > rotatedImage.rows) {
        std::cerr << "Error: Invalid ROI dimensions. Skipping this template extraction." << std::endl;
        std::cout << "extractTemplate: " << x1 << " " << y1 << " " << x2 << " " << y2 << " " << rotatedImage.cols << " " << rotatedImage.rows << std::endl;
        return cv::Mat();  
    }

    return rotatedImage(cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2))).clone();
}


cv::Mat TemplateMatcher::extractROI(const cv::Mat &image, const cv::Point2f &center, const cv::Size &size) {
    int x1 = std::max(0, static_cast<int>(center.x - size.width / 2));
    int y1 = std::max(0, static_cast<int>(center.y - size.height / 2));
    int x2 = std::min(image.cols, static_cast<int>(center.x + size.width / 2));
    int y2 = std::min(image.rows, static_cast<int>(center.y + size.height / 2));

    // std::cout << "extractROI: " << x1 << " " << y1 << " " << x2 << " " << y2 << " " << image.cols << " " << image.rows << std::endl;

    return image(cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2))).clone();
}

cv::Mat TemplateMatcher::rotateAndExtractROI(const cv::Mat &image, const cv::Point2f &center, float angle, const cv::Size &size) {
    cv::Mat rotationMatrix = cv::getRotationMatrix2D(center, angle, 1.0);

    cv::Mat rotatedImage;
    cv::warpAffine(image, rotatedImage, rotationMatrix, image.size());

    return extractROI(rotatedImage, center, size);
}

MatchResult TemplateMatcher::matchTemplate(const cv::Mat &image, const cv::Mat &templateImage) {
    cv::Mat result;
    cv::matchTemplate(image, templateImage, result, cv::TM_CCOEFF_NORMED);

    double maxVal;
    cv::Point maxLoc;
    cv::minMaxLoc(result, nullptr, &maxVal, nullptr, &maxLoc);

    return {maxVal, maxLoc};
}

float TemplateMatcher::computeIoU(const std::vector<cv::Point2f>& slot1, const std::vector<cv::Point2f>& slot2) {
    std::vector<cv::Point> all_points(slot1.begin(), slot1.end());
    all_points.insert(all_points.end(), slot2.begin(), slot2.end());

    cv::Rect bounding_rect = cv::boundingRect(all_points);
    cv::Mat slot1_mask = cv::Mat::zeros(bounding_rect.size(), CV_8UC1);
    cv::Mat slot2_mask = cv::Mat::zeros(bounding_rect.size(), CV_8UC1);

    std::vector<cv::Point> adjusted_slot1, adjusted_slot2;
    for (const auto& pt : slot1) adjusted_slot1.push_back(cv::Point(pt.x - bounding_rect.x, pt.y - bounding_rect.y));
    for (const auto& pt : slot2) adjusted_slot2.push_back(cv::Point(pt.x - bounding_rect.x, pt.y - bounding_rect.y));

    cv::fillPoly(slot1_mask, std::vector<std::vector<cv::Point>>{adjusted_slot1}, cv::Scalar(255));
    cv::fillPoly(slot2_mask, std::vector<std::vector<cv::Point>>{adjusted_slot2}, cv::Scalar(255));

    int intersection = cv::countNonZero(slot1_mask & slot2_mask);
    int union_area = cv::countNonZero(slot1_mask | slot2_mask);

    return union_area > 0 ? static_cast<float>(intersection) / union_area : 0;
}

// 计算强度占用率
float TemplateMatcher::intensityOcc(const cv::Mat& intensity_map, const cv::Point& p1, const cv::Point& p2, int buffer) {
    cv::Mat mask = cv::Mat::zeros(intensity_map.size(), CV_8UC1);
    cv::line(mask, p1, p2, cv::Scalar(255), buffer);

    cv::Mat line_region;
    cv::bitwise_and(mask, intensity_map, line_region);
    
    int total_count = cv::countNonZero(mask);
    int occupied_count = cv::countNonZero(line_region);

    return total_count > 0 ? static_cast<float>(occupied_count) / total_count : 0;
}

// 计算加权平均占用率
float TemplateMatcher::weightedAvgOcc(const std::vector<std::pair<cv::Point, cv::Point>>& edges, const cv::Mat& intensity_map) {
    float total_length = 0.0f;
    float weighted_occ_sum = 0.0f;

    for (const auto& edge : edges) {
        float length = cv::norm(edge.first - edge.second);
        float occ = intensityOcc(intensity_map, edge.first, edge.second);

        total_length += length * length;
        weighted_occ_sum += length * length * occ;
    }

    return total_length > 0 ? weighted_occ_sum / total_length : 0;
}

// 判断一个点是否在停车位范围内
bool TemplateMatcher::isPointWithinSlot(const cv::Point2f &point, const std::vector<ParkSpaceGroup> &park_space_groups, int expansion) {
    for (const auto &group : park_space_groups) {
        for (const auto &slot : group.parkspaces) {
            std::vector<cv::Point2f> slot_corners;
            for (const auto &pt : slot.points) {
                slot_corners.emplace_back(cv::Point2f(static_cast<float>(pt.x), static_cast<float>(pt.y)));
            }
            cv::Rect bbox = cv::boundingRect(slot_corners);
            bbox.x -= expansion;
            bbox.y -= expansion;
            bbox.width += 2 * expansion; 
            bbox.height += 2 * expansion; 
            if (bbox.contains(point)) {
                return true;
            }
        }
    }
    return false;
}

void TemplateMatcher::generateGroup(std::vector<ParkSpaceGroup> &park_space_group_vec, 
                                        std::vector<ParkSpace> temp_slots, bool is_matched){
    while (!temp_slots.empty()) {
        if (park_space_group_vec.empty()) {
            ParkSpaceGroup new_group;
            new_group.addParkSpace(temp_slots.front());
            temp_slots.erase(temp_slots.begin());
            park_space_group_vec.push_back(std::move(new_group));
            continue;
        }

        QVector<int> valid_ids; 
        auto& current_group = park_space_group_vec.back(); 

        for (int i = 0; i < temp_slots.size(); ++i) {
            const auto& park_space = temp_slots.at(i);
            if (is_matched) {
                if (current_group.isIntersectImage(park_space, true)) { 
                    current_group.addParkSpace(park_space);
                    valid_ids.append(i); 
                }
            } else {
                if (current_group.isIntersectImage(park_space)) { 
                    current_group.addParkSpace(park_space);
                    valid_ids.append(i); 
                }
            }
        }

        if (valid_ids.empty()) {
            ParkSpaceGroup new_group;
            new_group.addParkSpace(temp_slots.front());
            temp_slots.erase(temp_slots.begin());
            park_space_group_vec.push_back(std::move(new_group));
            continue;
        }

        for (int i = valid_ids.size() - 1; i >= 0; --i) {
            temp_slots.erase(temp_slots.begin() + valid_ids.at(i));
        }
    }
}

void TemplateMatcher::drawGroupEdges(const ParkSpaceGroup &group, cv::Mat &mapImage,
                                         std::vector<common::Line2d> &all_lines, std::vector<bool> &is_adjacent,
                                         std::vector<std::vector<int>> &adjacency, std::vector<AdjacentLinePair> &adj_line_pairs) {
    all_lines.clear();
    adj_line_pairs.clear();
    adjacency.assign(group.parkspaces.size(), std::vector<int>());
    std::vector<int> line_psIndex;  
    std::vector<int> line_cornerA;  
    std::vector<int> line_cornerB;
     for (int psIdx = 0; psIdx < group.parkspaces.size(); psIdx++) {
        const auto &ps = group.parkspaces[psIdx];
        for (int edgeId = 0; edgeId < 4; edgeId++) {
            common::Line2d ln = ps.getLine(edgeId);
            all_lines.emplace_back(ln);

            line_psIndex.push_back(psIdx);
            line_cornerA.push_back(edgeId);
            line_cornerB.push_back((edgeId + 1) % 4);
        }
    }

    is_adjacent.assign(all_lines.size(), false);

    for (size_t i = 0; i < all_lines.size(); ++i) {
        for (size_t j = i + 1; j < all_lines.size(); ++j) {
            const common::Line2d& lineA = all_lines[i];
            const common::Line2d& lineB = all_lines[j];

            bool share_two_endpoints = 
                (lineA.begin_point.isEqualTo(lineB.begin_point) && lineA.end_point.isEqualTo(lineB.end_point)) ||
                (lineA.begin_point.isEqualTo(lineB.end_point) && lineA.end_point.isEqualTo(lineB.begin_point));

            common::Point2d directionA = lineA.getDirection();
            common::Point2d directionB = lineB.getDirection();
            double dot_val = std::abs(directionA.dot(directionB));
            dot_val = std::min(1.0, std::max(0.0, dot_val)); 
            double direction_diff = std::acos(dot_val) * 180.0 / CV_PI;

            common::Point2d midA = lineA.getCenter();
            common::Point2d midB = lineB.getCenter();
            double mid_distance = midA.distanceTo(midB);

            bool adjacent = false;
            if (share_two_endpoints) {
                adjacent = true;
            } else if (direction_diff < ANGLE_DIFF_THRESH && mid_distance < OVERLAP_THRESH) { 
                adjacent = true;
            }

            if (adjacent) {
                is_adjacent[i] = true;
                is_adjacent[j] = true;

                int ps1 = i / 4;
                int ps2 = j / 4;

                if (ps1 != ps2 && ps1 < group.parkspaces.size() && ps2 < group.parkspaces.size()) {
                    if (std::find(adjacency[ps1].begin(), adjacency[ps1].end(), ps2) == adjacency[ps1].end()) {
                        adjacency[ps1].push_back(ps2);
                    }
                    if (std::find(adjacency[ps2].begin(), adjacency[ps2].end(), ps1) == adjacency[ps2].end()) {
                        adjacency[ps2].push_back(ps1);
                    }
                }

                AdjacentLinePair pair;

                pair.psA = line_psIndex[i];
                pair.cornerA1 = line_cornerA[i];
                pair.cornerA2 = line_cornerB[i];

                pair.psB = line_psIndex[j];
                pair.cornerB1 = line_cornerA[j];
                pair.cornerB2 = line_cornerB[j];

                adj_line_pairs.push_back(pair);
            }
        }
    }

    for (size_t i = 0; i < all_lines.size(); ++i) {
        cv::Scalar color = is_adjacent[i] ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);
        cv::Point p1(static_cast<int>(all_lines[i].begin_point.x), static_cast<int>(all_lines[i].begin_point.y));
        cv::Point p2(static_cast<int>(all_lines[i].end_point.x),   static_cast<int>(all_lines[i].end_point.y));
        cv::line(mapImage, p1, p2, color, 2);
    }

    return;
}

// 根据车位的邻接边数计算宽高增量
void TemplateMatcher::calcWidthHeightAdjustments(const ParkSpace &parking_space,
    const std::vector<common::Line2d> &all_lines, const std::vector<bool> &is_adjacent, int &widthAdd, int &heightAdd) {
    common::Line2d edge0 = parking_space.getLine(0);
    common::Line2d edge1 = parking_space.getLine(1);
    common::Line2d edge2 = parking_space.getLine(2);
    common::Line2d edge3 = parking_space.getLine(3);

    float dist0 = edge0.getLength();
    float dist1 = edge1.getLength();

    bool edge0IsShort = (dist0 < dist1);

    auto findLineIndex = [&](const common::Line2d& line)->int {
        for(size_t i = 0; i < all_lines.size(); ++i) {
            const auto &L = all_lines[i];
            if((line.begin_point.isEqualTo(L.begin_point) && line.end_point.isEqualTo(L.end_point)) ||
               (line.begin_point.isEqualTo(L.end_point) && line.end_point.isEqualTo(L.begin_point))) {
                return (int)i;
            }
        }
        return -1;
    };

    int idx0 = findLineIndex(edge0);
    int idx1 = findLineIndex(edge1);
    int idx2 = findLineIndex(edge2);
    int idx3 = findLineIndex(edge3);

    widthAdd = 0; 
    heightAdd = 0;

    auto checkEdge = [&](int idx, bool isShortEdge){
        if(idx >= 0 && idx < (int)is_adjacent.size()) {
            if(is_adjacent[idx]) {
                if(isShortEdge) {
                    heightAdd += 4;
                } else {
                    widthAdd += 4;
                }
            }
        }
    };

    if(edge0IsShort) {
        checkEdge(idx0, true);
        checkEdge(idx2, true);
        checkEdge(idx1, false);
        checkEdge(idx3, false);
    } else {
        checkEdge(idx0, false);
        checkEdge(idx2, false);
        checkEdge(idx1, true);
        checkEdge(idx3, true);
    }
}

// 创建模板图像函数：给定宽高，仅在上下和左右各8像素画白线(255)，其余为0
cv::Mat TemplateMatcher::createTemplateImg(int width, int height) {
    cv::Mat tmpl = cv::Mat::zeros(height, width, CV_8UC1);

    for (int y = 0; y < 8 && y < height; ++y) {
        for (int x = 0; x < width; ++x) tmpl.at<uchar>(y, x) = 255;
    }
    for (int y = height - 8; y < height; ++y) {
        if(y < 0) continue;
        for (int x = 0; x < width; ++x) tmpl.at<uchar>(y, x) = 255;
    }

    for (int x = 0; x < 8 && x < width; ++x) {
        for (int y = 0; y < height; ++y) tmpl.at<uchar>(y, x) = 255;
    }
    for (int x = width - 8; x < width; ++x) {
        if(x < 0) continue;
        for (int y = 0; y < height; ++y) tmpl.at<uchar>(y, x) = 255;
    }

    return tmpl;
}

MatchResultInfo TemplateMatcher::matchAllTemplates(const cv::Mat &roi, int widthAdd, int heightAdd) {
    std::vector<cv::Size> baseSizes = {
        cv::Size(120,265),
        cv::Size(125,300),
        cv::Size(110,225),
        cv::Size(110,215)
    };

    MatchResultInfo best{0.0, cv::Size(), cv::Point()};
    cv::Mat gray;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    cv::Mat otsu;
    // cv::threshold(roi, otsu, 128, 255, cv::THRESH_OTSU | cv::THRESH_BINARY);
    cv::threshold(gray, otsu, 128, 255, cv::THRESH_BINARY);

    for (auto &bs : baseSizes) {
        int tmplW = bs.width + widthAdd;
        int tmplH = bs.height + heightAdd;
        if (tmplW <= 0 || tmplH <= 0) continue;

        cv::Mat tmpl = createTemplateImg(tmplW, tmplH);
        if (otsu.cols < tmplW || otsu.rows < tmplH) continue;

        cv::Mat result;
        cv::matchTemplate(otsu, tmpl, result, cv::TM_CCOEFF_NORMED);

        double maxVal; 
        cv::Point maxLoc;
        cv::minMaxLoc(result, nullptr, &maxVal, nullptr, &maxLoc);

        if (maxVal > best.score) {
            best.score = maxVal;
            best.size = cv::Size(tmplW, tmplH);
            best.location = maxLoc;
        }
    }

    return best;
}

// 去除文本首位的零
std::string TemplateMatcher::remove_leading_zeros(const std::string& text) {
    size_t first_non_zero = text.find_first_not_of('0');
    if (first_non_zero == std::string::npos) {
        return "0";
    }
    return text.substr(first_non_zero);
}

// 编号排列
void TemplateMatcher::backtrack_sequences(int current_idx,
                         int current_num,
                         const std::vector<std::vector<int>>& adjacency,
                         std::vector<int>& cur_seq,
                         std::vector<std::vector<int>>& all_seqs)
{
    cur_seq[current_idx] = current_num;

    bool complete = true;
    for (int val : cur_seq) {
        if (val == 0) { 
            complete = false; 
            break;
        }
    }
    if (complete) {
        all_seqs.push_back(cur_seq);
        return;
    }

    for (int neighbor : adjacency[current_idx]) {
        if (cur_seq[neighbor] == 0) {
            std::vector<int> new_seq = cur_seq;
            new_seq[neighbor] = current_num + 1;
            backtrack_sequences(neighbor, current_num + 1, adjacency, new_seq, all_seqs);
        }
    }
}

// 纠正库位编号
bool TemplateMatcher::analyzeAndCorrectGroupNumbers(const std::vector<int>& inOcrNumbers,    
        std::vector<int>& outCorrectedNumbers, const std::vector<std::vector<int>>& adjacency){
    int n = (int)inOcrNumbers.size();
    outCorrectedNumbers.resize(n, -1);

    std::vector<std::vector<int>> all_sequences;
    for(int start = 0; start < n; start++) {
        std::vector<int> seq(n, 0);
        backtrack_sequences(start, 1, adjacency, seq, all_sequences);
    }
    if(all_sequences.empty()) {
        return false;
    }

    struct SeqScore {
        std::vector<int> seq;
        int score;
        std::vector<int> offsets;
    };
    std::vector<SeqScore> scored; 
    scored.reserve(all_sequences.size());

    for(const auto& seq : all_sequences) {
        int score = 0;
        std::vector<int> matched_offsets;
        for(int i = 0; i < n; i++) {
            if(inOcrNumbers[i] < 0) continue; 
            for(int j = 0; j < n; j++) {
                if(i == j || inOcrNumbers[j] < 0) continue; 
                int seq_diff = seq[j] - seq[i];
                int ocr_diff = inOcrNumbers[j] - inOcrNumbers[i];

                if(seq_diff == ocr_diff) {
                    score++;
                    int offset = inOcrNumbers[i] - seq[i];
                    matched_offsets.push_back(offset);
                }
                
            }
        }
        scored.push_back({seq, score, matched_offsets });
    }

    auto bestIt = std::max_element(scored.begin(), scored.end(), 
        [](const SeqScore& a, const SeqScore& b){
            return a.score < b.score;
        }
    );
    if(bestIt == scored.end() || bestIt->score == 0) {
        return false;
    }

    std::vector<int> matched_offsets = bestIt->offsets;
    std::map<int, int> offset_counts;
    for(auto offset : matched_offsets) {
        offset_counts[offset]++;
    }

    int most_common_offset = 0;
    int max_count = 0;
    for(const auto& [offset, count] : offset_counts) {
        if(count > max_count) {
            max_count = count;
            most_common_offset = offset;
        }
    }

    std::vector<int> adjusted_seq = bestIt->seq;
    for(int i = 0; i < n; i++) {
        adjusted_seq[i] += most_common_offset;
    }

    outCorrectedNumbers = adjusted_seq;
    return true;
}

// 校正函数
void TemplateMatcher::CorrectParkingGroup(ParkSpaceGroup& group, 
                                              const common::Line2d& adjacentEdge, 
                                              float angle_threshold, 
                                              float delta_threshold) {
    float dx = adjacentEdge.end_point.x - adjacentEdge.begin_point.x;
    float dy = adjacentEdge.end_point.y - adjacentEdge.begin_point.y;
    float angle = std::atan2(dy, dx) * 180.0f / CV_PI;

    cv::Point2f midpoint(
        (adjacentEdge.begin_point.x + adjacentEdge.end_point.x) / 2.0f,
        (adjacentEdge.begin_point.y + adjacentEdge.end_point.y) / 2.0f
    );

    cv::Mat rotationMatrix = cv::getRotationMatrix2D(midpoint, angle, 1.0);

    for (auto& ps : group.parkspaces) {
        std::vector<cv::Point2f> originalPoints;
        for (int k = 0; k < 4; ++k) {
            originalPoints.emplace_back(cv::Point2f(ps.points[k].x, ps.points[k].y));
        }
        std::vector<cv::Point2f> rotatedPoints;
        cv::transform(originalPoints, rotatedPoints, rotationMatrix);
        for (int k = 0; k < 4; ++k) {
            ps.points[k].x = rotatedPoints[k].x;
            ps.points[k].y = rotatedPoints[k].y;
        }
    }

    struct Midpoints {
        cv::Point2f leftMid;
        cv::Point2f rightMid;
        bool valid;
    };
    std::vector<Midpoints> parkingMidpoints(group.parkspaces.size(), Midpoints{cv::Point2f(-1, -1), cv::Point2f(-1, -1), false});

    float maxScore = -1.0f;
    int baseIndex = -1;

    for (size_t i = 0; i < group.parkspaces.size(); ++i) {
        common::Line2d firstEdge = group.parkspaces[i].getLine(0);
        float edge_dx = firstEdge.end_point.x - firstEdge.begin_point.x;
        float edge_dy = firstEdge.end_point.y - firstEdge.begin_point.y;
        float edge_angle = std::atan2(edge_dy, edge_dx) * 180.0f / CV_PI;
        float abs_edge_angle = std::abs(edge_angle);
        common::Line2d leftEdge, rightEdge;

        if (abs_edge_angle >= angle_threshold) {
            leftEdge = group.parkspaces[i].getLine(0);
            rightEdge = group.parkspaces[i].getLine(2);
        } else {
            leftEdge = group.parkspaces[i].getLine(1);
            rightEdge = group.parkspaces[i].getLine(3);
        }
        cv::Point2f leftMid(
            (leftEdge.begin_point.x + leftEdge.end_point.x) / 2.0f,
            (leftEdge.begin_point.y + leftEdge.end_point.y) / 2.0f
        );
        cv::Point2f rightMid(
            (rightEdge.begin_point.x + rightEdge.end_point.x) / 2.0f,
            (rightEdge.begin_point.y + rightEdge.end_point.y) / 2.0f
        );

        if (leftMid.x < rightMid.x) {
            parkingMidpoints[i].leftMid = leftMid;
            parkingMidpoints[i].rightMid = rightMid;
        } else {
            parkingMidpoints[i].leftMid = rightMid;
            parkingMidpoints[i].rightMid = leftMid;
        }
        parkingMidpoints[i].valid = true;

        if (group.parkspaces[i].score > maxScore) {
            maxScore = group.parkspaces[i].score;
            baseIndex = i;
        }
    }

    cv::Point2f baseLeftMid = parkingMidpoints[baseIndex].leftMid;
    cv::Point2f baseRightMid = parkingMidpoints[baseIndex].rightMid;

    for (size_t i = 0; i < group.parkspaces.size(); ++i) {
        if (i == baseIndex || !parkingMidpoints[i].valid) continue;

        cv::Point2f currentLeftMid = parkingMidpoints[i].leftMid;
        cv::Point2f currentRightMid = parkingMidpoints[i].rightMid;

        float deltaLeft = currentLeftMid.x - baseLeftMid.x;
        float deltaRight = currentRightMid.x - baseRightMid.x;

        bool leftExceeds = std::abs(deltaLeft) > delta_threshold;
        bool rightExceeds = std::abs(deltaRight) > delta_threshold;

        if (leftExceeds && rightExceeds) {
            float delta = (std::abs(deltaLeft) < std::abs(deltaRight)) ? deltaLeft : deltaRight;

            for (int k = 0; k < 4; ++k) {
                group.parkspaces[i].points[k].x -= delta;
            }
        }
    }

    cv::Mat inverseRotationMatrix = cv::getRotationMatrix2D(midpoint, -angle, 1.0);
    for (auto& ps : group.parkspaces) {
        std::vector<cv::Point2f> rotatedPoints;
        std::vector<cv::Point2f> currentPoints;
        for (int k = 0; k < 4; ++k) {
            currentPoints.emplace_back(cv::Point2f(ps.points[k].x, ps.points[k].y));
        }
        cv::transform(currentPoints, rotatedPoints, inverseRotationMatrix);
        for (int k = 0; k < 4; ++k) {
            ps.points[k].x = rotatedPoints[k].x;
            ps.points[k].y = rotatedPoints[k].y;
        }
    }
}

void TemplateMatcher::unifyAdjacentLines(ParkSpaceGroup &group, const std::vector<AdjacentLinePair> &pairs) {
    for (const auto &pair : pairs) {
        int idxA = pair.lineIdxA;
        int idxB = pair.lineIdxB;
        common::Line2d l1, l2;
        l1.begin_point = group.parkspaces[pair.psA].points[pair.cornerA1];
        l1.end_point = group.parkspaces[pair.psA].points[pair.cornerA2];
        l2.begin_point = group.parkspaces[pair.psB].points[pair.cornerB1];
        l2.end_point = group.parkspaces[pair.psB].points[pair.cornerB2];

        auto wrapAnglePi = [&](double angle) {
            while (angle >  M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;
            return angle;
        };

        double dxA = (l1.end_point.x - l1.begin_point.x);
        double dyA = (l1.end_point.y - l1.begin_point.y);
        double angleA  = std::atan2(dyA, dxA);      
        double angleA2 = std::atan2(-dyA, -dxA);   

        double dxB = (l2.end_point.x - l2.begin_point.x);
        double dyB = (l2.end_point.y - l2.begin_point.y);
        double angleB = std::atan2(dyB, dxB);

        double diff1 = angleA  - angleB;
        double diff2 = angleA2 - angleB;
        diff1 = wrapAnglePi(diff1);
        diff2 = wrapAnglePi(diff2);

        double bestA = angleA; 
        if (std::fabs(diff2) < std::fabs(diff1)) {
            bestA = angleA2;
        }

        double angleAvg = 0.5 * (bestA + angleB);
        angleAvg = wrapAnglePi(angleAvg);

        common::Point2d p1 = l1.begin_point;
        common::Point2d p2 = l1.end_point;
        common::Point2d p3 = l2.begin_point;
        common::Point2d p4 = l2.end_point;

        double cx = (p1.x + p2.x + p3.x + p4.x) / 4.0;
        double cy = (p1.y + p2.y + p3.y + p4.y) / 4.0;
        common::Point2d mid(cx, cy);

        double cosA = std::cos(angleAvg);
        double sinA = std::sin(angleAvg);

        common::Line2d unifyLine;
        unifyLine.begin_point = common::Point2d(mid.x - 10.0 * cosA, 
                                                mid.y - 10.0 * sinA);
        unifyLine.end_point   = common::Point2d(mid.x + 10.0 * cosA, 
                                                mid.y + 10.0 * sinA);

        common::Point2d pr1 = unifyLine.getProjectPoint(p1);
        common::Point2d pr2 = unifyLine.getProjectPoint(p2);
        common::Point2d pr3 = unifyLine.getProjectPoint(p3);
        common::Point2d pr4 = unifyLine.getProjectPoint(p4);

        group.parkspaces[pair.psA].points[pair.cornerA1] = pr1;
        group.parkspaces[pair.psA].points[pair.cornerA2] = pr2;
        group.parkspaces[pair.psB].points[pair.cornerB1] = pr3;
        group.parkspaces[pair.psB].points[pair.cornerB2] = pr4;
    }
}

std::vector<cv::Mat> TemplateMatcher::buildSub2OrigTransforms(int W, int H) {
    std::vector<cv::Mat> sub2orig(8, cv::Mat::eye(2,3,CV_32F));

    for(int i : {2,3})
    {
        cv::Mat M = cv::Mat::zeros(2,3,CV_32F);
        // x = 0*X +1*Y +0
        M.at<float>(0,1) = 1.f;
        // y = -1*X +0*Y +(H-1)
        M.at<float>(1,0) = -1.f;
        M.at<float>(1,2) = float(H - 1);
        sub2orig[i] = M;
    }

    // (4,5) => 180°: x=(W-1)-X, y=(H-1)-Y
    for(int i : {4,5})
    {
        cv::Mat M = cv::Mat::zeros(2,3,CV_32F);
        // x = -1*X +(W-1)
        M.at<float>(0,0) = -1.f;
        M.at<float>(0,2) = float(W - 1);
        // y = -1*Y +(H-1)
        M.at<float>(1,1) = -1.f;
        M.at<float>(1,2) = float(H - 1);
        sub2orig[i] = M;
    }

    // (6,7) => 270°: x=(W-1)-Y, y=X
    for(int i : {6,7})
    {
        cv::Mat M = cv::Mat::zeros(2,3,CV_32F);
        // x = -1*Y +(W-1)
        M.at<float>(0,1) = -1.f;
        M.at<float>(0,2) = float(W - 1);
        // y = +1*X
        M.at<float>(1,0) = 1.f;
        sub2orig[i] = M;
    }
    return sub2orig;
}


bool TemplateMatcher::match() {
    common::Dir::Mkdir(_file_system->getMatchDir());

    int index = 0; int nums = 4;
    UProgressTextValue(QObject::tr("Load images"), (index++), nums);
    // 读取图片
    cv::Mat mapImage = cv::imread(_file_system->getRgbImagePath());
    cv::Mat intensityMap = cv::imread(
        _file_system->getIntensityImagePath(), cv::IMREAD_GRAYSCALE);
    cv::Mat binaryIntensityMap = cv::imread(
        _file_system->getBinaryIntensityImagePath(), cv::IMREAD_GRAYSCALE);
    if (mapImage.empty() || intensityMap.empty() || binaryIntensityMap.empty()) {
        UWarn(QObject::tr("Unable load one or more images."));
        return false;
    }
    
    UProgressTextValue(QObject::tr("Load keypoints"), (index++), nums);
    auto unpaired_points_path = _file_system->getUnPairedPointsPath();
    auto pairs = loadPairsFile(unpaired_points_path);
    if (pairs.empty()) {
        UWarn(QObject::tr("Unable load keypoints %1.").arg(UTQ(unpaired_points_path)));
        return false;
    }

    UProgressTextValue(QObject::tr("Load slots"), (index++), nums);
    std::vector<PSlot> parkslots;
    auto slots_path = _file_system->getSlotsPath();
    loadTemplateFile(slots_path, parkslots, mapImage, intensityMap);
    if (parkslots.empty()) {
        UWarn(QObject::tr("Unable load templates %1.").arg(UTQ(slots_path)));
        return false;
    }
    
    UProgressTextValue(QObject::tr("Matching"), (index++), nums);
    std::vector<PSlot> matchTemplates;
    std::vector<cv::Point2f> singlePoints;

    int num_procs = omp_get_num_procs(); 
    int num_threads = num_procs / 2;
    if (num_threads > 8) {
        num_threads = 8;
    }

    omp_set_num_threads(num_threads);
    // ******跟现有库位模板匹配****** 
    auto start_time = std::chrono::high_resolution_clock::now();
    #pragma omp parallel for
    for (size_t pairIndex = 0; pairIndex < pairs.size(); ++pairIndex) {
        const auto &pair = pairs[pairIndex];
        if (pair.p2 == cv::Point2f(-1, -1)) {
            #pragma omp critical
            {
                singlePoints.emplace_back(pair.p1);
            }
            continue;
        }

        cv::Point2f center = (pair.p1 + pair.p2) * 0.5f;
        float angle = std::atan2(pair.p2.y - pair.p1.y, pair.p2.x - pair.p1.x) * 180.0f / CV_PI;

        cv::Mat mapROI = rotateAndExtractROI(mapImage, center, angle);
        cv::Mat intensityROI = rotateAndExtractROI(intensityMap, center, angle);

        float bestScore = 0.0f;
        cv::Point bestTopLeft;
        PSlot bestMatch;
        cv::Mat bestTemplate;

        for (size_t i = 0; i < parkslots.size(); ++i) {
            if (_rgb_template_rois.find(i) == _rgb_template_rois.end() || _intensity_template_rois.find(i) == _intensity_template_rois.end()) {
                UWarn(QObject::tr("Skipping template %1 due to missing ROI.").arg(i));
                continue;
            }

            const auto &mapTemplate = _rgb_template_rois[i];
            const auto &intensityTemplate = _intensity_template_rois[i];

            MatchResult mapResult = matchTemplate(mapROI, mapTemplate);
            MatchResult intensityResult = matchTemplate(intensityROI, intensityTemplate);

            float avgScore = (mapResult.maxVal + intensityResult.maxVal) / 2.0f;

            #pragma omp critical
            {
                if (avgScore > bestScore) {
                    bestScore = avgScore;
                    bestTopLeft = (mapResult.maxVal > intensityResult.maxVal) ? mapResult.maxLoc : intensityResult.maxLoc;
                    bestTemplate = (mapResult.maxVal > intensityResult.maxVal) ? mapTemplate.clone() : intensityTemplate.clone();
                }
            }
        }

        if (bestScore > SCORE_THRESH) {
            cv::Point topLeft(bestTopLeft.x + center.x - mapROI.cols / 2 + OFFSET,
                              bestTopLeft.y + center.y - mapROI.rows / 2 + OFFSET);
            cv::Point bottomRight(topLeft.x + bestTemplate.cols - 2 * OFFSET,
                                  topLeft.y + bestTemplate.rows - 2 * OFFSET);
            cv::Point2f topLeftROI(topLeft.x, topLeft.y);
            cv::Point2f topRightROI(bottomRight.x, topLeft.y);
            cv::Point2f bottomRightROI(bottomRight.x, bottomRight.y);
            cv::Point2f bottomLeftROI(topLeft.x, bottomRight.y);

            cv::Mat inverseRotationMatrix = cv::getRotationMatrix2D(center, -angle, 1.0);

            std::vector<cv::Point2f> roiPoints = {topLeftROI, topRightROI, bottomRightROI, bottomLeftROI};
            std::vector<cv::Point2f> imagePoints;
            cv::transform(roiPoints, imagePoints, inverseRotationMatrix);

            PSlot matchedSlot;
            matchedSlot.corners = {
                imagePoints[0],  // topLeft
                imagePoints[1],  // topRight
                imagePoints[2],  // bottomRight
                imagePoints[3]   // bottomLeft
            };
            matchedSlot.is_inferred = false;
            std::cout << matchedSlot.corners << std::endl;

            bool overlap = false;

            #pragma omp parallel for shared(overlap)
            for (size_t i = 0; i < parkslots.size(); ++i) {
                if (computeIoU(matchedSlot.corners, parkslots[i].corners) > IOU_THRESH) {
                    #pragma omp atomic write
                    overlap = true;
                }
            }

            for (size_t i = 0; i < matchTemplates.size(); ++i) {
                if (computeIoU(matchedSlot.corners, matchTemplates[i].corners) > IOU_THRESH) {
                    #pragma omp atomic write
                    overlap = true;
                }
            }

            if (!overlap) {
                std::vector<std::pair<cv::Point, cv::Point>> edges;
                for (size_t i = 0; i < matchedSlot.corners.size(); ++i) {
                    edges.emplace_back(matchedSlot.corners[i],
                                       matchedSlot.corners[(i + 1) % matchedSlot.corners.size()]);
                }
                float avgOcc = weightedAvgOcc(edges, binaryIntensityMap);

                if (avgOcc > INTENSITY_THRESH) {
                    #pragma omp critical
                    {
                        matchTemplates.push_back(matchedSlot);
                    }
                }
            }
        }
        else {
            #pragma omp critical
            {
                singlePoints.emplace_back(pair.p1);
                singlePoints.emplace_back(pair.p2);
            }
        }
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_time = end_time - start_time;
    UInfo(QObject::tr("Total processing time: %1 seconds.").arg(elapsed_time.count()));

    // ******确定库位具体尺寸******
    std::vector<PSlot> allSlots = parkslots;
    allSlots.insert(allSlots.end(), matchTemplates.begin(), matchTemplates.end());

    std::vector<ParkSpace> temp_slots;
    std::vector<ParkSpaceGroup> park_space_group_vec;
    std::vector<ParkSpace> matched_park_space;
    for (const auto& slot : allSlots) {
        ParkSpace ps;
        for (size_t i = 0; i < slot.corners.size() && i < 4; ++i) {
            ps.points[i] = common::Point2d(slot.corners[i].x, slot.corners[i].y);
        }
        temp_slots.push_back(ps);
    }

    generateGroup(park_space_group_vec, temp_slots);
    UInfo(QObject::tr("Park ground nums: %1.").arg(park_space_group_vec.size()));

    std::vector<EdgeInfo> edge_info_vec;
    cv::Mat intensityRGB;
    cv::cvtColor(intensityMap, intensityRGB, cv::COLOR_GRAY2BGR);
    cv::Mat drawImage = intensityRGB.clone();  
    // cv::Mat drawImage = bev_processor.findNearestPixel(intensityRGB, mask);
    cv::Mat enhancedIMap = drawImage.clone();
    cv::Mat enhancedIMap2 = drawImage.clone();
    cv::Mat enhancedIMap3 = drawImage.clone();
    cv::Mat mapImageCopy = mapImage.clone();
    cv::Mat mapImageCopy2 = mapImage.clone();

    DataPreProcessor data_preprocessor(_file_system);
    cv::Mat mask;
    cv::inRange(mapImageCopy, cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0), mask);
    cv::Mat mapImageOCR = data_preprocessor.findNearestPixel(mapImageCopy, mask);

    cv::Mat mask_i;
    cv::inRange(intensityRGB, cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0), mask_i);
    cv::Mat intensityImageOCR = data_preprocessor.findNearestPixel(intensityRGB, mask_i);

    for (const auto& group : park_space_group_vec) {
        std::vector<common::Line2d> all_lines;
        std::vector<bool> is_adjacent;
        std::vector<std::vector<int>> adjacency;
        std::vector<AdjacentLinePair> adj_line_pairs;
        drawGroupEdges(group, enhancedIMap, all_lines, is_adjacent, adjacency, adj_line_pairs);

        for (const auto& ps : group.parkspaces) {
            int widthAdd = 0, heightAdd = 0;
            calcWidthHeightAdjustments(ps, all_lines, is_adjacent, widthAdd, heightAdd);

            float length_01 = ps.getLine(0).getLength();
            float length_12 = ps.getLine(1).getLength();
            cv::Point2f shortEdgeStart, shortEdgeEnd;
            if (length_01 < length_12) {
                shortEdgeStart = cv::Point2f(ps.points[0].x, ps.points[0].y);
                shortEdgeEnd   = cv::Point2f(ps.points[1].x, ps.points[1].y);
            } else {
                shortEdgeStart = cv::Point2f(ps.points[1].x, ps.points[1].y);
                shortEdgeEnd   = cv::Point2f(ps.points[2].x, ps.points[2].y);
            }
            float angle = std::atan2(shortEdgeEnd.y - shortEdgeStart.y, shortEdgeEnd.x - shortEdgeStart.x) * 180.0f / CV_PI;
            float cx = 0, cy = 0;
            for (int k = 0; k < 4; k++){ cx += ps.points[k].x; cy +=ps.points[k].y; }
            cx /= 4; cy /= 4;
            cv::Point2f center(cx, cy);

            cv::Size roiSize(160,350);
            float bestAngle = angle;
            MatchResultInfo bestRes{0.0, cv::Size(), cv::Point()};

            // 在 angle ±2.5° 范围内以0.1° 步长遍历
            float startDelta = -2.5f;
            float endDelta = 2.5f;
            float step = 0.1f;
            int steps = (int)((endDelta - startDelta) / step + 0.5f);

            #pragma omp parallel
            {
                float localBestAngle = angle;
                MatchResultInfo localBestRes{0.0, cv::Size(), cv::Point()};

                #pragma omp for
                for (int d_i = 0; d_i <= steps; d_i++) {
                    float delta = startDelta + d_i * step;
                    float testAngle = angle + delta;
                    cv::Mat testROI = rotateAndExtractROI(drawImage, center, testAngle, roiSize);
                    MatchResultInfo testRes = matchAllTemplates(testROI, widthAdd, heightAdd);

                    if (testRes.score > localBestRes.score) {
                        localBestRes = testRes;
                        localBestAngle = testAngle;
                    }
                }

                #pragma omp critical
                {
                    if (localBestRes.score > bestRes.score) {
                        bestRes = localBestRes;
                        bestAngle = localBestAngle;
                    }
                }
            }

            std::cout << "Best score: " << bestRes.score 
                    << " Angle:" << bestAngle 
                    << " Size:" << bestRes.size.width << "x" << bestRes.size.height
                    << " WidthAdd:" << widthAdd << " HeightAdd:" << heightAdd << std::endl;

            if (bestRes.score > 0.55) {
                cv::Point2f roiTopLeftRot(
                    center.x - roiSize.width / 2.0f,
                    center.y - roiSize.height / 2.0f
                );

                cv::Point2f topLeftRot(
                    roiTopLeftRot.x + bestRes.location.x,
                    roiTopLeftRot.y + bestRes.location.y
                );
                cv::Point2f bottomRightRot(
                    topLeftRot.x + bestRes.size.width,
                    topLeftRot.y + bestRes.size.height
                );

                cv::Point2f topRightRot(bottomRightRot.x, topLeftRot.y);
                cv::Point2f bottomLeftRot(topLeftRot.x, bottomRightRot.y);

                std::vector<cv::Point2f> roiPoints = {topLeftRot, topRightRot, bottomRightRot, bottomLeftRot};
                std::vector<cv::Point2f> imagePoints;

                cv::Mat inverseRotationMatrix = cv::getRotationMatrix2D(center, -bestAngle, 1.0);
                cv::transform(roiPoints, imagePoints, inverseRotationMatrix);

                cv::Point cv_p[4];
                ParkSpace matchedPs;
                for (int m = 0; m < 4; m++) {
                    matchedPs.points[m] = common::Point2d(imagePoints[m].x, imagePoints[m].y);
                    cv_p[m] = cv::Point((int)matchedPs.points[m].x, (int)matchedPs.points[m].y);
                    cv::circle(drawImage, cv_p[m], 6, cv::Scalar(0,255,0), 6);
                }
                cv::line(drawImage, cv_p[0], cv_p[1], cv::Scalar(0,255,0), 2);
                cv::line(drawImage, cv_p[1], cv_p[2], cv::Scalar(0,255,0), 2);
                cv::line(drawImage, cv_p[2], cv_p[3], cv::Scalar(0,255,0), 2);
                cv::line(drawImage, cv_p[3], cv_p[0], cv::Scalar(0,255,0), 2);
                matchedPs.score = bestRes.score;
                matched_park_space.push_back(matchedPs);

                std::ostringstream ss;
                ss << std::fixed << std::setprecision(2) << bestRes.score;

                int sumX = 0;
                int sumY = 0;
                for (int i = 0; i < 4; i++) {
                    sumX += cv_p[i].x;
                    sumY += cv_p[i].y;
                }
                cv::Point textPos(sumX / 4, sumY / 4);

                cv::putText(drawImage, ss.str(), textPos, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,255,0), 2);
            }
        }
    }

    cv::imwrite(_file_system->getEdgeIntersectImagePath(), enhancedIMap);
    cv::imwrite(_file_system->getMatchedSlotsImagePath(), drawImage);

    // ******库位矫正&生成设计图******
    std::string ocr_output_dir = _file_system->getOCRDir();
    common::Dir::Mkdir(ocr_output_dir);
    OCRDetector ocr_detector(_ocr_detect_onnx_model_path);
    OCRRecognizer ocr_recognizer(_ocr_recognize_onnx_model_path, _dict_path);
    std::vector<ParkSpaceGroup> matched_ps_group_vec;
    generateGroup(matched_ps_group_vec, matched_park_space, true);
    int groupIdx = 0;
    std::string ocr_info_path = _file_system->getOCRDir() + "/ocr_info.txt";
    std::ofstream ocr_info_file(ocr_info_path);
    for (auto& group : matched_ps_group_vec) {
        std::vector<common::Line2d> all_lines;
        std::vector<bool> is_adjacent;
        std::vector<std::vector<int>> adjacency;
        std::vector<AdjacentLinePair> adj_line_pairs;
        drawGroupEdges(group, enhancedIMap2, all_lines, is_adjacent, adjacency, adj_line_pairs);

        int parkspaceIdx = 0;
        int n = group.parkspaces.size(); 
        std::vector<int> inOcrNumbers;

        for (auto &ps : group.parkspaces) {
            float length_01 = ps.getLine(0).getLength();
            float length_12 = ps.getLine(1).getLength();
            bool e0IsShort = (length_01 < length_12);

            cv::Point2f shortEdgeStart, shortEdgeEnd;
            if (e0IsShort) {
                shortEdgeStart = cv::Point2f(ps.points[0].x, ps.points[0].y);
                shortEdgeEnd = cv::Point2f(ps.points[1].x, ps.points[1].y);
            } else {
                shortEdgeStart = cv::Point2f(ps.points[1].x, ps.points[1].y);
                shortEdgeEnd = cv::Point2f(ps.points[2].x, ps.points[2].y);
            }

            float angle = std::atan2(shortEdgeEnd.y - shortEdgeStart.y, shortEdgeEnd.x - shortEdgeStart.x)*180.0f/CV_PI;

            float cx = 0, cy = 0;
            for (int k = 0; k < 4; k++) { cx += ps.points[k].x; cy += ps.points[k].y; }
            cx /= 4; cy /= 4;
            cv::Point2f center(cx, cy);

            cv::Mat rotMat = cv::getRotationMatrix2D(center, angle, 1.0);
            std::vector<cv::Point2f> originalPoints, rotatedPoints;
            for (int i = 0; i < 4; i++){
                originalPoints.push_back(cv::Point2f(ps.points[i].x, ps.points[i].y));
            }
            cv::transform(originalPoints, rotatedPoints, rotMat);
            cv::Mat rotatedRGBImage, rotatedIImage;
            cv::warpAffine(mapImageOCR, rotatedRGBImage, rotMat, mapImageOCR.size());// rgb_ocr
            cv::warpAffine(intensityImageOCR, rotatedIImage, rotMat, intensityImageOCR.size()); // intensity_ocr

            auto findLineIndex = [&](const common::Line2d& line)->int {
                for (size_t i = 0; i < all_lines.size(); ++i) {
                    const auto &L = all_lines[i];
                    if ((line.begin_point.isEqualTo(L.begin_point) && line.end_point.isEqualTo(L.end_point)) ||
                        (line.begin_point.isEqualTo(L.end_point) && line.end_point.isEqualTo(L.begin_point))) {
                        return (int)i;
                    }
                }
                return -1;
            };

            common::Line2d e0 = ps.getLine(0);
            common::Line2d e1 = ps.getLine(1);
            common::Line2d e2 = ps.getLine(2);
            common::Line2d e3 = ps.getLine(3);

            int idx0 = findLineIndex(e0);
            int idx1 = findLineIndex(e1);
            int idx2 = findLineIndex(e2);
            int idx3 = findLineIndex(e3);

            auto shrinkEdges = [&](int idx, bool isShortEdge) {
                if (idx >= 0 && idx < (int)is_adjacent.size()) {
                    if (is_adjacent[idx]) {
                        if (isShortEdge) {
                            int pA, pB;
                            if (idx == idx0) { pA = 0; pB = 1; }
                            else if (idx == idx1) { pA = 1; pB = 2; }
                            else if (idx == idx2) { pA = 2; pB = 3; }
                            else { pA = 3; pB = 0; }

                            float avgY = (rotatedPoints[pA].y + rotatedPoints[pB].y)*0.5f;
                            float offsetY = (avgY < center.y) ? 4.0f : -4.0f;

                            rotatedPoints[pA].y += offsetY;
                            rotatedPoints[pB].y += offsetY;
                        } else {
                            int pA, pB;
                            if (idx == idx0) { pA = 0; pB = 1; }
                            else if (idx == idx1) { pA = 1; pB = 2; }
                            else if (idx == idx2) { pA = 2; pB = 3; }
                            else { pA = 3; pB = 0; }

                            float avgX = (rotatedPoints[pA].x + rotatedPoints[pB].x)*0.5f;
                            float offsetX = (avgX < center.x) ? 4.0f : -4.0f;
                            rotatedPoints[pA].x += offsetX;
                            rotatedPoints[pB].x += offsetX;
                        }
                    }
                }
            };

            if (e0IsShort) {
                shrinkEdges(idx0, true);
                shrinkEdges(idx2, true);
                shrinkEdges(idx1, false);
                shrinkEdges(idx3, false);
            } else {
                shrinkEdges(idx0, false);
                shrinkEdges(idx2, false);
                shrinkEdges(idx1, true);
                shrinkEdges(idx3, true);
            }

            cv::Mat invRotMat = cv::getRotationMatrix2D(center, -angle, 1.0);
            std::vector<cv::Point2f> finalPoints;
            cv::transform(rotatedPoints, finalPoints, invRotMat);
            for (int i = 0; i < 4; i++){
                ps.points[i] = common::Point2d(finalPoints[i].x, finalPoints[i].y);
            }

            float minX = rotatedPoints[0].x, maxX = rotatedPoints[0].x;
            float minY = rotatedPoints[0].y, maxY = rotatedPoints[0].y;
            for (const auto& pt : rotatedPoints) {
                if (pt.x < minX) minX = pt.x;
                if (pt.x > maxX) maxX = pt.x;
                if (pt.y < minY) minY = pt.y;
                if (pt.y > maxY) maxY = pt.y;
            }
            
            // OCR
            int padX = 20;
            int padY = 40;
            minX = std::max(minX - padX, 0.0f);
            minY = std::max(minY - padY, 0.0f);
            maxX = std::min(maxX + padX, static_cast<float>(rotatedRGBImage.cols));
            maxY = std::min(maxY + padY, static_cast<float>(rotatedRGBImage.rows));
            
            std::vector<cv::Mat> ocr_input_regions;
            cv::Rect roi(static_cast<int>(minX), static_cast<int>(minY), static_cast<int>(maxX - minX), static_cast<int>(maxY - minY));
            cv::Mat croppedRGBImage = rotatedRGBImage(roi).clone();
            cv::Mat croppedIImage = rotatedIImage(roi).clone();
            std::string baseName = "group" + std::to_string(groupIdx) + "_ps" + std::to_string(parkspaceIdx);
            std::string rgbFilename = ocr_output_dir + "/rgb_" + baseName + "_0.png";
            cv::imwrite(rgbFilename, croppedRGBImage);
            std::string iFilename = ocr_output_dir + "/i_" + baseName + "_0.png";
            cv::imwrite(iFilename, croppedIImage);
            ocr_input_regions.push_back(croppedRGBImage);
            ocr_input_regions.push_back(croppedIImage);
            auto sub2orig = buildSub2OrigTransforms(croppedRGBImage.cols, croppedRGBImage.rows);
            if (ocr_info_file.is_open()) {
                ocr_info_file << baseName << " " << center.x << " " << center.y << std::endl;
            }
            for (int r = 1; r < 4; r++) {
                cv::Mat rotatedRGBCropped, rotatedICropped;
                cv::rotate(croppedRGBImage, rotatedRGBCropped, cv::ROTATE_90_CLOCKWISE);
                std::string rotatedRGBFilename = ocr_output_dir + "/rgb_" + baseName + "_" + std::to_string(r) + ".png";
                cv::imwrite(rotatedRGBFilename, rotatedRGBCropped);
                croppedRGBImage = rotatedRGBCropped; 
                cv::rotate(croppedIImage, rotatedICropped, cv::ROTATE_90_CLOCKWISE);
                std::string rotatedIFilename = ocr_output_dir + "/i_" + baseName + "_" + std::to_string(r) + ".png";
                cv::imwrite(rotatedIFilename, rotatedICropped);
                croppedIImage = rotatedICropped; 
                ocr_input_regions.push_back(croppedRGBImage);
                ocr_input_regions.push_back(croppedIImage);
            }

            std::vector<std::vector<DetectionBox>> detected_regions;
            detected_regions = ocr_detector.detect(ocr_input_regions);
            std::pair<std::string, float> best_result;
            // std::vector<cv::Point2f> bestPts;
            int img_idx = 0;
            int best_idx = -1;
            best_result.second = -1.0f;
            for(const auto& region_group : detected_regions) {
                for(const auto& box : region_group){
                    cv::UMat ucropped_img;
                    box.cropped_img.copyTo(ucropped_img);     
                    std::vector<std::pair<std::string, float>> ocr_results = ocr_recognizer.ocr(ucropped_img);
                    for(const auto& result : ocr_results){
                        const std::string& text = result.first;
                        float confidence = result.second;

                        if(std::all_of(text.begin(), text.end(), ::isdigit)){
                            if(confidence > best_result.second){
                                best_result = result;
                                best_idx = img_idx;
                                // bestPts = box.points;
                            }
                        }
                    }
                }
                img_idx++;
            }
            if(best_result.second != -1.0f){
                std::string cleaned_text = remove_leading_zeros(best_result.first);
                try {
                    int number = std::stoi(cleaned_text);
                    inOcrNumbers.push_back(number);
                } catch (...) {
                    inOcrNumbers.push_back(-1);
                }

                // std::vector<cv::Point2f> boxInImg0(4);
                // cv::Mat M_sub2sub0 = sub2orig[best_idx];
                // for(int i = 0; i < 4; i++){
                //     float x = bestPts[i].x, y = bestPts[i].y;
                //     float X = M_sub2sub0.at<float>(0,0)*x + M_sub2sub0.at<float>(0,1)*y + M_sub2sub0.at<float>(0,2) + minX;
                //     float Y = M_sub2sub0.at<float>(1,0)*x + M_sub2sub0.at<float>(1,1)*y + M_sub2sub0.at<float>(1,2) + minY;
                //     boxInImg0[i] = cv::Point2f(X,Y);
                // }

                // std::vector<cv::Point2f> finalBox(4);
                // cv::Mat invRot = cv::getRotationMatrix2D(center, -angle, 1.0); 
                // cv::transform(boxInImg0, finalBox, invRot);

                // std::vector<std::vector<cv::Point>> contour(1);
                // for(auto &pt : finalBox) contour[0].push_back({(int)pt.x, (int)pt.y});
                // cv::polylines(mapImageCopy2, contour, true, cv::Scalar(0,255,0), 3);

                cv::putText(mapImageCopy2, best_result.first, center,
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);

                std::string confidenceStr = std::to_string(best_result.second);
                size_t dotPos = confidenceStr.find('.');
                if (dotPos != std::string::npos && dotPos + 3 < confidenceStr.length()) {
                    confidenceStr = confidenceStr.substr(0, dotPos + 3);
                }
                cv::Point cpt2(center.x, center.y + 30);
                cv::putText(mapImageCopy2, confidenceStr, cpt2,
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);

            } else {
                inOcrNumbers.push_back(-1);
            }
            parkspaceIdx++;
        }

        // 进行编号纠正
        std::vector<int> correctedNums(n, -1);
        bool ok = true;
        if (n > 1) ok = analyzeAndCorrectGroupNumbers(inOcrNumbers, correctedNums, adjacency);
        else if (n == 1 && inOcrNumbers[0] >= 0) correctedNums[0] = inOcrNumbers[0];
        else if (n == 1 && inOcrNumbers[0] < 0) ok = false;
        if(!ok) {
            std::cout << "[Group " << groupIdx << "] 编号纠正失败.\n";
        } else {
            std::cout << "[Group " << groupIdx << "] 编号纠正成功:\n";
            for(int i = 0; i < n; i++) {
                std::cout << "  parkspace " << i << " => " << correctedNums[i] << "\n";
                const auto& ps = group.parkspaces[i];
                float sum_x = 0.0f, sum_y = 0.0f;
                for(int k = 0; k < 4; k++) {
                    sum_x += ps.points[k].x;
                    sum_y += ps.points[k].y;
                }
                cv::Point2f centroid(sum_x / 4.0f, sum_y / 4.0f);
                std::string text = std::to_string(correctedNums[i]);
                cv::putText(enhancedIMap3, text, centroid, cv::FONT_HERSHEY_SIMPLEX, 
                    0.7, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            }
        }
        // 校正模板匹配中存在偏移的库位 && 统一中心线
        if (group.parkspaces.size() <= 1 || group.parkspaces.size() > 4) {
            unifyAdjacentLines(group, adj_line_pairs);
            groupIdx++;
            continue;
        }
        bool adjacentEdgeFound = false;
        common::Line2d adjacentEdge;
        for (size_t i = 0; i < all_lines.size(); ++i) {
            if (is_adjacent[i]) {
                adjacentEdge = all_lines[i];
                adjacentEdgeFound = true;
                break;
            }
        }
        if (!adjacentEdgeFound) {
            groupIdx++;
            continue;
        }
        CorrectParkingGroup(group, adjacentEdge);
        unifyAdjacentLines(group, adj_line_pairs);
        
        groupIdx++;
    }

    ocr_info_file.close();
    cv::imwrite(_file_system->getOCRImagePath(), mapImageCopy2);

    // ******绘制未匹配角点******
    std::vector<cv::Point2f> filteredSinglePoints;
    size_t num_singlePoints = singlePoints.size();
    #pragma omp parallel
    {
        std::vector<cv::Point2f> local_filtered;

        #pragma omp for nowait
        for (size_t i = 0; i < num_singlePoints; ++i) {
            const cv::Point2f &point = singlePoints[i];
            if (!isPointWithinSlot(point, matched_ps_group_vec, 10)) { 
                local_filtered.emplace_back(point);
            }
        }

        #pragma omp critical
        {
            filteredSinglePoints.insert(filteredSinglePoints.end(), local_filtered.begin(), local_filtered.end());
        }
    }
    for (const auto &point : filteredSinglePoints) {
        cv::circle(enhancedIMap3, point, 8, cv::Scalar(0, 0, 255), 8);
    }

    std::string outputFilePath = _file_system->getFinalParkingSlotsPath();
    std::ofstream outputFile(outputFilePath);
    if (!outputFile.is_open()) {
        UWarn(QObject::tr("Unable save final parking slots to %1").arg(UTQ(outputFilePath)));
        return false;
    }
    for (const auto& group : matched_ps_group_vec) {
        for (auto &ps : group.parkspaces) {
            cv::Scalar color;
            if (ps.score >= 0.75f) {
                color = cv::Scalar(0, 255, 0);
            }
            else if (ps.score >= 0.65f) {
                color = cv::Scalar(255, 0, 0); 
            }
            else if (ps.score >= 0.55f) {
                color = cv::Scalar(230, 230, 6); 
            }
            else {
                continue; 
            }

            cv::Point cv_p[4];
            for (int i = 0; i < 4; i++) {
                cv_p[i] = cv::Point(static_cast<int>(ps.points[i].x), static_cast<int>(ps.points[i].y));
                float point_cloud_x = MIN_X + (ps.points[i].x * PIXEL_SIZE);
                float point_cloud_y = MAX_Y - (ps.points[i].y * PIXEL_SIZE);
                outputFile << point_cloud_x << " " << point_cloud_y << std::endl;
            }
            outputFile << std::endl;
            for (int i = 0; i < 4; i++) {
                cv::circle(enhancedIMap3, cv_p[i], 6, color, 6);
            }
            cv::line(enhancedIMap3, cv_p[0], cv_p[1], color, 2);
            cv::line(enhancedIMap3, cv_p[1], cv_p[2], color, 2);
            cv::line(enhancedIMap3, cv_p[2], cv_p[3], color, 2);
            cv::line(enhancedIMap3, cv_p[3], cv_p[0], color, 2);
        }
    }

    outputFile.close();
    cv::imwrite(_file_system->getMatchedEdgeIntersectImagePath(), enhancedIMap2);
    cv::imwrite(_file_system->getRevisedMatchedSlotsImagePath(), enhancedIMap3);
    
    UInfo(QObject::tr("Template match result save to %1 successfully!").arg(UTQ(outputFilePath)));
    return true;
}
}