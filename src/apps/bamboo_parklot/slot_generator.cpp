#include "slot_generator.h"
#include <QFileInfo>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>
#include <common/base/dir.h>
#include <uface/uface.hpp>
#include "file_system.h"
namespace welkin::bamboo {
SlotGenerator::SlotGenerator(ParklotFileSystem* fs) : _file_system(fs) {
    Q_ASSERT(fs);
}
SlotGenerator::~SlotGenerator() {}

bool SlotGenerator::generate() {
    // 创建slot文件夹
    common::Dir::Mkdir(_file_system->getSlotDir());

    int index = 0; int nums = 5;
    UProgressTextValue(QObject::tr("Load images"), (index++), nums);

    auto rgb_map_path = _file_system->getRgbImagePath();
    cv::Mat map_image = cv::imread(rgb_map_path);
    if (map_image.empty()) {
        UWarn(QObject::tr("Failed to load map image from %1.").arg(UTQ(rgb_map_path)));
        return false;
    }
    
    auto binary_intensity_path = _file_system->getBinaryIntensityImagePath();
    cv::Mat intensity_map = cv::imread(binary_intensity_path, cv::IMREAD_GRAYSCALE);
    if (intensity_map.empty()) {
        UWarn(QObject::tr("Failed to load intensity map from %1.").arg(UTQ(binary_intensity_path)));
        return false;
    }
    
    auto high_diff_path = _file_system->getHeightDiffImagePath();
    cv::Mat high_diff_map = cv::imread(high_diff_path, cv::IMREAD_GRAYSCALE);
    if (high_diff_map.empty()) {
        UWarn(QObject::tr("Failed to load high diff map from %1.").arg(UTQ(high_diff_path)));
        return false;
    }
    
    UProgressTextValue(QObject::tr("Load keypoints"), (index++), nums);
    auto keypoints_path = _file_system->getKeypointsPath();
    std::vector<cv::Point> keypoints = loadKeypoints(keypoints_path);
    if (keypoints.empty()) {
        UWarn(QObject::tr("Failed to load keypoints from %1.").arg(UTQ(keypoints_path)));
        return false;
    }

    UProgressTextValue(QObject::tr("Find neighbor pairs"), (index++), nums);
    std::vector<bool> used_keypoints(keypoints.size(), false);
    std::vector<std::pair<cv::Point, cv::Point>> pairs = findNeighborPairs(keypoints);
    
    UProgressTextValue(QObject::tr("Process pairs"), (index++), nums);
    std::vector<PS> parking_slots;
    std::vector<std::pair<cv::Point, cv::Point>> non_slot_pairs;
    processPairs(keypoints, pairs, intensity_map, high_diff_map, parking_slots, non_slot_pairs);

    for (const auto& slot : parking_slots) {
        for (const auto& pt : slot.slot) {
            auto it = std::find(keypoints.begin(), keypoints.end(), pt);
            if (it != keypoints.end()) {
                size_t index = std::distance(keypoints.begin(), it);
                used_keypoints[index] = true;
            }
        }
    }
    
    for (const auto& slot : parking_slots) {
        const auto& points = slot.slot;
        cv::Scalar color = slot.is_inferred ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 255, 0); 
        for (size_t i = 0; i < points.size(); ++i) {
            cv::circle(map_image, points[i], 6, color, 6);
            cv::line(map_image, points[i], points[(i + 1) % points.size()], color, 2);
        }
    }

    UProgressTextValue(QObject::tr("Save parking slots"), (index++), nums);
    auto slots_image_path = _file_system->getSlotsImagePath();
    if (cv::imwrite(slots_image_path, map_image)) {
        UInfo(QObject::tr("Slots image saved to %1").arg(UTQ(slots_image_path)));
    } else {
        UWarn(QObject::tr("Failed to save slots image to %1").arg(UTQ(slots_image_path)));
        return false;
    }

    float min_x = -40.0f; float max_y = 60.0f; // 小昆山
    // float min_x = -30.0f; float max_y = 40.0f; // 九里亭
    // float min_x = -34.0f; float max_y = 53.0f; // 金浩园_2
    // float min_x = -50.0f; float max_y = 100.0f; // 金浩园_3
    // float min_x = -40.0f; float max_y = 60.0f; // 金浩园_4
    float pixel_size = 0.02f;

    std::string slots_path = _file_system->getSlotsPath();
    std::ofstream slots_file(slots_path);
    if (!slots_file.is_open()) {
        UWarn(QObject::tr("Failed to save slots to %1").arg(UTQ(slots_path)));
        return false;
    }

    for (const auto& slot : parking_slots) {
        for (const auto& point : slot.slot) {
            
            float point_cloud_x = min_x + (point.x * pixel_size);
            float point_cloud_y = max_y - (point.y * pixel_size);

            slots_file << point_cloud_x << " " << point_cloud_y << std::endl;
        }
        slots_file << slot.is_inferred << std::endl;
        slots_file << std::endl;
    }
    slots_file.close();
    UInfo(QObject::tr("Slots saved to %1").arg(UTQ(slots_path)));

    std::string unpaired_points_path = _file_system->getUnPairedPointsPath();
    std::ofstream unpaired_file(unpaired_points_path);
    if (!unpaired_file.is_open()) {
        UWarn(QObject::tr("Failed to save unpaired slots to %1").arg(UTQ(unpaired_points_path)));
        return false;
    }

    for (const auto& pair : non_slot_pairs) {
        unpaired_file << pair.first.x << " " << pair.first.y << " " << pair.second.x << " " << pair.second.y << std::endl;
    }
    for (size_t i = 0; i < keypoints.size(); ++i) {
        if (!used_keypoints[i]) {
            const auto& pt = keypoints[i];
            unpaired_file << pt.x << " " << pt.y << std::endl; 
        }
    }
    unpaired_file.close();
    UInfo(QObject::tr("Unpaired slots saved to %1").arg(UTQ(unpaired_points_path)));
    
    return true;
}

// 加载关键点
std::vector<cv::Point> SlotGenerator::loadKeypoints(const std::string& file_path) {
    std::vector<cv::Point> keypoints;
    std::ifstream file(file_path);
    if (!file.is_open()) {
        UWarn(QObject::tr("Failed to open keypoints file!"));
        return keypoints;
    }
    int x, y;
    float confidence;
    while (file >> x >> y >> confidence) {
        keypoints.push_back(cv::Point(x, y));
    }
    return keypoints;
}

// 查找邻域点
std::vector<std::pair<cv::Point, cv::Point>> SlotGenerator::findNeighborPairs(
    const std::vector<cv::Point>& keypoints, float min_distance, float max_distance) {
    std::vector<std::pair<cv::Point, cv::Point>> pairs;
    int num_keypoints = keypoints.size();
    for (int i = 0; i < num_keypoints; i++) {
        for (int j = i + 1; j < num_keypoints; j++) {
            float distance = cv::norm(keypoints[i] - keypoints[j]);
            if (distance >= min_distance && distance <= max_distance) {
                pairs.push_back(std::make_pair(keypoints[i], keypoints[j]));
            }
        }
    }
    return pairs;
}

// 计算点线距离
float SlotGenerator::pointToLineDist(float px, float py, float A, float B, float C) {
    return std::abs(A * px + B * py + C) / std::sqrt(A * A + B * B);
}

// 查找垂直点
std::vector<cv::Point> SlotGenerator::findPerpendicularPoints(
    const std::vector<cv::Point>& keypoints, const std::pair<cv::Point, cv::Point>& pair,
    float distance_threshold, float min_distance, float max_distance) {
    
    cv::Point kp1 = pair.first, kp2 = pair.second;
    float x1 = kp1.x, y1 = kp1.y;
    float x2 = kp2.x, y2 = kp2.y;

    float A1 = x2 - x1;
    float B1 = y2 - y1;
    float C1 = -(A1 * x1 + B1 * y1);

    float A2 = x2 - x1;
    float B2 = y2 - y1;
    float C2 = -(A2 * x2 + B2 * y2);

    std::vector<cv::Point> perp_points;
    for (const auto& kp : keypoints) {
        if (kp == kp1 || kp == kp2) continue;

        float x = kp.x, y = kp.y;
        float dist_kp1 = cv::norm(kp - kp1);
        float dist_kp2 = cv::norm(kp - kp2);
        float distance_to_line1 = pointToLineDist(x, y, A1, B1, C1);
        float distance_to_line2 = pointToLineDist(x, y, A2, B2, C2);

        if ((distance_to_line1 < distance_threshold && dist_kp1 >= min_distance && dist_kp1 <= max_distance) ||
            (distance_to_line2 < distance_threshold && dist_kp2 >= min_distance && dist_kp2 <= max_distance)) {
            perp_points.push_back(kp);
        }
    }
    return perp_points;
}

// 检查斜率
bool SlotGenerator::checkSlope(const std::pair<cv::Point, cv::Point>& pair, 
    const cv::Point& p1, const cv::Point& p2, float max_angle_diff) {
    float dx1 = pair.second.x - pair.first.x;
    float dy1 = pair.second.y - pair.first.y;
    float dx2 = p2.x - p1.x;
    float dy2 = p2.y - p1.y;

    float angle1 = atan2(dy1, dx1) * 180.0 / M_PI;
    float angle2 = atan2(dy2, dx2) * 180.0 / M_PI;
    float angle_diff = std::abs(angle1 - angle2);

    if (angle_diff > 180.0) angle_diff = 360.0 - angle_diff;
    if (angle_diff > 90.0) angle_diff = 180.0 - angle_diff;

    return angle_diff <= max_angle_diff;
}

// 计算强度占用率
float SlotGenerator::intensityOcc(const cv::Mat& intensity_map, const cv::Point& p1, const cv::Point& p2, int buffer) {
    cv::Mat mask = cv::Mat::zeros(intensity_map.size(), CV_8UC1);
    cv::line(mask, p1, p2, cv::Scalar(255), buffer);

    cv::Mat line_region;
    cv::bitwise_and(mask, intensity_map, line_region);
    
    int total_count = cv::countNonZero(mask);
    int occupied_count = cv::countNonZero(line_region);

    return total_count > 0 ? static_cast<float>(occupied_count) / total_count : 0;
}

// 计算两个停车位的IoU
float SlotGenerator::computeIoU(const std::vector<cv::Point>& slot1, const std::vector<cv::Point>& slot2) {
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

// 计算加权平均占用率
float SlotGenerator::weightedAvgOcc(const std::vector<std::pair<cv::Point, cv::Point>>& edges, const cv::Mat& intensity_map) {
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

// 检查停车位是否穿过高差区域
bool SlotGenerator::checkIfCrossWall(const std::vector<cv::Point>& slot, const cv::Mat& mask, int min_crossing_edges) {
    int crossing_count = 0;
    int num_points = slot.size();

    for (int i = 0; i < num_points; ++i) {
        cv::Point p1 = slot[i];
        cv::Point p2 = slot[(i + 1) % num_points];

        cv::Mat edge_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
        cv::line(edge_mask, p1, p2, cv::Scalar(255), 1);

        cv::Mat overlap;
        cv::bitwise_and(edge_mask, mask, overlap);
        if (cv::countNonZero(overlap) > 0) {
            crossing_count++;
        }

        if (crossing_count >= min_crossing_edges) {
            return true;
        }
    }
    return false;
}

// 对点进行排序
std::vector<cv::Point> SlotGenerator::orderPoints(const std::vector<cv::Point>& points) {
    if (points.size() != 4) {
        throw std::invalid_argument("Input must be a list of 4 points.");
    }
    
    cv::Point2f center(0, 0);
    for (const auto& pt : points) {
        center.x += pt.x;
        center.y += pt.y;
    }
    center.x /= 4;
    center.y /= 4;

    std::vector<cv::Point> sorted_points = points;
    std::sort(sorted_points.begin(), sorted_points.end(), [center](const cv::Point& a, const cv::Point& b) {
        float angleA = atan2(a.y - center.y, a.x - center.x);
        float angleB = atan2(b.y - center.y, b.x - center.x);
        return angleA < angleB;
    });
    return sorted_points;
}

void SlotGenerator::processPairs(const std::vector<cv::Point>& keypoints,const std::vector<std::pair<cv::Point, cv::Point>>& pairs,
                                      const cv::Mat& intensity_map, const cv::Mat& high_diff_map, std::vector<PS>& parking_slots,
                                      std::vector<std::pair<cv::Point, cv::Point>>& non_slot_pairs,
                                      float intensity_threshold, float iou_threshold) {
    // std::vector<ParkingSlot> parking_slots;
    // std::vector<std::pair<cv::Point, cv::Point>> non_slot_pairs;

    for (const auto& pair : pairs) {
        auto perp_points = findPerpendicularPoints(keypoints, pair);

        if (perp_points.empty()) {
            non_slot_pairs.push_back(pair);
            continue;
        }

        std::vector<cv::Point> perp_points_to_remove;
        bool slot_formed = false;

        // 处理两个垂直点的情况
        for (size_t i = 0; i < perp_points.size(); ++i) {
            for (size_t j = i + 1; j < perp_points.size(); ++j) {
                if (checkSlope(pair, perp_points[i], perp_points[j])) {
                    std::vector<cv::Point> slot = {pair.first, pair.second, perp_points[i], perp_points[j]};
                    std::vector<cv::Point> ordered_slot = orderPoints(slot);
                    
                    perp_points_to_remove.push_back(perp_points[i]);
                    perp_points_to_remove.push_back(perp_points[j]);
                    
                    bool should_add = true;
                    float avg_occ = weightedAvgOcc(
                        { {ordered_slot[0], ordered_slot[1]}, {ordered_slot[1], ordered_slot[2]},
                          {ordered_slot[2], ordered_slot[3]}, {ordered_slot[3], ordered_slot[0]} }, intensity_map);


                    if (avg_occ > intensity_threshold && !checkIfCrossWall(ordered_slot, high_diff_map)) {
                        for (auto it = parking_slots.begin(); it != parking_slots.end();) {
                            float iou = computeIoU(ordered_slot, it->slot);
                            if (iou > iou_threshold) {
                                if (it->avg_occ >= avg_occ) {
                                    should_add = false;
                                    break;
                                } else {
                                    it = parking_slots.erase(it);
                                    continue;
                                }
                            }
                            ++it;
                        }
                    } else {
                        should_add = false;
                    }

                    if (should_add) {
                        parking_slots.push_back({ordered_slot, avg_occ, false});
                        slot_formed = true;
                    }
                }
            }
        }

        for (const auto& p : perp_points_to_remove) {
            perp_points.erase(std::remove(perp_points.begin(), perp_points.end(), p), perp_points.end());
        }

        // 处理只有一个垂直点的情况
        for (const auto& p : perp_points) {
            cv::Point v1 = pair.second - pair.first;
            cv::Point v2 = p - pair.first;
            cv::Point v3 = p - pair.second;

            float dot1 = v1.dot(v2) / (cv::norm(v1) * cv::norm(v2));
            float dot2 = v1.dot(v3) / (cv::norm(v1) * cv::norm(v3));

            if (std::abs(dot1) > 0.1f && std::abs(dot2) > 0.1f) {
                continue;
            }

            cv::Point new_point;
            if (std::abs(dot1) < std::abs(dot2)) {
                new_point = cv::Point(pair.second.x + (p.x - pair.first.x), pair.second.y + (p.y - pair.first.y));
                slot_formed = true;
            } else {
                new_point = cv::Point(pair.first.x + (p.x - pair.second.x), pair.first.y + (p.y - pair.second.y));
                slot_formed = true;
            }

            std::vector<cv::Point> slot = {pair.first, pair.second, p, new_point};
            std::vector<cv::Point> ordered_slot = orderPoints(slot);

            float avg_occ = weightedAvgOcc(
                { {ordered_slot[0], ordered_slot[1]}, {ordered_slot[1], ordered_slot[2]},
                  {ordered_slot[2], ordered_slot[3]}, {ordered_slot[3], ordered_slot[0]} }, intensity_map);

        
            
            bool should_add = true;
            if (avg_occ > intensity_threshold && !checkIfCrossWall(ordered_slot, high_diff_map)) {
                for (auto it = parking_slots.begin(); it != parking_slots.end();) {
                    float iou = computeIoU(ordered_slot, it->slot);
                    if (iou > iou_threshold) {
                        if (it->avg_occ >= avg_occ) {
                            should_add = false;
                            break;
                        } else {
                            it = parking_slots.erase(it);
                            continue;
                        }
                    }
                    ++it;
                } 
            } else {
                should_add = false;
            }
                
            if (should_add) {
                parking_slots.push_back({ordered_slot, avg_occ, true});
            }

        }

        if (!slot_formed) {
            non_slot_pairs.push_back(pair);
        }
    }
}
}