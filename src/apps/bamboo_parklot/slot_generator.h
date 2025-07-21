#pragma once
#include <memory>
#include <vector>
#include <opencv2/core.hpp>

namespace welkin::bamboo {
const float MIN_DISTANCE = 90.0f;
const float MAX_DISTANCE = 150.0f;
const float DISTANCE_THRESHOLD = 15.0f;
const float MIN_DISTANCE_PERP = 200.0f;
const float MAX_DISTANCE_PERP = 325.0f;
const float MAX_ANGLE_DIFF = 5.0f;
const float INTENSITY_THRESHOLD = 0.3f;
const float IOU_THRESHOLD = 0.1f;
const int MIN_CROSSING_EDGES = 2;

struct PS {
    std::vector<cv::Point> slot;
    float avg_occ;
    bool is_inferred;
};
class ParklotFileSystem;
class SlotGenerator {
public:
    using Ptr = std::shared_ptr<SlotGenerator>;
    SlotGenerator(ParklotFileSystem* fs);
    virtual ~SlotGenerator();

    bool generate();
private:
    std::vector<cv::Point> loadKeypoints(const std::string& file_path);
    std::vector<std::pair<cv::Point, cv::Point>> findNeighborPairs(
                    const std::vector<cv::Point>& keypoints, float min_distance = MIN_DISTANCE, float max_distance = MAX_DISTANCE);
    float pointToLineDist(float px, float py, float A, float B, float C);
    std::vector<cv::Point> findPerpendicularPoints(const std::vector<cv::Point>& keypoints, const std::pair<cv::Point, cv::Point>& pair,
                    float distance_threshold = DISTANCE_THRESHOLD, float min_distance = MIN_DISTANCE_PERP, float max_distance = MAX_DISTANCE_PERP);
    bool checkSlope(const std::pair<cv::Point, cv::Point>& pair, const cv::Point& p1, const cv::Point& p2, float max_angle_diff = MAX_ANGLE_DIFF);
    float intensityOcc(const cv::Mat& intensity_map, const cv::Point& p1, const cv::Point& p2, int buffer = 5);
    float computeIoU(const std::vector<cv::Point>& slot1, const std::vector<cv::Point>& slot2);
    float weightedAvgOcc(const std::vector<std::pair<cv::Point, cv::Point>>& edges, const cv::Mat& intensity_map);
    bool checkIfCrossWall(const std::vector<cv::Point>& slot, const cv::Mat& mask, int min_crossing_edges = MIN_CROSSING_EDGES);
    std::vector<cv::Point> orderPoints(const std::vector<cv::Point>& points);
    // void drawParkingSlots(cv::Mat& image, const std::vector<PS>& slots);
    void processPairs(const std::vector<cv::Point>& keypoints,
                    const std::vector<std::pair<cv::Point, cv::Point>>& pairs, const cv::Mat& intensity_map, const cv::Mat& high_diff_map,
                    std::vector<PS>& parking_slots, std::vector<std::pair<cv::Point, cv::Point>>& non_slot_pairs,
                    float intensity_threshold = INTENSITY_THRESHOLD, float iou_threshold = IOU_THRESHOLD);
private:
    ParklotFileSystem* _file_system;
};
}