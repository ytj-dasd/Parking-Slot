#pragma once
#include <memory>
#include <opencv2/core.hpp>
#include <common/geometry/pose3.h>
#include "parkspace.h"
#include "parkspacegroup.h"

namespace welkin::bamboo {
struct PointPair {
    cv::Point2f p1, p2;
};

struct PSlot {
    std::vector<cv::Point2f> corners;
    bool is_inferred;
};

struct EdgeInfo {
    common::Point2d p1;
    common::Point2d p2;
    cv::Scalar color;
};

struct MatchResult {
    double maxVal;       
    cv::Point maxLoc;    
};

struct MatchResultInfo {
    double score;
    cv::Size size;
    cv::Point location;
};

struct AdjacentLinePair {
    int lineIdxA, lineIdxB;
    int psA, cornerA1, cornerA2;
    int psB, cornerB1, cornerB2;
};

const int OFFSET = 5;
const double ANGLE_DIFF_THRESH = 10.0f;
const double OVERLAP_THRESH = 50;
const float INTENSITY_THRESH = 0.4f;
const float IOU_THRESH = 0.3f;
const float SCORE_THRESH = 0.45;
const cv::Size ROISIZE(166, 650);

class ParklotFileSystem;
class TemplateMatcher {
public:
    using Ptr = std::shared_ptr<TemplateMatcher>;
    TemplateMatcher(ParklotFileSystem* fs);
    virtual ~TemplateMatcher();
    bool match();
    void setDetectOnnxModelPath(const std::string& model_path);
    void setRecognizeOnnxModelPath(const std::string& model_path);
    void setDictPath(const std::string& dict);
private:
    std::vector<PointPair> loadPairsFile(const std::string &filePath);
    void loadTemplateFile(const std::string &filePath, std::vector<PSlot>& parkslots, const cv::Mat &mapImage, const cv::Mat &intensityMap);
    
    cv::Mat rotateAndExtractTemplate(const cv::Mat &image, const PSlot &slot, int offset = OFFSET);
    cv::Mat extractROI(const cv::Mat &image, const cv::Point2f &center, const cv::Size &size = ROISIZE);
    cv::Mat rotateAndExtractROI(const cv::Mat &image, const cv::Point2f &center, float angle, const cv::Size &size = ROISIZE);
    MatchResult matchTemplate(const cv::Mat &image, const cv::Mat &templateImage);
    float computeIoU(const std::vector<cv::Point2f>& slot1, const std::vector<cv::Point2f>& slot2);
    float intensityOcc(const cv::Mat& intensity_map, const cv::Point& p1, const cv::Point& p2, int buffer = 5);
    float weightedAvgOcc(const std::vector<std::pair<cv::Point, cv::Point>>& edges, const cv::Mat& intensity_map);
    bool isPointWithinSlot(const cv::Point2f &point, const std::vector<ParkSpaceGroup> &park_space_groups, int expansion);
    void generateGroup(std::vector<ParkSpaceGroup> &park_space_group_vec, std::vector<ParkSpace> temp_slots, bool is_matched = false);
    void drawGroupEdges(const ParkSpaceGroup &group, cv::Mat &mapImage,
            std::vector<common::Line2d> &all_lines, std::vector<bool> &is_adjacent, 
            std::vector<std::vector<int>> &adjacency, std::vector<AdjacentLinePair> &adj_line_pairs);
    void calcWidthHeightAdjustments(const ParkSpace &parking_space,
            const std::vector<common::Line2d> &all_lines, const std::vector<bool> &is_adjacent, int &widthAdd, int &heightAdd);
    cv::Mat createTemplateImg(int width, int height);
    MatchResultInfo matchAllTemplates(const cv::Mat &roi, int widthAdd, int heightAdd);
    std::string remove_leading_zeros(const std::string& text);
    void backtrack_sequences(int current_idx, int current_num,
            const std::vector<std::vector<int>>& adjacency, std::vector<int>& cur_seq, std::vector<std::vector<int>>& all_seqs);
    bool analyzeAndCorrectGroupNumbers(const std::vector<int>& inOcrNumbers, std::vector<int>& outCorrectedNumbers,
            const std::vector<std::vector<int>>& adjacency);
    void CorrectParkingGroup(ParkSpaceGroup& group, const common::Line2d& adjacentEdge, 
            float angle_threshold = 20.0f, float delta_threshold = 10.0f);
    void unifyAdjacentLines(ParkSpaceGroup &group, const std::vector<AdjacentLinePair> &pairs);
    std::vector<cv::Mat> buildSub2OrigTransforms(int W, int H);
private:
    ParklotFileSystem* _file_system = nullptr;

    std::unordered_map<int, cv::Mat> _rgb_template_rois;
    std::unordered_map<int, cv::Mat> _intensity_template_rois;

    std::string _ocr_detect_onnx_model_path;
    std::string _ocr_recognize_onnx_model_path;
    std::string _dict_path;
};
}