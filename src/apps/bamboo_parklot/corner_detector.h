#pragma once
#include <vector>
#include <opencv2/core.hpp>
#include <memory>

namespace welkin::bamboo {
struct KeyPoint {
    int x;
    int y;
    float confidence;
};

class ParklotFileSystem;
class CornerDetector {
public:
    using Ptr = std::shared_ptr<CornerDetector>;
    CornerDetector(ParklotFileSystem* fs);
    CornerDetector(const std::string& inference_mode, ParklotFileSystem* fs);
    virtual ~CornerDetector();
    const std::string& getInferenceMode() const;
    void setInferenceMode(const std::string& mode);
    const std::string& getOnnxModelPath() const;
    void setOnnxModelPath(const std::string& model_path);

    bool detect();
private:
    std::vector<KeyPoint> nmsKeypoints(
        std::vector<KeyPoint>& keypoints, float distance_threshold = 20);
    void drawParkingSlot(
        cv::Mat& image, const std::vector<std::vector<float>>& points_pred,
        const std::pair<int, int>& patch_position, std::vector<KeyPoint>& keypoints);
    void drawFinalKeypoints(cv::Mat& image, const std::vector<KeyPoint>& keypoints);

private:
    std::string _inference_mode = "CPU";
    ParklotFileSystem* _file_system = nullptr;
    std::string _onnx_model_path;
};
}