#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "bamboo/base/types.h"

namespace welkin::bamboo { 
struct BevPixel {
    float diff_min_z;
    float diff_max_z;
    uint8_t r, g, b;  
    uint8_t intensity;  

    BevPixel()
        : diff_min_z(std::numeric_limits<float>::max()),
          diff_max_z(std::numeric_limits<float>::lowest()),
          r(0), g(0), b(0), intensity(0) {}
};
class Project;
// 数据预处理
class ParklotFileSystem;
class DataPreProcessor {
public:
    using Ptr = std::shared_ptr<DataPreProcessor>;
    DataPreProcessor(ParklotFileSystem* fs);
    virtual ~DataPreProcessor();
    void setProject(Project* proj);

    bool process();

    void bevTransform(const PointCloudPtr& cloud, 
        double min_x, double max_x, double min_y, double max_y);
    void splitImage(const cv::Mat& image, int window_size = 512, int step_size = 256);
    cv::Mat findNearestPixel(const cv::Mat& img, const cv::Mat& mask, int max_dist = 2);

private:
    Project* _project = nullptr;
    const float _resolution = 0.02;         // 网格分辨率
    const float _height_threshold = 1.5;   // 高度差阈值
    ParklotFileSystem* _file_system = nullptr;
};

}