#pragma once
#include <common/geometry/line2.h>
#include "bamboo/base/types.h"
#include <fstream>

namespace welkin::bamboo {
struct Transform {
    float ox = 0.f;
    float oy = 0.f;
    float oa = 0.f;
    float sinth = 1.0;
    float costh = 0.0;
    float score = 0.f;
    int point_nums = 0;
    float avg_score = 0.f;
    void computeScore(const PointCloudPtr& cloud, double line_length);
    Eigen::Matrix3d toMatrix33() const {
        return (Eigen::Matrix3d() <<
            costh, -sinth, ox,
            sinth,  costh, oy,
            0, 0, 1).finished();
    }

private:
    static float line_width;
};

struct TransformCorner {
    float ox = 0.f;  // 
    float oy = 0.f;  // 
    float score = 0.f;  
    int point_count = 0;
    void computeScore(const PointCloudPtr& cloud, bool rotated_dir1_positive_x, bool rotated_dir2_positive_y); 
    void computeMultiScore(const PointCloudPtr& cloud, bool rotated_dir1_positive_x, bool rotated_dir2_positive_y); 
    void computeLScore(const PointCloudPtr& cloud, bool rotated_dir1_positive_x, bool rotated_dir2_positive_y); 
    void computeTScore(const PointCloudPtr& cloud, bool rotated_dir1_positive_x, bool rotated_dir2_positive_y, int move_direction_flag);  
};

// 库位线优化器
class ParklineOptimizer {
public:
    ParklineOptimizer();
    virtual ~ParklineOptimizer();
    // // 优化
    void optimate(const common::Line2d& src_line, 
        const PointCloudPtr& cloud, common::Line2d& tgt_line, 
        bool& need_check); // neek_check需要检核
    void optimate2(const common::Line2d& src_line, 
        const PointCloudPtr& cloud, common::Line2d& tgt_line, 
        bool& need_check); // neek_check需要检核
    void optimate3(const common::Line2d& src_line, 
        const PointCloudPtr& cloud, common::Line2d& tgt_line, 
        bool& need_check); // neek_check需要检核
    void optimate4(const PointCloudPtr& cloud, const common::Point2d& original_corner, 
        common::Point2d& optimized_corner, const Eigen::Vector2f& direction1, 
        const Eigen::Vector2f& direction2, bool& need_check);
    void optimate4_2(const PointCloudPtr& cloud, const common::Point2d& original_corner, 
        common::Point2d& optimized_corner, const Eigen::Vector2f& direction1, 
        const Eigen::Vector2f& direction2, bool& need_check, int corner_type, int move_direction_flag);
    float calKurtosis(const std::vector<float>& data, int peak_index, int num_points);
    float kurtosis(const std::vector<float>& data);    
    double computeSpearmanRatio(const std::vector<float>& scores);
    void intensity_Midfilter(const PointCloudPtr& cloud);
    void gammaCorrection(const PointCloudPtr& cloud, float gamma);
    std::vector<Eigen::Vector2f> generateWindowPointClouds(const PointCloudPtr& region, float window_size);
    std::vector<Eigen::Vector2f> extractBoundaryMidPoints(const PointCloudPtr& region, 
        const std::vector<Eigen::Vector2f>& window_centers, float window_size, float expansion, 
        float intensity_threshold);
    std::vector<Eigen::Vector2f> refineBoundaryPoints(const PointCloudPtr& region, 
        const std::vector<Eigen::Vector2f>& window_centers, float window_size, float expansion, 
        float intensity_threshold);
    std::vector<Eigen::Vector2f> fitBoundaryPoints(const PointCloudPtr& region, 
        const std::vector<Eigen::Vector2f>& window_centers, float window_size, float expansion, 
        float intensity_threshold);
    Eigen::Vector2f fitLineUsingMidPoints(const std::vector<Eigen::Vector2f>& mid_points);
    Eigen::Vector2f fitLineUsingRANSAC(const std::vector<Eigen::Vector2f>& mid_points, 
    const Eigen::Vector2f& initial_direction, double dist_threshold = 0.03, double angle_threshold = 5.0, int max_iter = 200);
private:
    std::vector<std::vector<Transform>> _trans_table;
    std::vector<std::vector<TransformCorner>> _xy_trans_table;
    std::ofstream _outfile;
};
}