#include "pole.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/ransac.h>

namespace welkin::bamboo {
bool GetGroundHeight(const PointCloudPtr cloud_in, double& height, 
        int min_point_nums, double res_z) {
    std::map<int, int> point_num_map;
    for (auto& pt : cloud_in->points) {
        auto z_index = static_cast<int>(pt.z / res_z);
        if (point_num_map.find(z_index) == point_num_map.end()) {
            point_num_map[z_index] = 0;
        } else {
            point_num_map[z_index]++;
        }
    }
    for (auto& elem : point_num_map) {
        if (elem.second >= min_point_nums) {
            height = static_cast<double>(elem.first * res_z);
            return true;
        }
    }
    height = 0.0;
    return false;
}

struct PoleCell {
    std::set<int> z_sets;
};
class PoleGrid : public common::Ortho2d {
public:
    PoleGrid() {}
    virtual ~PoleGrid() {}
    bool init(double origin_x, double origin_y, 
            double res_x, double res_y, int width, int height) {
        this->setOrigin(origin_x, origin_y);
        this->setRes(res_x, res_y);
        this->setSize(width, height);
        _cells.resize(width * height);
        return true;
    }
    PoleCell& getCell(int index) {
        return _cells[index];
    }
    PoleCell& getCell(int row, int col) {
        return getCell(getIndex(row, col));
    }
private:
    std::vector<PoleCell> _cells;
};
void FilterPoleCloud(
        const PointCloudPtr cloud_in, PointCloudPtr& cloud_out,
        int min_cell_nums, int max_cell_nums,
        double res_x, double res_y, double res_z) {
    PointX min_pt, max_pt;
    pcl::getMinMax3D(*cloud_in, min_pt, max_pt);
    double origin_x = min_pt.x; double origin_y = min_pt.y;
    int width = std::ceil((max_pt.x - min_pt.x) / res_x);
    int height = std::ceil((max_pt.y - min_pt.y) / res_y);
    PoleGrid grid;
    grid.init(origin_x, origin_y, res_x, res_y, width, height);
    for (auto& pt : cloud_in->points) {
        auto col = grid.getCol(pt.x);
        auto row = grid.getRow(pt.y);
        if (grid.isIn(row, col)) {
            auto& cell = grid.getCell(row, col);
            cell.z_sets.insert(static_cast<int>(pt.z / res_z));
        }
    }
    int count = 0;
    cloud_out->clear();
    for (auto& pt : cloud_in->points) {
        auto col = grid.getCol(pt.x);
        auto row = grid.getRow(pt.y);
        if (grid.isIn(row, col)) {
            auto& cell = grid.getCell(row, col);
            count = cell.z_sets.size();
            if (count >= min_cell_nums && count <= max_cell_nums) {
                cloud_out->push_back(pt);
            }
        }
    }
}

bool FitPoleCloud(
        const PointCloudPtr cloud_in, common::Circled& circle,
        const common::Ranged& radius_limits, double scale, 
        double min_inlier_ratio, double sac_dist, int max_iter) {
    PointCloudPtr cloud_tmp(new PointCloud());
    for (auto pt : cloud_in->points) {
        pt.x *= scale; pt.y *= scale;
        cloud_tmp->push_back(pt);
    }
    pcl::SampleConsensusModelCircle2D<PointX>::Ptr circle_model(
        new pcl::SampleConsensusModelCircle2D<PointX>(cloud_tmp));
    double min_radius = radius_limits.min_v * scale;
    double max_radius = radius_limits.max_v * scale;
    circle_model->setRadiusLimits(min_radius, max_radius);
    pcl::RandomSampleConsensus<PointX> ransac(circle_model);
    ransac.setMaxIterations(max_iter);
    ransac.setDistanceThreshold(sac_dist * scale);
    if (!ransac.computeModel()) {return false;}

    // 解算
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    ransac.getInliers(inliers->indices);
    auto ratio = inliers->indices.size() * 1.0 / cloud_tmp->size();
    if (ratio < min_inlier_ratio) {return false;}

    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);
    // 优化
    Eigen::VectorXf coeff_refined(circle_model->getModelSize ());
    circle_model->optimizeModelCoefficients(inliers->indices, coeff, coeff_refined);
    coefficients->values.resize(coeff_refined.size ());
    memcpy(&coefficients->values[0], &coeff_refined[0], coeff_refined.size () * sizeof (float));
    coefficients->values[0] /= scale;
    coefficients->values[1] /= scale;
    coefficients->values[2] /= scale;
    circle.cnt_pt.x = coefficients->values[0];
    circle.cnt_pt.y = coefficients->values[1];
    circle.range_radius.min_v = 0.0;
    circle.range_radius.max_v = coefficients->values[2];
    return true;
}
}