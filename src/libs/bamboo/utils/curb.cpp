#include "curb.h"
#include <common/geometry/ortho2.h>
#include <pcl/common/common.h>

namespace welkin::bamboo {
struct CurbCell {
    std::set<int> z_sets;
};
class CurbGrid : public common::Ortho2d {
public:
    CurbGrid() {}
    virtual ~CurbGrid() {}
    bool init(double origin_x, double origin_y, 
            double res_x, double res_y, int width, int height) {
        this->setOrigin(origin_x, origin_y);
        this->setRes(res_x, res_y);
        this->setSize(width, height);
        _cells.resize(width * height);
        return true;
    }
    CurbCell& getCell(int index) {
        return _cells[index];
    }
    CurbCell& getCell(int row, int col) {
        return getCell(getIndex(row, col));
    }
private:
    std::vector<CurbCell> _cells;
};
void FilterCurbCloud(
        const PointCloudPtr cloud_in, PointCloudPtr& cloud_out,
        int min_cell_nums, int max_cell_nums) {
    PointX min_pt, max_pt;
    pcl::getMinMax3D(*cloud_in, min_pt, max_pt);
    double res_x = 0.02; double res_y = 0.02; double res_z = 0.02;
    double origin_x = min_pt.x; double origin_y = min_pt.y;
    int width = std::ceil((max_pt.x - min_pt.x) / res_x);
    int height = std::ceil((max_pt.y - min_pt.y) / res_y);
    CurbGrid grid;
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

// 做直线拟合
void FitCurbWithCloud(
        const PointCloudPtr cloud_in,
        std::vector<common::Point2d>& point_list,
        double max_dist, double sac_dist) {
    std::size_t pt_num = point_list.size();
    if (pt_num < 2) {return;}
    // 变成线段
    int line_num = pt_num - 1;
    std::vector<common::Line2d> line_vec(line_num);
    std::vector<common::Circled> circle_vec(line_num);
    // 半径用于快速排除局部点
    for (int i = 0; i < line_vec.size(); ++i) {
        line_vec[i].begin_point = point_list[i];
        line_vec[i].end_point = point_list[(i + 1) % pt_num];
        circle_vec[i].cnt_pt = line_vec[i].getCenter();
        circle_vec[i].range_radius.min_v = 0;
        circle_vec[i].range_radius.max_v = line_vec[i].getLength() * 0.5 - 0.05; //
    }

    std::vector<int> index_vec; // 每个点对应的线索引
    index_vec.resize(cloud_in->size(), -1);
    std::vector<int> count_vec; // 第条线上点的数量
    count_vec.resize(line_vec.size(), 0);
    for (std::size_t i = 0; i < index_vec.size(); ++i) {
        auto& pt = cloud_in->points[i];
        for (std::size_t j = 0; j < line_vec.size(); ++j) {
            CONTINUE_IF(!circle_vec[j].isIn(pt.x, pt.y))
            CONTINUE_IF(std::fabs(line_vec[j].signDistanceTo(pt.x, pt.y)) > max_dist)
            index_vec[i] = j;
            count_vec[j] += 1;
            break;
        }
    }
    // 估计点
    std::vector<common::Point2d> est_points;
    PointCloudPtr cloud_line_ptr(new PointCloud());
    pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices());
    for (std::size_t i = 0; i < line_vec.size(); ++i) {
        if (count_vec[i] < 10) {
            std::cout << "点数太少:" << i << " " << count_vec[i] << std::endl;
            continue;
        }
        cloud_line_ptr->clear();
        cloud_line_ptr->reserve(count_vec[i]);
        for (std::size_t j = 0; j < index_vec.size(); ++j) {
            if (index_vec[j] != i) {
                // std::cout << "序号不对!" << std::endl;
                continue;
            }
            cloud_line_ptr->emplace_back(cloud_in->points[j]);
        }
        float scale_ratio = 1.0; // 对点进行缩放，增加平面解算精度
        for (auto& pt : cloud_line_ptr->points) {
            pt.x *= scale_ratio;
            pt.y *= scale_ratio;
            pt.z *= 0.0;
        }
        indices_ptr->indices.clear();

        pcl::SACSegmentation<PointX> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        // 点到线的距离
        seg.setDistanceThreshold(sac_dist * scale_ratio);
        seg.setMaxIterations(200);
        seg.setInputCloud(cloud_line_ptr);
        pcl::ModelCoefficients line_coeff;
        seg.segment(*indices_ptr, line_coeff);

        if (indices_ptr->indices.size() < 10) {
            std::cout << "拟合失败！" << std::endl;
            continue;
        }
        // std::cout << "cloud num: " << cloud_plane_ptr->size() << std::endl;
        // std::cout << "inlier num: " << indices_ptr->indices.size() << std::endl;
        est_points.clear();
        est_points.reserve(indices_ptr->indices.size());
        for (auto idx : indices_ptr->indices) {
            auto& pt = cloud_line_ptr->points[idx];
            est_points.emplace_back(common::Point2d(pt.x, pt.y));
        }
        line_vec[i].begin_point = common::Point2d(line_coeff.values[0], line_coeff.values[1]);
        line_vec[i].end_point = line_vec[i].begin_point + common::Point2d(100.0 * line_coeff.values[3], 100.0 * line_coeff.values[4]);
    }
    est_points.clear();
    est_points.resize(pt_num);
    est_points[0] = line_vec[0].getProjectPoint(point_list[0]);
    // for (std::size_t i = 1; i < est_points.size() - 1; ++i) {
    //     est_points[i] = line_vec[i - 1].getCrossPoint(line_vec[i]);
    // }
    // est_points[0] = line_vec[0].getProjectPoint(point_list[0]);
    // est_points[pt_num - 1] = line_vec[line_num - 1].getProjectPoint(point_list[pt_num - 1]);
    // 使用垂足
    for (std::size_t i = 1; i < est_points.size(); ++i) {
        est_points[i] = line_vec[i - 1].getProjectPoint(point_list[i]);
    }
    for (std::size_t i = 0; i < est_points.size(); ++i) {
        point_list[i] = est_points[i];
    }
}
}