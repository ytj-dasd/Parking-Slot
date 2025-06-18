#include "cloud_algorithm.h"
#include <common/base/dir.h>
#include <common/base/uuid.h>
#include <common/image/utils.h>
#include <common/geometry/circle.h>
#include "bamboo/base/converter.h"

namespace welkin::bamboo {
// 根据点云优化多线
void OptimatePolylineWithCloud(
        const PointCloud::Ptr cloud_in,
        std::vector<common::Point2d>& point_list,
        bool is_closed, double max_dist, double sac_dist) {
    std::size_t pt_num = point_list.size();
    if (pt_num < 2) {return;}
    // 变成线段
    int line_num = pt_num - 1;
    if (is_closed) {
        line_num = pt_num;
    }
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
    PointCloudPtr cloud_plane_ptr(new PointCloud());
    pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices());
    for (std::size_t i = 0; i < line_vec.size(); ++i) {
        CONTINUE_IF(count_vec[i] < 10)
        cloud_plane_ptr->clear();
        cloud_plane_ptr->reserve(count_vec[i]);
        for (std::size_t j = 0; j < index_vec.size(); ++j) {
            CONTINUE_IF(index_vec[j] != i)
            cloud_plane_ptr->emplace_back(cloud_in->points[j]);
        }
        float scale_ratio = 5.0; // 对点进行缩放，增加平面解算精度
        for (auto& pt : cloud_plane_ptr->points) {
            pt.x *= scale_ratio;
            pt.y *= scale_ratio;
            pt.z *= 2.0;
        }
        indices_ptr->indices.clear();
        common::Planed plane_out;
        common::point_cloud::PlaneSACSegmentation<PointX>(cloud_plane_ptr,
            *indices_ptr, plane_out, sac_dist * scale_ratio, 20);
        CONTINUE_IF(indices_ptr->indices.size() < 10)
        // std::cout << "cloud num: " << cloud_plane_ptr->size() << std::endl;
        // std::cout << "inlier num: " << indices_ptr->indices.size() << std::endl;
        est_points.clear();
        est_points.reserve(indices_ptr->indices.size());
        for (auto idx : indices_ptr->indices) {
            auto& pt = cloud_plane_ptr->points[idx];
            est_points.emplace_back(common::Point2d(pt.x, pt.y));
        }
        line_vec[i].estimate(est_points);
        line_vec[i].begin_point *= 1.0 / scale_ratio;
        line_vec[i].end_point *= 1.0 / scale_ratio;
    }
    est_points.clear();
    est_points.resize(pt_num);
    for (std::size_t i = 1; i < est_points.size() - 1; ++i) {
        est_points[i] = line_vec[i - 1].getCrossPoint(line_vec[i]);
    }
    if (is_closed) {
        est_points[0] = line_vec[0].getCrossPoint(line_vec[line_num - 1]);
        est_points[pt_num - 1] = line_vec[line_num - 2].getCrossPoint(line_vec[line_num - 1]);
    } else {
        est_points[0] = line_vec[0].getProjectPoint(point_list[0]);
        est_points[pt_num - 1] = line_vec[line_num - 1].getProjectPoint(point_list[pt_num - 1]);
    }
    //
    for (std::size_t i = 0; i < est_points.size(); ++i) {
        if (est_points[i].distanceTo(point_list[i]) < 1.0) {
            point_list[i] = est_points[i];
        }
    }
}
void OptimatePolylineWithCloud(
        const PointCloudPtr cloud_in,
        std::vector<common::Point3d>& point_list,
        bool is_closed, double max_dist, double sac_dist) {
    std::vector<common::Point2d> point2_list;
    point2_list.resize(point_list.size());
    for (std::size_t i = 0; i < point2_list.size(); ++i) {
        point2_list[i].x = point_list[i].x;
        point2_list[i].y = point_list[i].y;
    }
    OptimatePolylineWithCloud(cloud_in, point2_list,
        is_closed, max_dist, sac_dist);
    for (std::size_t i = 0; i < point2_list.size(); ++i) {
        point_list[i].x = point2_list[i].x;
        point_list[i].y = point2_list[i].y;
    }
}

bool ForegroundSegmentByRGB(
        const PointCloud::Ptr& cloud_in, PointCloud::Ptr& cloud_out,
        uint8_t r, uint8_t g, uint8_t b) {
    cloud_out->clear();
    for (auto& pt : cloud_in->points) {
        if (pt.r > r && pt.g > g && pt.b > b) {
            cloud_out->push_back(pt);
        }
    }
    return !cloud_out->empty();
}
}
