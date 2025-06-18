#include "bamboo/utils/cloud_algorithm.h"
#include "park_lot.h"

namespace welkin::bamboo {
// 停车位分割线拟合
bool FitParkLotLine(
        const PointCloud::Ptr& cloud_in, const common::Line2d& init_line,
        common::Line2d& line1, common::Line2d& line2) {
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // 解算第一条线
    // 5.0度的容差
    if (!FitParallelLine2d<PointX>(cloud_in, init_line, *inliers, line1, 0.02, 5.0, 500)) {
        return false;
    }
    // 获取剩余点
    PointCloud::Ptr cloud_negative(new PointCloud());
    ExtractCloud<PointX>(cloud_in, cloud_negative, inliers, true);

    // 保留与第一条线0.15m左右的点
    PointCloud::Ptr cloud_left(new PointCloud());
    for (auto pt : cloud_negative->points) {
        auto dist = line1.distanceTo(common::Point2d(pt.x, pt.y));
        if (dist > 0.05 && dist < 0.25) {
            cloud_left->push_back(pt);
        }
    }
    if (cloud_left->empty()) {return false;}
    // 解算第二条线
    // 0.1度容差
    return FitParallelLine2d<PointX>(cloud_left, line1, *inliers, line2, 0.02, 0.1, 500);
}
bool FitParkLot(
        const PointCloud::Ptr& cloud_in, const common::Polygon2d& border,
        std::vector<common::Polygon2d>& lots) {
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    common::Planed plane;
    // 地面拟合
    if (!GroundSACSegmentation<PointX>(cloud_in, *inliers, plane, 0.1, 5.0, 500)) {
        return false;
    }
    PointCloud::Ptr cloud_ground(new PointCloud());
    ExtractCloud<PointX>(cloud_in, cloud_ground, inliers, false);
    PointCloud::Ptr cloud_park_lot(new PointCloud());
    // 前背景分割
    if (!ForegroundSegmentByRGB(cloud_ground, cloud_park_lot, 128, 128, 128)) {
        return false;
    }
    // DBScan聚类
    std::vector<pcl::PointIndices> cluster_indices;
    if (!DBScanCluster<PointX>(cloud_park_lot, cluster_indices, 10, 0.15, 100, 2500000)) {
        return false;
    }
    PointCloud::Ptr cloud_dbscan(new PointCloud());
    if (!ExtractCloud<PointX>(cloud_park_lot, cloud_dbscan, cluster_indices)) {
        return false;
    }
    // 边界提取
    PointCloud::Ptr cloud_boundaries(new PointCloud());
    if (!ExtractBoundaries<PointX>(cloud_dbscan, cloud_boundaries, 50, 108)) {
        return false;
    }
    // 标准库位参数
    double lot_length = 5.0; double lot_width = 2.5;

    common::Line2d top_line(border.points[0], border.points[3]);
    common::Line2d bot_line(border.points[1], border.points[2]);
    common::Line2d left_line(border.points[0], border.points[1]);
    common::Line2d right_line(border.points[3], border.points[2]);
    double length = top_line.getLength();
    // 求库位数量
    double cell_length = left_line.getLength() < (lot_length - 1.0) ? lot_length : lot_width;
    int nums = std::round(length / cell_length);
    // 带正负的距离
    double sign_dist = left_line.signDistanceTo(border.points[2]);
    double cell_dist = sign_dist / nums;
    double delta = 0.5;
    std::vector<common::Line2d> lines;
    // 求车库分割线
    for (int i = 1; i < nums; ++i) {
        // 求偏移的线
        auto init_line = left_line.getParallelLine(cell_dist * i);
        // 获取合适点云
        PointCloud::Ptr cloud_filter(new PointCloud());
        for (auto pt : cloud_boundaries->points) {
            if (init_line.distanceTo(common::Point2d(pt.x, pt.y)) < delta) {
                cloud_filter->push_back(pt);
            }
        }
        // 拟合线
        common::Line2d line1, line2;
        if (!FitParkLotLine(cloud_filter, init_line, line1, line2)) {continue;}
        line1.begin_point = line1.getCrossPoint(top_line);
        line1.end_point = line1.getCrossPoint(bot_line);
        line2.begin_point = line2.getCrossPoint(top_line);
        line2.end_point = line2.getCrossPoint(bot_line);
        common::Line2d center_line;
        center_line.begin_point = (line1.begin_point + line2.begin_point) * 0.5;
        center_line.end_point = (line1.end_point + line2.end_point) * 0.5;
        lines.push_back(center_line);
    }
    lines.insert(lines.begin(), left_line);
    lines.push_back(right_line);

    lots.clear();
    for (int i = 0; i < lines.size() - 1; ++i) {
        auto line1 = lines.at(i);
        auto line2 = lines.at(i + 1);
        common::Polygon2d polygon;
        polygon.push_back(line1.begin_point);
        polygon.push_back(line1.end_point);
        polygon.push_back(line2.end_point);
        polygon.push_back(line2.begin_point);
        lots.push_back(polygon);
    }
    return !lots.empty();
}
}