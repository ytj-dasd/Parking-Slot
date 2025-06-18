#pragma once
#include <flann/flann.hpp>
#include "bamboo/base/types.h"
#include "bamboo/base/macro.h"
#include <pcl/common/angles.h>
#include <pcl/features/feature.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include "bamboo/utils/DBSCAN_kdtree.h"

namespace welkin::bamboo {
// 根据点云优化多线
void BAMBOO_EXPORT OptimatePolylineWithCloud(
        const PointCloud::Ptr cloud_in,
        std::vector<common::Point2d>& point_list,
        bool is_closed, double max_dist, double sac_dist);
void BAMBOO_EXPORT OptimatePolylineWithCloud(
        const PointCloudPtr cloud_in,
        std::vector<common::Point3d>& point_list,
        bool is_closed, double max_dist, double sac_dist);

// 点云抽取
template <typename PointT>
bool ExtractCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
        typename pcl::PointCloud<PointT>::Ptr& cloud_out,
        const pcl::PointIndices::Ptr& inliers, bool negative = false) {
    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers
    extract.setInputCloud (cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(negative);
    extract.filter(*cloud_out);
    return !cloud_out->empty();
}
template <typename PointT>
bool ExtractCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
        typename pcl::PointCloud<PointT>::Ptr& cloud_out,
        const std::vector<pcl::PointIndices>& cluster_indices) {
    cloud_out->clear();
    // std::vector<pcl::PointIndices>::const_iterator
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
        // std::vector<int>::const_iterator
        for(auto pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cloud_out->push_back(cloud_in->points[*pit]);
        }
    }
    return !cloud_out->empty();
}

// 地面提取
// SAC：采样一致性，也即是RANSAC
template<typename PointT>
bool GroundSACSegmentation(
        const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
        pcl::PointIndices& indices_out, common::Planed& plane_out,
        double dist_threshold, double angle_threshold, int max_iter = 30) {
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    typename pcl::SampleConsensusModelPerpendicularPlane<PointT>::Ptr model(
        new pcl::SampleConsensusModelPerpendicularPlane<PointT>(cloud_in));
    model->setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
    model->setEpsAngle(pcl::deg2rad(angle_threshold));
    pcl::RandomSampleConsensus<PointT> ransac(model);
    ransac.setMaxIterations(max_iter);
    ransac.setDistanceThreshold(dist_threshold);
    if (!ransac.computeModel()) {return false;}
    // 解算
    ransac.getInliers(indices_out.indices);
    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);
    // 优化
    Eigen::VectorXf coeff_refined(model->getModelSize());
    model->optimizeModelCoefficients(indices_out.indices, coeff, coeff_refined);
    coefficients->values.resize(coeff_refined.size ());
    memcpy(&coefficients->values[0], &coeff_refined[0], coeff_refined.size () * sizeof (float));

    plane_out.a = coefficients->values[0];
    plane_out.b = coefficients->values[1];
    plane_out.c = coefficients->values[2];
    plane_out.d = coefficients->values[3];
    plane_out.normalize();
    return !indices_out.indices.empty();
}

// 边界提取
template<typename PointT>
bool ExtractBoundaries(
        const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
        typename pcl::PointCloud<PointT>::Ptr& cloud_boundaries,
        int search_k = 50, double angle_threshold = 108.0) {
    // 计算法线
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_in);
    ne.setKSearch(search_k);
    ne.compute(*cloud_normals);

    // 计算边界
    pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>); //声明一个boundary类指针，作为返回值
    boundaries->resize(cloud_in->size()); //初始化大小
    pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary> boundary_estimation; //声明一个BoundaryEstimation类
    boundary_estimation.setInputCloud(cloud_in); //设置输入点云
    boundary_estimation.setInputNormals(cloud_normals); //设置输入法线
    typename pcl::search::KdTree<PointT>::Ptr kdtree_ptr(new pcl::search::KdTree<PointT>); 
    boundary_estimation.setSearchMethod(kdtree_ptr); //设置搜寻k近邻的方式
    boundary_estimation.setKSearch(search_k); //设置k近邻数量
    //设置角度阈值，大于阈值为边界
    boundary_estimation.setAngleThreshold(pcl::deg2rad(angle_threshold));
    boundary_estimation.compute(*boundaries); //计算点云边界，结果保存在boundaries中

    for(size_t i = 0; i < cloud_in->size(); i++) {
        if(boundaries->points[i].boundary_point != 0) {
            PointT pt = cloud_in->points[i];
            cloud_boundaries->push_back(pt);
        }
    }
    return !cloud_boundaries->empty();
}

// DBScan聚类
template <typename PointT>
bool DBScanCluster(
        const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
        std::vector<pcl::PointIndices>& cluster_indices,
        int core_point_min_pts = 10, double cluster_tolerance = 0.15,
        int min_cluster_size = 100, int max_cluster_size = 100000) {
    // dbscan聚类
    typename pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    kdtree->setInputCloud(cloud_in);
    DBSCANKdtreeCluster<PointT> ec;
    ec.setCorePointMinPts(core_point_min_pts);
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(kdtree);
    ec.setInputCloud(cloud_in);
    ec.extract(cluster_indices);
    return !cluster_indices.empty();
}

// 二维平行线拟合
template <typename PointT>
bool FitParallelLine2d(
        const typename pcl::PointCloud<PointT>::Ptr& cloud_in, const common::Line2d& init_line,
        pcl::PointIndices& indices_out, common::Line2d& res_line,
        double dist_threshold = 0.02, double angle_threshold = 5.0, int max_iter = 500) {
    // 解算第一条线
    typename pcl::SampleConsensusModelParallelLine<PointT>::Ptr model(
        new pcl::SampleConsensusModelParallelLine<PointT>(cloud_in));
    auto dir = init_line.getDirection();
    // 约束线方向
    model->setAxis(Eigen::Vector3f(dir.x, dir.y, 0.0));
    model->setEpsAngle(pcl::deg2rad(angle_threshold));
    pcl::RandomSampleConsensus<PointT> ransac(model);
    ransac.setMaxIterations(max_iter);
    ransac.setDistanceThreshold(dist_threshold);
    if (!ransac.computeModel()) {return false;}
    ransac.getInliers(indices_out.indices);
    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);
    // 优化
    Eigen::VectorXf coeff_refined(model->getModelSize());
    model->optimizeModelCoefficients(indices_out.indices, coeff, coeff_refined);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    coefficients->values.resize(coeff_refined.size ());
    memcpy(&coefficients->values[0], &coeff_refined[0], coeff_refined.size () * sizeof (float));

    res_line.begin_point = common::Point2d(coefficients->values[0], coefficients->values[1]);
    res_line.end_point = res_line.begin_point + common::Point2d(coefficients->values[3], coefficients->values[4]);
    return true;
}

// 根据RGB进行前背景分割
bool BAMBOO_EXPORT ForegroundSegmentByRGB(
        const PointCloud::Ptr& cloud_in, PointCloud::Ptr& cloud_out,
        uint8_t r = 128, uint8_t g = 128, uint8_t b = 128);
}
