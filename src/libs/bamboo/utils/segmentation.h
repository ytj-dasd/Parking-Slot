#pragma once
#define PCL_NO_PRECOMPILE
#include <pcl/common/angles.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <common/geometry/line2.h>
#include <common/geometry/line3.h>

namespace welkin::bamboo {
// 三维直线RANSAC拟合
template <typename PointT>
bool Line3dSACSegmentation(
        const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
        pcl::PointIndices& indices_out, common::Line3d& line_out,
        double dist_threshold, int max_iter = 30, bool optimize = true) {
    typename pcl::SampleConsensusModelLine<PointT>::Ptr model(
        new pcl::SampleConsensusModelLine<PointT>(cloud_in));
    pcl::RandomSampleConsensus<PointT> ransac(model);
    ransac.setMaxIterations(max_iter);
    ransac.setDistanceThreshold(dist_threshold);
    // 解算失败！
    if (!ransac.computeModel()) {return false;}
    // 解算
    ransac.getInliers(indices_out.indices);
    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    if (optimize) { // 优化
        Eigen::VectorXf coeff_refined(model->getModelSize());
        model->optimizeModelCoefficients(indices_out.indices, coeff, coeff_refined);
        coefficients->values.resize(coeff_refined.size ());
        memcpy(&coefficients->values[0], &coeff_refined[0], coeff_refined.size () * sizeof (float));
    } else {
        memcpy(&coefficients->values[0], &coeff[0], coeff.size () * sizeof (float));
    }
    line_out.begin_point = common::Point3d(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    line_out.end_point = line_out.begin_point + common::Point3d(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
    return true;
}
// 二维直线RANSAC拟合
template <typename PointT>
bool Line2dSACSegmentation(
        const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
        pcl::PointIndices& indices_out, common::Line2d& line_out,
        double dist_threshold, int max_iter = 30, bool optimize = true) {
    typename pcl::SampleConsensusModelLine<PointT>::Ptr model(
        new pcl::SampleConsensusModelLine<PointT>(cloud_in));
    pcl::RandomSampleConsensus<PointT> ransac(model);
    ransac.setMaxIterations(max_iter);
    ransac.setDistanceThreshold(dist_threshold);
    // 解算失败！
    if (!ransac.computeModel()) {return false;}
    // 解算
    ransac.getInliers(indices_out.indices);
    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    if (optimize) { // 优化
        Eigen::VectorXf coeff_refined(model->getModelSize());
        model->optimizeModelCoefficients(indices_out.indices, coeff, coeff_refined);
        coefficients->values.resize(coeff_refined.size ());
        memcpy(&coefficients->values[0], &coeff_refined[0], coeff_refined.size () * sizeof (float));
    } else {
        memcpy(&coefficients->values[0], &coeff[0], coeff.size () * sizeof (float));
    }
    line_out.begin_point = common::Point2d(coefficients->values[0], coefficients->values[1]);
    line_out.end_point = line_out.begin_point + common::Point2d(coefficients->values[3], coefficients->values[4]);
    return true;
}

// 三维平行线RANSAC拟合
template <typename PointT>
bool ParallelLine3dSACSegmentation(
        const typename pcl::PointCloud<PointT>::Ptr& cloud_in, const common::Line3d& line_in,
        pcl::PointIndices& indices_out, common::Line3d& line_out,
        double dist_threshold, double angle_threshold = 5.0, 
        int max_iter = 30, bool optimize = true) {
    typename pcl::SampleConsensusModelParallelLine<PointT>::Ptr model(
        new pcl::SampleConsensusModelParallelLine<PointT>(cloud_in));
    // 设置方向约束及角度阈值
    auto dir = line_in.getDirection();
    model->setAxis(Eigen::Vector3f(dir.x, dir.y, dir.z));
    model->setEpsAngle(pcl::deg2rad(angle_threshold));

    pcl::RandomSampleConsensus<PointT> ransac(model);
    ransac.setMaxIterations(max_iter);
    ransac.setDistanceThreshold(dist_threshold);
    // 解算失败！
    if (!ransac.computeModel()) {return false;}

    // 解算
    ransac.getInliers(indices_out.indices);
    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    if (optimize) { // 优化
        Eigen::VectorXf coeff_refined(model->getModelSize());
        model->optimizeModelCoefficients(indices_out.indices, coeff, coeff_refined);
        coefficients->values.resize(coeff_refined.size ());
        memcpy(&coefficients->values[0], &coeff_refined[0], coeff_refined.size () * sizeof (float));
    } else {
        memcpy(&coefficients->values[0], &coeff[0], coeff.size () * sizeof (float));
    }
    line_out.begin_point = common::Point3d(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    line_out.end_point = line_out.begin_point + common::Point3d(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
    return true;
}
// 二维平行线RANSAC拟合
template <typename PointT>
bool ParallelLine2dSACSegmentation(
        const typename pcl::PointCloud<PointT>::Ptr& cloud_in, const common::Line2d& line_in,
        pcl::PointIndices& indices_out, common::Line2d& line_out,
        double dist_threshold, double angle_threshold = 5.0, 
        int max_iter = 30, bool optimize = true) {
    typename pcl::SampleConsensusModelParallelLine<PointT>::Ptr model(
        new pcl::SampleConsensusModelParallelLine<PointT>(cloud_in));
    // 设置方向约束及角度阈值
    auto dir = line_in.getDirection();
    model->setAxis(Eigen::Vector3f(dir.x, dir.y, 0.0));
    model->setEpsAngle(pcl::deg2rad(angle_threshold));
    pcl::RandomSampleConsensus<PointT> ransac(model);
    ransac.setMaxIterations(max_iter);
    ransac.setDistanceThreshold(dist_threshold);
    // 解算失败！
    if (!ransac.computeModel()) {return false;}
    // 解算
    ransac.getInliers(indices_out.indices);
    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    if (optimize) { // 优化
        Eigen::VectorXf coeff_refined(model->getModelSize());
        model->optimizeModelCoefficients(indices_out.indices, coeff, coeff_refined);
        coefficients->values.resize(coeff_refined.size ());
        memcpy(&coefficients->values[0], &coeff_refined[0], coeff_refined.size () * sizeof (float));
    } else {
        memcpy(&coefficients->values[0], &coeff[0], coeff.size () * sizeof (float));
    }
    line_out.begin_point = common::Point2d(coefficients->values[0], coefficients->values[1]);
    line_out.end_point = line_out.begin_point + common::Point2d(coefficients->values[3], coefficients->values[4]);
    return true;
}
}