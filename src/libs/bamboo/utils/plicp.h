// point-to-line icp algorithm
#pragma once
#include <Eigen/Eigen>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "bamboo/base/types.h"
#include "bamboo/base/macro.h"
#include <common/geometry/pose2.h>
namespace welkin::bamboo {
// template <typename T>
// T distance(const Eigen::Matrix<T, 2, 1>& p,
//         const Eigen::Matrix<T, 2, 1>& q1, const Eigen::Matrix<T, 2, 1>& q2) {
//     Eigen::Matrix<T, 2, 1> normal = (q1 - q2).normalized();
//     Eigen::Matrix<T, 2, 1> pq = p - q1;
//     return pq.dot(normal);
// }

// 2维点到线
struct PointToLineError {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointToLineError(Eigen::Vector2d p_, Eigen::Vector2d q1_, Eigen::Vector2d q2_) 
            : p(p_), q(q1_) {
        Eigen::Vector2d l = (q2_ - q1_).normalized();
        normal = Eigen::Vector2d(l[1], -l[0]);
    }
    template <typename T>
    bool operator()(const T* pT, T* residual) const {
        T theta = pT[2]; // 第3个数表示旋转
        Eigen::Matrix<T, 2, 2> R = Eigen::Matrix<T, 2, 2>::Zero();
        R << ceres::cos(theta), -ceres::sin(theta),
             ceres::sin(theta),  ceres::cos(theta);
        Eigen::Map<const Eigen::Matrix<T, 2, 1>> t(pT); // 前两个数表示平移
        Eigen::Matrix<T, 2, 1> p1 = R * p.template cast<T>() + t;
        Eigen::Matrix<T, 2, 1> pq = p1 - q.template cast<T>();
        residual[0] = pq.dot(normal.template cast<T>());
        return true;
    }
protected:
    Eigen::Vector2d p;
    Eigen::Vector2d q;
    Eigen::Vector2d normal;
};

void BAMBOO_EXPORT PlIcp(const PointCloud::Ptr cloud_in, 
        const std::vector<common::Point2d>& polyline, 
        common::Pose2d& pose, bool closed = true);
}