#include "piecewise_line_fit.h"
#include <Eigen/Eigen>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <common/geometry/line2.h>
namespace welkin::bamboo {
// 点投影是否在线上
bool IsProjectInLine(const common::Point2d& pt, const common::Line2d& line) {
    common::Point2d dir0 = line.getDirection();
    common::Point2d dir1 = (pt - line.begin_point).normalized();
    common::Point2d dir2 = (pt - line.end_point).normalized();
    return dir0.dot(dir1) >= 0.0 && dir0.dot(dir2) <= 0.0;
}
double Distance(const common::Point2d& pt, const common::Line2d& line) {
    common::Point2d dir0 = line.getDirection();
    // 求直线垂直向量
    common::Point2d normal = common::Point2d(dir0.y, -dir0.x);
    common::Point2d dir1 = pt - line.begin_point;
    return dir1.dot(normal);
}
struct PointToLineError {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointToLineError(Eigen::Vector2d p_) : p(p_){}
    template <typename T>
    bool operator()(const T* begin, const T* end, T* residual) const {
        Eigen::Vector2<T> q1; q1 << begin[0], begin[1];
        Eigen::Vector2<T> q2; q2 << end[0], end[1];
        Eigen::Vector2<T> d1 = (q2 - q1).normalized();
        Eigen::Vector2<T> normal; normal << d1[1], -d1[0];
        Eigen::Vector2<T> d2 = p.template cast<T>() - q1;
        residual[0] = d2.dot(normal);
        return true;
    }
protected:
    Eigen::Vector2d p;
};

struct PointDiffError {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointDiffError(Eigen::Vector2d p_) : p(p_){}
    template <typename T>
    bool operator()(const T* q, T* residual) const {
        Eigen::Vector2<T> q1; q1 << q[0], q[1];
        Eigen::Vector2<T> d = q1 - p.template cast<T>();
        residual[0] = d[0];
        residual[1] = d[1];
        return true;
    }
protected:
    Eigen::Vector2d p;
};
void PiecewiseLineFit(const PointCloud::Ptr& cloud, std::vector<common::Point2d>& points) {
    auto points_copy = points;
    int iteration_nums = 5;
    std::vector<double> coord; coord.resize(points.size() * 2);
    for (int i = 0; i < points.size(); ++i) {
        coord[2 * i] = points[i].x;
        coord[2 * i + 1] = points[i].y;
    }
    double delta = 0.2;
    for (int i = 0; i < iteration_nums; ++i) {
        // 构建误差方程
        int valid_nums = 0;
        ceres::Problem problem;
        auto point_count = points.size();
        for (auto& pt : cloud->points) {
            common::Point2d p(pt.x, pt.y);
            Eigen::Vector2d pe; pe << p.x, p.y;
            double min_dist = std::numeric_limits<double>::max();
            int best_index = -1;
            for (int j = 0; j < point_count - 1; ++j) {
                auto dir = common::Line2d(points.at(j), points.at(j + 1)).getDirection();
                auto q1 = points.at(j) - common::Point2d(delta * dir.x, delta * dir.y);
                auto q2 = points.at(j + 1) + common::Point2d(delta * dir.x, delta * dir.y);
                if (!IsProjectInLine(p, common::Line2d(q1, q2))) {continue;}
                double dist = std::fabs(Distance(p, common::Line2d(q1, q2)));
                // if (!IsProjectInLine(p, common::Line2d(points.at(j), points.at(j + 1)))) {continue;}
                // double dist = std::fabs(Distance(p, common::Line2d(points.at(j), points.at(j + 1))));
                if (dist < min_dist) {
                    best_index = j;
                    min_dist = dist;
                }
            }
            if (best_index != -1 && min_dist < 0.2) { // 距离小于20cm，才参与
                ceres::CostFunction* pCostFunction = 
                    new ceres::AutoDiffCostFunction<PointToLineError, 1, 2, 2>(new PointToLineError(pe));
                auto q1 = &(coord.data()[2 * best_index]);
                auto q2 = &(coord.data()[2 * (best_index + 1)]);
                problem.AddResidualBlock(pCostFunction, nullptr, q1, q2);
                ++valid_nums;
            }
        }
        for (int j = 0; j < point_count; ++j) {
            Eigen::Vector2d p(points_copy.at(j).x, points_copy.at(j).y);
            ceres::CostFunction* pCostFunction = 
                    new ceres::AutoDiffCostFunction<PointDiffError, 2, 2>(new PointDiffError(p));
            auto q = &(coord.data()[2 * j]);
            problem.AddResidualBlock(pCostFunction, nullptr, q);
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        for (int j = 0; j < points.size(); ++j) {
            points[j].x = coord[2 * j];
            points[j].y = coord[2 * j + 1];
        }
    }
}
}