#include "plicp.h"
namespace welkin::bamboo {
double deg2rad(double x) {
    return x * M_PI / 180;
}
double rad2deg(double x) {
    return x * 180 / M_PI;
}
double distance(const Eigen::Vector2d& p, 
        const Eigen::Vector2d& q1, const Eigen::Vector2d& q2) {
    Eigen::Vector2d l = (q1 - q2).normalized();
    // l的垂直向量
    Eigen::Vector2d normal = Eigen::Vector2d(l[1], -l[0]);
    Eigen::Vector2d pq = p - q1;
    return std::fabs(pq.dot(normal));
}

void PlIcp(const PointCloud::Ptr cloud_in, 
        const std::vector<common::Point2d>& polyline, 
        common::Pose2d& pose, bool closed) {
    // 设置初值
    double pT[3];
    pT[0] = 0.0; pT[1] = 0.0; pT[2] = 0.0;

    int iteration_nums = 10;
    for (int i = 0; i < iteration_nums; ++i) {
        // 构建旋转矩阵R和平移向量t
        Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
        double alpha = pT[2];
        R << cos(alpha), -sin(alpha), 
            sin(alpha),  cos(alpha);
        Eigen::Vector2d t = Eigen::Vector2d(pT[0], pT[1]);
        
        // 构建误差方程
        ceres::Problem problem;
        for (auto& pt : cloud_in->points) {
            Eigen::Vector2d p(pt.x, pt.y);
            Eigen::Vector2d p1 = R * p + t;
            double min_dist = std::numeric_limits<double>::max();
            Eigen::Vector2d best_q1, best_q2;
            auto point_count = polyline.size();
            Eigen::Vector2d q1, q2;
            for (int j = 0; j < point_count - 1; ++j) {
                q1 = Eigen::Vector2d(polyline.at(j).x, polyline.at(j).y);
                q2 = Eigen::Vector2d(polyline.at(j + 1).x, polyline.at(j + 1).y);
                double dist = distance(p1, q1, q2);
                if (dist < min_dist) {
                    best_q1 = q1;
                    best_q2 = q2;
                    min_dist = dist;
                }
            }
            if (closed) {
                q1 = Eigen::Vector2d(polyline.back().x, polyline.back().y);
                q2 = Eigen::Vector2d(polyline.at(0).x, polyline.at(0).y);
                double dist = distance(p1, q1, q2);
                if (dist < min_dist) {
                    best_q1 = q1;
                    best_q2 = q2;
                    min_dist = dist;
                }
            }
            if (min_dist < 0.2) { // 距离小于20cm，才参与
                ceres::CostFunction* pCostFunction = 
                    new ceres::AutoDiffCostFunction<PointToLineError, 1, 3>(
                        new PointToLineError(p, best_q1, best_q2));
                problem.AddResidualBlock(pCostFunction, nullptr, pT);
            }
        }
        // problem.SetParameterLowerBound(pT, 2, -1e-6);
        // problem.SetParameterUpperBound(pT, 2, 1e-6);
        // problem.SetParameterLowerBound(pT, 2, -0.02);
        // problem.SetParameterUpperBound(pT, 2, 0.02);

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        // // 每一个残差块的平均误差小于10e-8, 则认为已经收敛(不确定是收敛到局部最优解还是全局最优解)，可以结束迭代
        // if((std::fabs(summary.initial_cost - summary.final_cost) / summary.initial_cost) / summary.num_residual_blocks < 10e-8) {
        //     std::cout << "iterations = " << i << std::endl;
        //     break;
        // }
    }
    // 获取变换矩阵-点云到多线
    pose.x = pT[0];
    pose.y = pT[1];
    pose.theta = pT[2];
    std::cout << "theta = " << rad2deg(pT[2]) << ", t = [" << pT[0] << " " << pT[1] << "]" << std::endl;
}
}