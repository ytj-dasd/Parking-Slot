#include <pcl/common/transforms.h>
#include <common/geometry/grid1.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include "pcl/impl/pcl_base.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include "parkline_optimizer.h"
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
// #include <common/geometry/range_step.h>

namespace welkin::bamboo {
using PointXYZRGBI = common::point_cloud::PointXYZRGBI;
float expansion = 0.08f; // 候选区域宽度: 8cm(九亭)
float window_size = 0.1f;  // 滑动窗口步长: 10cm
float intensity_threshold = 10.0f;  // 反射强度阈值: 10(小昆山) 5（九亭）
float step_size = 0.01f; // 反射强度直方图步长: 10mm
float sd_factor = 50.0; // 标准差置信度缩放因子
float outlier_factor = 1; // 离群点占比缩放因子
float valid_window_factor = 1; // 有效窗口占比缩放因子
float fine_step = 0.002f;  // 1mm步长

double spearmanRatio(const std::vector<int>& ranks1, const std::vector<int>& ranks2) {
    if (ranks1.size() != ranks2.size()) {
        AERROR << "Two ranks has no same size!";
        return 0.0;
    }
    int n = ranks1.size();
    double sum_square_d = 0.0;
    for (int i = 0; i < ranks1.size(); ++i) {
        double d = ranks1.at(i) - ranks2.at(i);
        sum_square_d += d * d;
    }
    // 斯皮尔曼相关系数
    return 1 - 6 * sum_square_d / (n * (n * n - 1));
}
void Transform::computeScore(const PointCloudPtr& cloud, double line_length) {
    score = 0.f;
    point_nums = 0;
    float tx, ty;
    float w = 1.0;
    float l1 = line_width * 0.5;
    float l2 = l1 / 3.0; // l2 = 0.025
    // 采用w = a / (ty * ty + b) 的定权方式
    // ty < 0.025, w = 1.0
    // ty = 0.025, w = 1.0; ty = 0.075, w = 0.7
    
    float b = (0.7 * l1 * l1 - l2 * l2) / (1.0 - 0.7);
    float a = 1.0 * (l2 * l2 + b);
    for (const auto& pt : cloud->points) {
        tx = pt.x * costh - pt.y * sinth + ox;
        if (std::fabs(tx) > 0.5 * line_length - 0.2) {continue;}
        ty = pt.x * sinth + pt.y * costh + oy;
        if (std::fabs(ty) > l1) {continue;}
        if (std::fabs(ty) < l2) {
            w = 1.0;
        } else {
            w = a / (ty * ty + b);
            // w = 1.0;
        }
        ++point_nums;
        // score += pt.intensity * w;
        if (pt.intensity > 0.7) {
            score += pt.intensity * w * 1.5;
        } else if (pt.intensity < 0.3) {
            score += pt.intensity * w * 0.5;
        } else {
            score += pt.intensity * w;
        }
    }
    avg_score = score / point_nums;
}

float Transform::line_width = 0.15;


void TransformCorner::computeScore(const PointCloudPtr& cloud, bool rotated_dir1_positive_x, bool rotated_dir2_positive_y) {
    score = 0.f;
    point_count = 0;
    float tx, ty;
    float w = 1.0;
    float l1 = 0.15 * 0.5;  
    float l2 = l1 / 3.0;    
    
    // 采用 w = a / (ty * ty + b) 的定权方式
    float b = (0.7 * l1 * l1 - l2 * l2) / (1.0 - 0.7);
    float a = 1.0 * (l2 * l2 + b);

    for (const auto& pt : cloud->points) {
        tx = pt.x + ox;  
        ty = pt.y + oy;

        bool is_in_dir1_range = false;
        if (rotated_dir1_positive_x) {
            if (std::fabs(ty) < l1 && tx >= -l1 && tx <= 0.5) {
                is_in_dir1_range = true;
                if (std::fabs(ty) < l2) {
                    w = 1.0;
                } else {
                    w = a / (ty * ty + b);
                }
            }
        } else {
            if (std::fabs(ty) < l1 && tx >= -0.5 && tx <= l1) {
                is_in_dir1_range = true;
                if (std::fabs(ty) < l2) {
                    w = 1.0;
                } else {
                    w = a / (ty * ty + b);
                }
            }
        }

        bool is_in_dir2_range = false;
        if (rotated_dir2_positive_y && !is_in_dir1_range) {
            if (std::fabs(tx) < l1 && ty >= -l1 && ty <= 0.5) {
                is_in_dir2_range = true;
                if (std::fabs(tx) < l2) {
                    w = 1.0;  
                } else {
                    w = a / (tx * tx + b);
                }
            }
        } else if (!is_in_dir1_range) {
            if (std::fabs(tx) < l1 && ty >= -0.5 && ty <= l1) {
                is_in_dir2_range = true;
                if (std::fabs(tx) < l2) {
                    w = 1.0;  
                } else {
                    w = a / (tx * tx + b);
                }
            }
        }

        if (is_in_dir1_range || is_in_dir2_range) {
            ++point_count;
            if (pt.intensity > 0.7) {
                score += pt.intensity * w * 1.5;
            } else if (pt.intensity < 0.3) {
                score += pt.intensity * w * 0.5;
            } else {
                score += pt.intensity * w;
            }
        }
    }
}

void TransformCorner::computeMultiScore(const PointCloudPtr& cloud, bool rotated_dir1_positive_x, bool rotated_dir2_positive_y) {
    score = 0.f;
    point_count = 0;
    float tx, ty;
    float w = 1.0;
    float l1 = 0.6;  
    float l2 = 0.15;    
    
    // 采用 w = a / (ty * ty + b) 的定权方式
    float b = (0.8 * l1 * l1 - l2 * l2) / (1.0 - 0.8);
    float a = 1.0 * (l2 * l2 + b);

    for (const auto& pt : cloud->points) {
        tx = pt.x + ox;  
        ty = pt.y + oy;

        bool is_in_range = false;
        if (std::fabs(ty) <= l1 && std::fabs(tx) <= l2 * 0.5) {
            is_in_range = true;
            if (std::fabs(ty) <= l2 * 0.5) {
                w = 1.0;
            } else {
                w = a / (ty * ty + b);
            }
        } else if (std::fabs(ty) <= l2 * 0.5 && std::fabs(tx) <= l1) {
            is_in_range = true;
            w = a / (tx * tx + b);
        }
        

        if (is_in_range) {
            ++point_count;
            if (pt.intensity > 0.7) {
                score += pt.intensity * w * 1.5;
            } else if (pt.intensity < 0.3) {
                score += pt.intensity * w * 0.5;
            } else {
                score += pt.intensity * w;
            }
        }
    }
}

void TransformCorner::computeLScore(const PointCloudPtr& cloud, bool rotated_dir1_positive_x, bool rotated_dir2_positive_y) {
    score = 0.f;
    point_count = 0;
    float tx, ty;
    float w = 1.0;
    float l1 = 0.6;  
    float l2 = 0.15;    
    
    // 采用 w = a / (ty * ty + b) 的定权方式
    float b = (0.8 * l1 * l1 - l2 * l2) / (1.0 - 0.8);
    float a = 1.0 * (l2 * l2 + b);

    for (const auto& pt : cloud->points) {
        tx = pt.x + ox;  
        ty = pt.y + oy;

        bool is_in_range = false;
        if (rotated_dir1_positive_x) {
            if (rotated_dir2_positive_y) {    
                if (ty >= 0 && ty <= l2 && tx >= 0 && tx <= l1) {
                    is_in_range = true;
                    if (std::fabs(tx) <= l2) {
                        w = 1.0;
                    } else {
                        w = a / (tx * tx + b);
                    }
                } else if (ty >= l2 && ty <= l1 && tx >= 0 && tx <= l2){
                        is_in_range = true;
                        w = a / (ty * ty + b);
                }            
            } else {
                if (ty >= -l2 && ty <= 0 && tx >= 0 && tx <= l1) {
                    is_in_range = true;
                    if (std::fabs(tx) <= l2) {
                        w = 1.0;
                    } else {
                        w = a / (tx * tx + b);
                    }
                } else if (ty >= -l1 && ty <= -l2 && tx >= 0 && tx <= l2){
                        is_in_range = true;
                        w = a / (ty * ty + b);
                }
            }  
        } else {
            if (rotated_dir2_positive_y) {
                if (ty >= 0 && ty <= l2 && tx >= -l1 && tx <= 0) {
                    is_in_range = true;
                    if (std::fabs(tx) <= l2) {
                        w = 1.0;
                    } else {
                        w = a / (tx * tx + b);
                    }
                } else if (ty >= l2 && ty <= l1 && tx >= -l2 && tx <= 0){
                        is_in_range = true;
                        w = a / (ty * ty + b);
                } 
            } else {
                if (ty >= -l2 && ty <= 0 && tx >= -l1 && tx <= 0) {
                    is_in_range = true;
                    if (std::fabs(tx) <= l2) {
                        w = 1.0;
                    } else {
                        w = a / (tx * tx + b);
                    }
                } else if (ty >= -l1 && ty <= -l2 && tx >= -l2 && tx <= 0){
                        is_in_range = true;
                        w = a / (ty * ty + b);
                }
            }  
        }
        

        if (is_in_range) {
            ++point_count;
            if (pt.intensity > 0.7) {
                score += pt.intensity * w * 1.5;
            } else if (pt.intensity < 0.3) {
                score += pt.intensity * w * 0.5;
            } else {
                score += pt.intensity * w;
            }
        }
    }
}

void TransformCorner::computeTScore(const PointCloudPtr& cloud, bool rotated_dir1_positive_x, bool rotated_dir2_positive_y, int move_direction_flag) {
    score = 0.f;
    point_count = 0;
    float tx, ty;
    float w = 1.0;
    float l1 = 0.6;  
    float l2 = 0.15;    
    
    // 采用 w = a / (ty * ty + b) 的定权方式
    float b = (0.8 * l1 * l1 - l2 * l2) / (1.0 - 0.8);
    float a = 1.0 * (l2 * l2 + b);

    for (const auto& pt : cloud->points) {
        tx = pt.x + ox;  
        ty = pt.y + oy;

        bool is_in_range = false;
        if (move_direction_flag == 1) {
            if (rotated_dir1_positive_x) {
                if (std::fabs(ty) <= l1 && tx >= 0 && tx <= l2) {
                    is_in_range = true;
                    if (std::fabs(ty) <= l2 * 0.5) {
                        w = 1.0;
                    } else {
                        w = a / (ty * ty + b);
                    }
                } else if (std::fabs(ty) <= l2 * 0.5 && tx >= l2 && tx <= l1) {
                    is_in_range = true;
                    w = a / (tx * tx + b);
                }
            } else {
                if (std::fabs(ty) <= l1 && tx >= -l2 && tx <= 0) {
                    is_in_range = true;
                    if (std::fabs(ty) <= l2 * 0.5) {
                        w = 1.0;
                    } else {
                        w = a / (ty * ty + b);
                    }
                } else if (std::fabs(ty) <= l2 * 0.5 && tx >= -l1 && tx <= -l2) {
                    is_in_range = true;
                    w = a / (tx * tx + b);
                }
            }
        } else if (move_direction_flag == 2) {
            if (rotated_dir2_positive_y) {
                if (std::fabs(tx) <= l1 && ty >= 0 && ty <= l2) {
                    is_in_range = true;
                    if (std::fabs(tx) <= l2 * 0.5) {
                        w = 1.0;
                    } else {
                        w = a / (tx * tx + b);
                    }
                } else if (std::fabs(tx) <= l2 * 0.5 && ty >= l2 && ty <= l1) {
                    is_in_range = true;
                    w = a / (ty * ty + b);
                }
            } else {
                if (std::fabs(tx) <= l1 && ty >= -l2 && ty <= 0) {
                    is_in_range = true;
                    if (std::fabs(tx) <= l2 * 0.5) {
                        w = 1.0;
                    } else {
                        w = a / (tx * tx + b);
                    }
                } else if (std::fabs(tx) <= l2 * 0.5 && ty >= -l1 && ty <= -l2) {
                    is_in_range = true;
                    w = a / (ty * ty + b);
                }
            }
        }

        if (is_in_range) {
            ++point_count;
            if (pt.intensity > 0.7) {
                score += pt.intensity * w * 1.5;
            } else if (pt.intensity < 0.3) {
                score += pt.intensity * w * 0.5;
            } else {
                score += pt.intensity * w;
            }
        }
    }
}

/// ParklineOptimizer
ParklineOptimizer::ParklineOptimizer() {
    // auto tvec = common::RangeStepf(-3.5, 3.5, 0.1).getSampling();
    // auto yvec = common::RangeStepf(-0.05, 0.05, 0.001).getSampling();
    auto tvec = common::Grid1f(-3.5, 0.1, 71).getSampling();
    auto yvec = common::Grid1f(-0.05, 0.001, 101).getSampling();
    for (float ra : tvec) {
        std::vector<Transform> trans_list;
        for (float ry : yvec) {
            Transform item;
            item.ox = 0;
            item.oy = ry;
            item.oa = ra;
            item.sinth = std::sin(ra * M_PI / 180);
            item.costh = std::cos(ra * M_PI / 180);
            trans_list.push_back(item);
        }
        _trans_table.push_back(trans_list);
    }

    // auto x_values = common::RangeStepf(-0.05, 0.05, 0.001).getSampling();  // X 方向平移的范围
    // auto y_values = common::RangeStepf(-0.05, 0.05, 0.001).getSampling();  // Y 方向平移的范围
    auto x_values = common::Grid1f(-0.05, 0.001, 101).getSampling();  // X 方向平移的范围
    auto y_values = common::Grid1f(-0.05, 0.001, 101).getSampling();  // Y 方向平移的范围
    for (float tx : x_values) {
        std::vector<TransformCorner> xy_trans_list;
        for (float ty : y_values) {
            TransformCorner xy_item;
            xy_item.ox = tx;  
            xy_item.oy = ty;  
            xy_trans_list.push_back(xy_item);
        }
        _xy_trans_table.push_back(xy_trans_list);  
    }

    std::string stats_path = "/home/guitu/CLIP_data/stats.txt";
    _outfile.open(stats_path);
}

ParklineOptimizer::~ParklineOptimizer() {
    _outfile.close();
}

void ParklineOptimizer::intensity_Midfilter(const PointCloudPtr& cloud) {
    pcl::KdTreeFLANN<PointXYZRGBI> kdtree;
    kdtree.setInputCloud(cloud);
    int sum = 0;

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;
        std::vector<float> intensities;
        intensities.push_back(cloud->points[i].intensity);

        // 搜索y轴正方向的近邻点
        PointXYZRGBI searchPoint = cloud->points[i];
        if (kdtree.nearestKSearch(searchPoint, 200, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            int found_points = 0;
            for (int idx : pointIdxNKNSearch) {
                if (cloud->points[idx].y > searchPoint.y) {
                    intensities.push_back(cloud->points[idx].intensity);
                    found_points++;
                    if (found_points >= 2) break;
                }
            }
        }

        // 搜索y轴负方向的近邻点
        if (kdtree.nearestKSearch(searchPoint, 200, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            int found_points = 0;
            for (int idx : pointIdxNKNSearch) {
                if (cloud->points[idx].y < searchPoint.y) {
                    intensities.push_back(cloud->points[idx].intensity);
                    found_points++;
                    if (found_points >= 2) break;
                }
            }
        }

        if (intensities.size() < 5) {
            continue;
        }

        // 中值滤波
        std::sort(intensities.begin(), intensities.end());
        cloud->points[i].intensity = intensities[intensities.size() / 2];
        sum++;
    }
    std::cout << "\tfiltered proportion: " << sum * 100.0 / cloud->points.size() << std::endl;
}

void ParklineOptimizer::gammaCorrection(const PointCloudPtr& cloud, float gamma) {
    for (auto& point : cloud->points) {
        point.intensity = std::pow(point.intensity / 255.0f, gamma) * 255.0f;
    }
}

std::vector<Eigen::Vector2f> ParklineOptimizer::generateWindowPointClouds(const PointCloudPtr& region, float window_size) {
    std::vector<Eigen::Vector2f> window_centers;
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();

    // 找到区域的x方向边界
    for (const auto& point : region->points) {
        if (point.x < min_x) min_x = point.x;
        if (point.x > max_x) max_x = point.x;
    }

    min_x += 0.3;
    max_x -= 0.3;

    // 生成窗口中心点
    for (float x = min_x; x < max_x; x += window_size) {
        window_centers.push_back(Eigen::Vector2f(x + window_size / 2, 0));
    }

    return window_centers;
}

std::vector<Eigen::Vector2f> ParklineOptimizer::extractBoundaryMidPoints(const PointCloudPtr& region, 
    const std::vector<Eigen::Vector2f>& window_centers, float window_size, float expansion, 
    float intensity_threshold) {

    std::vector<Eigen::Vector2f> boundary_midpoints;
    int window_index = 0;

    for (const auto& center : window_centers) {
        // 创建窗口点云
       PointCloudPtr window_cloud(new PointCloud());
        
        // 提取窗口点云
        for (const auto& point : region->points) {
            if (std::abs(point.x - center.x()) <= window_size / 2) {
                window_cloud->points.emplace_back(point);
            }
        }
        // 保存每个窗口的点云
        // window_cloud->width = window_cloud->points.size();
        // window_cloud->height = 1; 
        // window_cloud->is_dense = true;
        // std::stringstream window_ss;
        // window_ss << save_dir << "edge_" << edge_index << "_window_" << window_index << ".pcd";
        // pcl::io::savePCDFile(window_ss.str(), *window_cloud);

        // 统计滑动窗口内的反射强度
        std::vector<float> y_positions;
        std::vector<float> intensities;

        for (float y = -expansion; y <= expansion; y += step_size) {
            float max_intensity = 0.0f;
            float min_intensity = 100.0f;
            float intensity_sum = 0.0f;
            int count = 0;

            // 计算每个正交方向上的点的反射强度
            for (const auto& point : window_cloud->points) {
                if (point.y >= y && point.y < y + step_size) {
                    intensity_sum += point.intensity;
                    count++;
                    if (point.intensity > max_intensity) {
                        max_intensity = point.intensity;
                    }
                    if (point.intensity < min_intensity) {
                        min_intensity = point.intensity;
                    }
                }
            }

            if (count > 0) {
                y_positions.push_back(y + step_size / 2); // 使用窗口中心点
                float avg_intensity = intensity_sum / count; // 平均反射强度

                float max_weight = 0.7f;  // 最大反射强度权重
                float avg_weight = 0.3f;  // 平均反射强度权重

                // 使用加权值存入 intensities
                intensities.push_back(max_weight * max_intensity + avg_weight * avg_intensity);
            }
        }

        // std::ofstream outfile;
        // std::stringstream data_ss;
        // data_ss << save_dir << "edge_" << edge_index << "_window_" << window_index << "_intensity.txt";
        // outfile.open(data_ss.str());
        // for (size_t i = 0; i < y_positions.size(); ++i) {
        //     outfile << y_positions[i] << " " << intensities[i] << "\n";
        // }

        // 记录反射强度突变点
        if (intensities.size() > 1) {
            std::vector<float> intensity_changes;
            for (size_t i = 1; i < intensities.size(); ++i) {
                intensity_changes.push_back(intensities[i] - intensities[i - 1]);
            }

            // 找到变化最大的突变点
            auto max_change_iter = std::max_element(intensity_changes.begin(), intensity_changes.end(), 
                [](float a, float b) { return std::abs(a) < std::abs(b); });
            size_t max_change_index = std::distance(intensity_changes.begin(), max_change_iter);

            if (std::abs(intensity_changes[max_change_index]) > intensity_threshold) {
                // 找到突变点前后两个区间强度更大的那个
                float boundary_y;
                if (intensities[max_change_index] > intensities[max_change_index + 1]) {
                    boundary_y = y_positions[max_change_index];
                } else {
                    boundary_y = y_positions[max_change_index + 1];
                }
                Eigen::Vector2f boundary_point(center.x(), boundary_y);
                boundary_midpoints.push_back(boundary_point);
                // outfile << "ChangePoint " << boundary_y << " " << intensities[max_change_index + 1] << "\n";
            }
        }
        // outfile.close();
        window_index++;
    }

    return boundary_midpoints;
}

std::vector<Eigen::Vector2f> ParklineOptimizer::refineBoundaryPoints(const PointCloudPtr& region, 
    const std::vector<Eigen::Vector2f>& window_centers, float window_size, float expansion, 
    float intensity_threshold) {

    std::vector<Eigen::Vector2f> boundary_midpoints;
    int window_index = 0;

    for (const auto& center : window_centers) {
        PointCloudPtr window_cloud(new PointCloud());

        for (const auto& point : region->points) {
            if (std::abs(point.x - center.x()) <= window_size / 2) {
                window_cloud->points.emplace_back(point);
            }
        }

        // 统计滑动窗口内的反射强度
        std::vector<float> y_positions;
        std::vector<float> intensities;

        for (float y = -expansion; y <= expansion; y += step_size) {
            float max_intensity = 0.0f;
            float min_intensity = 100.0f;
            float intensity_sum = 0.0f;
            int count = 0;

            // 计算每个正交方向上的点的反射强度
            for (const auto& point : window_cloud->points) {
                if (point.y >= y && point.y < y + step_size) {
                    intensity_sum += point.intensity;
                    count++;
                    if (point.intensity > max_intensity) {
                        max_intensity = point.intensity;
                    }
                    if (point.intensity < min_intensity) {
                        min_intensity = point.intensity;
                    }
                }
            }

            if (count > 0) {
                y_positions.push_back(y + step_size / 2); // 使用窗口中心点
                float avg_intensity = intensity_sum / count; // 平均反射强度

                float max_weight = 0.7f;  // 最大反射强度权重
                float avg_weight = 0.3f;  // 平均反射强度权重

                // 使用加权值存入 intensities
                intensities.push_back(max_weight * max_intensity + avg_weight * avg_intensity);
            }
        }

        // 记录反射强度突变点
        if (intensities.size() > 1) {
            std::vector<float> intensity_changes;
            for (size_t i = 1; i < intensities.size(); ++i) {
                intensity_changes.push_back(intensities[i] - intensities[i - 1]);
            }

            // 找到变化最大的突变点
            auto max_change_iter = std::max_element(intensity_changes.begin(), intensity_changes.end(), 
                [](float a, float b) { return std::abs(a) < std::abs(b); });
            size_t max_change_index = std::distance(intensity_changes.begin(), max_change_iter);

            if (std::abs(intensity_changes[max_change_index]) > intensity_threshold) {
                // 合并优化区间：将突变区间初值的前后两个区间合并为一个整体
                float coarse_boundary_start, coarse_boundary_end;
                coarse_boundary_start = y_positions[max_change_index] - 0.005f;
                coarse_boundary_end = y_positions[max_change_index + 1] + 0.005f;
                // if (intensities[max_change_index] > intensities[max_change_index + 1])
                // {
                //     coarse_boundary_start = y_positions[max_change_index] - 0.005f;  
                //     coarse_boundary_end = y_positions[max_change_index] + 0.005f;        
                // } else {
                //     coarse_boundary_start = y_positions[max_change_index + 1] - 0.005f;  
                //     coarse_boundary_end = y_positions[max_change_index + 1] + 0.005f;       
                // }
                

                // 细化分界点查找：在合并后的区间内以1mm为步长重新划分
                std::vector<float> fine_y_positions;
                std::vector<float> fine_intensities;

                for (float fine_y = coarse_boundary_start; fine_y <= coarse_boundary_end; fine_y += fine_step) {
                    float fine_intensity_sum = 0.0f;
                    float fine_max_intensity = 0.0f;
                    float fine_min_intensity = 100.0f;
                    int fine_count = 0;

                    // 统计1mm区间内的反射强度
                    for (const auto& point : window_cloud->points) {
                        if (point.y >= fine_y && point.y < fine_y + fine_step) {
                            fine_intensity_sum += point.intensity;
                            if (point.intensity > fine_max_intensity) {
                                fine_max_intensity = point.intensity;
                            }
                            if (point.intensity < fine_min_intensity) {
                                fine_min_intensity = point.intensity;
                            }
                            fine_count++;
                        }
                    }

                    if (fine_count > 0) {
                        fine_y_positions.push_back(fine_y + fine_step / 2);  // 使用窗口中心点
                        float fine_avg_intensity = fine_intensity_sum / fine_count;  // 平均反射强度
                        fine_intensities.push_back(0.7 * fine_max_intensity + 0.3 * fine_avg_intensity);
                        // fine_intensities.push_back(fine_avg_intensity);  // 存储每个窗口的平均强度
                    }
                }
                // std::cout << " fine_y_positions" << fine_y_positions.size() << std::endl;
                std::vector<float> fine_intensity_changes; 
                for (size_t i = 1; i < fine_y_positions.size(); ++i) {
                    fine_intensity_changes.push_back(fine_intensities[i] - fine_intensities[i - 1]);
                }
                // std::cout << " fine_intensity_changes" << fine_intensity_changes.size() << std::endl;
                auto fine_max_change_iter = std::max_element(fine_intensity_changes.begin(), fine_intensity_changes.end(), 
                [](float a, float b) { return std::abs(a) < std::abs(b); });
                size_t fine_max_change_index = std::distance(fine_intensity_changes.begin(), fine_max_change_iter);
                float refined_boundary_y;
                if (fine_intensities[fine_max_change_index] > fine_intensities[fine_max_change_index + 1]) {
                    refined_boundary_y = fine_y_positions[fine_max_change_index];
                } else {
                    refined_boundary_y = fine_y_positions[fine_max_change_index + 1];
                }

                // 保存细化后的分界点
                Eigen::Vector2f boundary_point(center.x(), refined_boundary_y);
                boundary_midpoints.push_back(boundary_point);
                // std::cout << "coarse boundary start " << coarse_boundary_start << " coarse boundary end " << coarse_boundary_end << " refine_boundary_y " << refined_boundary_y << std::endl;
            }
        }

        window_index++;
    }

    return boundary_midpoints;
}

Eigen::Vector2f ParklineOptimizer::fitLineUsingMidPoints(const std::vector<Eigen::Vector2f>& mid_points) {
    PointCloudPtr cloud(new PointCloud());
    for (const auto& mp : mid_points) {
        cloud->points.emplace_back(mp.x(), mp.y(), 0); 
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);

    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Vector3f eigen_vector = eigen_solver.eigenvectors().col(2);

    // 仅返回x和y方向的向量
    return eigen_vector.head<2>().normalized();
}

Eigen::Vector2f ParklineOptimizer::fitLineUsingRANSAC(const std::vector<Eigen::Vector2f>& mid_points, 
     const Eigen::Vector2f& initial_direction, double dist_threshold, double angle_threshold, int max_iter) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& mp : mid_points) {
        cloud->points.emplace_back(mp.x(), mp.y(), 0);  // 仅考虑平面坐标
    }

    // pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_line(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));
    pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>::Ptr model_line(
        new pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>(cloud));
    Eigen::Vector3f direction3D(initial_direction.x(), initial_direction.y(), 0.0f);
    model_line->setAxis(direction3D);
    model_line->setEpsAngle(pcl::deg2rad(angle_threshold)); // 设置方向约束及角度阈值

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_line);
    ransac.setMaxIterations(max_iter);
    ransac.setDistanceThreshold(dist_threshold);  // 设置距离阈值
    ransac.computeModel();

    pcl::PointIndices inliers;
    ransac.getInliers(inliers.indices);

    Eigen::VectorXf model_coefficients;
    ransac.getModelCoefficients(model_coefficients);
    if (model_coefficients.size() == 0) {
        std::cerr << "RANSAC failed to find a fitting line." << std::endl;
        return Eigen::Vector2f::Zero();
    }

    // 优化
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    Eigen::VectorXf coeff_refined(model_line->getModelSize());
    model_line->optimizeModelCoefficients(inliers.indices, model_coefficients, coeff_refined);
    coefficients->values.resize(coeff_refined.size());
    memcpy(&coefficients->values[0], &coeff_refined[0], coeff_refined.size () * sizeof (float));

    // 返回拟合直线的方向向量
    Eigen::Vector2f direction(coefficients->values[3], coefficients->values[4]);
    return direction.normalized();
}

double ParklineOptimizer::computeSpearmanRatio(const std::vector<float>& scores) {
    std::vector<std::pair<int, float>> index_score_vec;
    for (int i = 0; i < scores.size(); ++i) {
        index_score_vec.push_back(std::make_pair(i, scores[i]));
    }
    std::sort(index_score_vec.begin(), index_score_vec.end(), 
    [](const std::pair<int, float>& lhs, const std::pair<int, float>& rhs) {
        return lhs.second > rhs.second;
    });
    std::vector<int> xranks, yranks;
    xranks.resize(index_score_vec.size());
    yranks.resize(index_score_vec.size());
    for (int i = 0; i < index_score_vec.size(); ++i) {
        xranks[i] = index_score_vec.at(i).first;
        yranks[i] = i;
    }
    return spearmanRatio(xranks, yranks);
}

float ParklineOptimizer::calKurtosis(const std::vector<float>& data, int peak_index, int num_points) {
    int left = std::max(0, peak_index - num_points);
    int right = std::min(static_cast<int>(data.size()), peak_index + num_points);

    int left_points = peak_index - left;
    int right_points = right - peak_index;

    int symmetric_points = std::min(left_points, right_points);
    left = peak_index - symmetric_points;
    right = peak_index + symmetric_points;

    std::vector<float> symmetric_data(data.begin() + left, data.begin() + right);

    float kurt = kurtosis(symmetric_data);
    return kurt;
}

float ParklineOptimizer::kurtosis(const std::vector<float>& data) {
    int n = data.size();
    float mean = std::accumulate(data.begin(), data.end(), 0.0f) / n;
    float variance = 0.0f;
    for (const auto& x : data) {
        variance += (x - mean) * (x - mean);
    }
    variance /= n;

    if (variance == 0) return std::numeric_limits<float>::quiet_NaN();

    float kurtosis = 0.0f;
    for (const auto& x : data) {
        kurtosis += std::pow((x - mean) / std::sqrt(variance), 4);
    }
    kurtosis /= n;
    return kurtosis - 3.0f;  // 正态分布的峰态系数为0，因此减3
}

void ParklineOptimizer::optimate(const common::Line2d& src_line, 
        const PointCloudPtr& cloud, common::Line2d& tgt_line, bool& need_check) {
    // 点云变换到以直线为X轴的坐标系下
    PointCloudPtr cloud_trans(new PointCloud());
    // src_line.begin_point.translate(0.15, 0.15);
    // src_line.end_point.translate(-0.10, -0.10);
    Eigen::Matrix4d matrix = src_line.toMatrix();
    std::cout << "cloud_size = " << cloud->size() << std::endl;
    pcl::transformPointCloud(*cloud, *cloud_trans, matrix);
    for (auto& pt : cloud_trans->points) {
        if (pt.intensity < 2) {
            pt.intensity = 0;
        } else if (pt.intensity > 60) {
            pt.intensity = 1.0;
        } else {
            pt.intensity = (pt.intensity - 2) * 1.0 / (60 - 2);
        }
    }
    { // test
        // static int i = 0;
        // std::string save_dir = "/home/guitu/CLIP_data/optimizer/";
        // std::string cloud_path = save_dir + std::to_string(i) + ".pcd";
        // pcl::io::savePCDFile(cloud_path, *cloud, true);
        // cloud_path = save_dir + std::to_string(i) + "_trans.pcd";
        // pcl::io::savePCDFile(cloud_path, *cloud_trans, true);
        // ++i;
    }

    // 计算得分
    auto line_length = src_line.getLength();
    for (int i = 0; i < _trans_table.size(); ++i) {
        auto& trans_list = _trans_table[i];
        for (int j = 0; j < trans_list.size(); ++j) {
            auto& item = trans_list[j];
            item.computeScore(cloud_trans, line_length);
        }
    }

    float min_value = std::numeric_limits<float>::max();
    float max_value = -std::numeric_limits<float>::max();
    int max_i = -1, max_j = -1;
    for (int i = 0; i < _trans_table.size(); ++i) {
        const auto& trans_list = _trans_table[i];
        for (int j = 0; j < trans_list.size(); ++j) {
            const auto& item = trans_list[j];
            if (item.score > max_value) {
                max_value = item.score;
                max_i = i;
                max_j = j;
            } else if (item.score < min_value) {
                min_value = item.score;
            } else {}
        }
    }
    
    min_value = std::numeric_limits<float>::max();
    max_value = -std::numeric_limits<float>::max();
    const auto& trans_list = _trans_table[max_i];
    for (int j = 0; j < trans_list.size(); ++j) {
        const auto& item = trans_list[j];
        if (item.score > max_value) {
            max_value = item.score;
        } else if (item.score < min_value) {
            min_value = item.score;
        } else {}
    }
    std::cout << min_value << " " << max_value 
        << " idx: " << max_i << ", " << max_j << std::endl;
    std::cout << _trans_table[max_i][max_j].ox << " " 
        << _trans_table[max_i][max_j].oy << " "
        << _trans_table[max_i][max_j].oa << std::endl;

    { // 输出统计指标
        for (int j = 0; j < trans_list.size(); ++j) {
            const auto& item = trans_list[j];
            _outfile << item.score << " ";
        }
        _outfile << std::endl;
    }
    
    Eigen::Matrix3d matrix_res = src_line.toMatrix33().inverse() 
        * _trans_table[max_i][max_j].toMatrix33().inverse();
    double lx = src_line.getLength() * 0.5;
    tgt_line.begin_point = common::Point2d(-lx, 0);
    tgt_line.end_point = common::Point2d(lx, 0);
    tgt_line.begin_point.transform(matrix_res);
    tgt_line.end_point.transform(matrix_res);
    auto dist1 = src_line.begin_point.distanceTo(tgt_line.begin_point);
    auto dist2 = src_line.end_point.distanceTo(tgt_line.end_point);
    std::cout << "distance: " << dist1 << ", " << dist2 << std::endl;
    int list_size = static_cast<int>(trans_list.size());
    int span = 15;
    if (max_j - span - 1 < 0 || max_j + span - 1 > list_size) {
        need_check = true;
        std::cout << "需要检核情况02" << std::endl;
        return;
    }
    std::vector<float> left_scores;
    std::cout << "left score delta: ";
    for (int j = max_j; j > max_j - span; --j) {
        left_scores.push_back(trans_list[j].score - trans_list[j - 1].score);
        std::cout << trans_list[j].score - trans_list[j - 1].score << ", ";
    }
    std::cout << std::endl;
    double left_pho = computeSpearmanRatio(left_scores);
    
    std::vector<float> right_scores;
    std::cout << "right score delta: ";
    for (int j = max_j; j < max_j + span; ++j) {
        right_scores.push_back(trans_list[j].score - trans_list[j + 1].score);
        std::cout << trans_list[j].score - trans_list[j + 1].score << ", ";
    }
    std::cout << std::endl;
    double right_pho = computeSpearmanRatio(right_scores);
    std::cout << "spearman ratio: " << left_pho << ", " << right_pho << std::endl;
    if (left_pho < 0.95 || right_pho < 0.95) {
        need_check = true;
        std::cout << "需要检核情况03" << std::endl;
        return;
    }
    need_check = false;
}

void ParklineOptimizer::optimate2(const common::Line2d& src_line, 
        const PointCloudPtr& cloud, common::Line2d& tgt_line, bool& need_check) {
    common::Line2d raw_line = src_line;
    int iter_count = 0;
    double angle_diff = std::numeric_limits<double>::max();
    double trans_diff = std::numeric_limits<double>::max();
    
    // const int max_iter = 5; // 最大迭代优化次数
    // const double angle_thres = 0.2;  // 角度变化阈值: 0.2
    // const double trans_thres = 0.002;  // 平移量阈值: 0.002

    bool check_done = false;

    PointCloudPtr cloud_trans(new PointCloud());
    Eigen::Matrix4d matrix = raw_line.toMatrix();
    pcl::transformPointCloud(*cloud, *cloud_trans, matrix);

    intensity_Midfilter(cloud_trans);
    // gammaCorrection(cloud_trans, 0.5f);

    for (auto& pt : cloud_trans->points) {
        if (pt.intensity < 2) {
            pt.intensity = 0;
        } else if (pt.intensity > 60) {
            pt.intensity = 1.0;
        } else {
            pt.intensity = (pt.intensity - 2) * 1.0 / (60 - 2);
        }
    }

    // 计算得分
    auto line_length = raw_line.getLength();
    for (int i = 0; i < _trans_table.size(); ++i) {
        auto& trans_list = _trans_table[i];
        for (int j = 0; j < trans_list.size(); ++j) {
            auto& item = trans_list[j];
            item.computeScore(cloud_trans, line_length);
        }
    }

    float max_value = -std::numeric_limits<float>::max();
    int max_i = -1, max_j = -1;
    for (int i = 0; i < _trans_table.size(); ++i) {
        const auto& trans_list = _trans_table[i];
        for (int j = 0; j < trans_list.size(); ++j) {
            const auto& item = trans_list[j];
            if (item.score > max_value) {
                max_value = item.score;
                max_i = i;
                max_j = j;
            }
        }
    }

    Eigen::Matrix3d matrix_res = raw_line.toMatrix33().inverse() 
        * _trans_table[max_i][max_j].toMatrix33().inverse();
    double lx = line_length * 0.5;
    tgt_line.begin_point = common::Point2d(-lx, 0);
    tgt_line.end_point = common::Point2d(lx, 0);
    tgt_line.begin_point.transform(matrix_res);
    tgt_line.end_point.transform(matrix_res);

    angle_diff = _trans_table[max_i][max_j].oa;
    trans_diff = _trans_table[max_i][max_j].oy;

    std::cout << "\tAngle difference: " << angle_diff << std::endl;
    std::cout << "\tTrans difference: " << trans_diff << std::endl;

    raw_line = tgt_line;

    // 只在第一次迭代时执行检核和need_check设置
    if (!check_done) {
        int list_size = static_cast<int>(_trans_table[max_i].size());
        int span = 15;
        if (max_j - span < 0 || max_j + span > list_size) {
            need_check = true;
            std::cout << "\t需要检核情况02" << std::endl;
            _outfile << "2";
            _outfile << std::endl;
            // return;

        }
        std::vector<float> left_scores;
        for (int j = max_j; j > max_j - span; --j) {
            left_scores.push_back(_trans_table[max_i][j].score);
        }
        double left_pho = computeSpearmanRatio(left_scores);
        std::vector<float> right_scores;
        for (int j = max_j; j < max_j + span; ++j) {
            right_scores.push_back(_trans_table[max_i][j].score);
        }
        double right_pho = computeSpearmanRatio(right_scores);
        std::cout << "\tspearman ratio: " << left_pho << ", " << right_pho << std::endl;
        if (left_pho < 0.95 || right_pho < 0.95) { // 小昆山： 0.95 
            need_check = true;
            std::cout << "\t需要检核情况03" << std::endl;
            _outfile << "3";
            _outfile << std::endl;
            // return;
        }

        // 峰态系数检查
        std::vector<float> scores;
        for (const auto& item : _trans_table[max_i]) {
            scores.push_back(item.score);
        }

        int peak_index = std::distance(scores.begin(), std::max_element(scores.begin(), scores.end()));
        int num_points = 50;  
        auto kurt = calKurtosis(scores, peak_index, num_points);
        std::cout << "\tkurt: " << kurt << std::endl;
        if (kurt < -1.3 || kurt > -0.65) { // 小昆山：(-1.3, -0.65）
            need_check = true;
            std::cout << "\t需要检核情况04: 峰态系数不在范围内" << std::endl;
            _outfile << "4";
            _outfile << std::endl;
            // return;
        }

        // need_check = false;
        _outfile << "1";
        _outfile << std::endl;

        check_done = true;  
    }
}



void ParklineOptimizer::optimate3(const common::Line2d& src_line, 
        const PointCloudPtr& cloud, common::Line2d& tgt_line, bool& need_check) {
    // 点云变换到以直线为X轴的坐标系下
    PointCloudPtr cloud_trans(new PointCloud());

    Eigen::Matrix4d matrix = src_line.toMatrix();
    pcl::transformPointCloud(*cloud, *cloud_trans, matrix);
    
    // test
    // static int i = 0;
    // std::string save_dir = "/home/guitu/CLIP_data/optimizer/";
    // std::string cloud_path = save_dir + std::to_string(i) + ".pcd";
    // pcl::io::savePCDFile(cloud_path, *cloud);
    // cloud_path = save_dir + std::to_string(i) + "_trans.pcd";
    // pcl::io::savePCDFile(cloud_path, *cloud_trans);
    
    intensity_Midfilter(cloud_trans);
    gammaCorrection(cloud_trans, 0.5f);

    // 提取边界中点
    auto window_centers = generateWindowPointClouds(cloud_trans, window_size);
    // auto mid_points = extractBoundaryMidPoints(cloud_trans, window_centers, window_size, expansion, intensity_threshold);
    auto mid_points = refineBoundaryPoints(cloud_trans, window_centers, window_size, expansion, intensity_threshold);
    // auto mid_points = fitBoundaryPoints(cloud_trans, window_centers, window_size, expansion, intensity_threshold);


    if(mid_points.size() < 2) {
        need_check = true;
        std::cout << "需要检核情况04: 直线拟合点数太少" << std::endl;
        return;
    }

    // 计算均值和标准差
    Eigen::Vector2f mid_sum(0, 0);
    for (const auto& mp : mid_points) {
        mid_sum += mp;
    }
    Eigen::Vector2f avg = mid_sum / mid_points.size();

    // 标准差
    float variance = 0;
    for (const auto& mp : mid_points) {
        variance += pow(mp.y() - avg.y(), 2);
    }
    float standard_deviation = sqrt(variance / mid_points.size());
    float outlier_count = 0.0f;
    for (const auto& mp : mid_points) {
        if (fabs(mp.y() - avg.y()) > 2 * standard_deviation) {
            outlier_count += 1.0f;
        }
    }

    // 计算置信度
    float edge_length = src_line.getLength();
    float score1 = 2 * standard_deviation / edge_length;
    // float score2 = outlier_count / mid_points.size();
    float score3 = 1.0f - mid_points.size() / static_cast<float>(window_centers.size());
    // float confidence = exp(-sd_factor * score1 - outlier_factor * score2);
    // float confidence = exp(-sd_factor * score1 - valid_window_factor * score3);
    float confidence = exp(-sd_factor * score1);

    //剔除不符合条件的点
    if (mid_points.size() > 10){
        mid_points.erase(
            std::remove_if(mid_points.begin(), mid_points.end(),
                [&avg, &standard_deviation](const Eigen::Vector2f& mp) {
                    return abs(mp.y() - avg.y()) > 2 * standard_deviation;
                }),
            mid_points.end()
        );
    }    

    PointCloudPtr whole_mid_points(new PointCloud());
    Eigen::Matrix3d matrix_res = src_line.toMatrix33().inverse();
    // double lx = src_line.getLength() * 0.5;
    for (auto& mp : mid_points) {
        common::Point2d mp_2d(mp.x(), mp.y());
        mp_2d.transform(matrix_res);
        mp = Eigen::Vector2f(mp_2d.x, mp_2d.y);
        whole_mid_points->points.emplace_back(mp.x(), mp.y(), 1.0);
    }

    // 计算所有中点的平均值
    Eigen::Vector2f mid_points_sum(0, 0);
    for (const auto& mp : mid_points) {
        mid_points_sum += mp;
    }
    Eigen::Vector2f avg_point = mid_points_sum / mid_points.size();

    // 直线拟合
    Eigen::Vector2f initial_direction(src_line.getDirection().x, src_line.getDirection().y);
    Eigen::Vector2f direction = fitLineUsingMidPoints(mid_points).normalized();
    // Eigen::Vector2f direction = fitLineUsingRANSAC(mid_points, initial_direction);
    if (direction.isZero()) {
        need_check = true;    
        return;
    }

    { // test direction
        // PointCloudPtr direction_cloud(new PointCloud());
        // float step_length = 0.2f;
        // int steps = 20;
        // for (int j = -steps; j <= steps; ++j) {
        //     Eigen::Vector2f line_point = avg_point + j * step_length * direction;
        //     direction_cloud->points.emplace_back(line_point.x(), line_point.y(), 1.0);
        // }

        // direction_cloud->width = direction_cloud->points.size();
        // direction_cloud->height = 1;
        // direction_cloud->is_dense = true;
        // cloud_path = save_dir + std::to_string(i) + "_direction.pcd";
        // pcl::io::savePCDFile(cloud_path, *direction_cloud);

        // // 原始坐标系所有窗口中点
        // whole_mid_points->width = whole_mid_points->points.size();
        // whole_mid_points->height = 1;
        // whole_mid_points->is_dense = true;
        // cloud_path = save_dir + std::to_string(i) + "_whole_mid_points.pcd";
        // pcl::io::savePCDFile(cloud_path, *whole_mid_points);
        // ++i;
    }

    Eigen::Vector2f direction_pt = avg_point + 0.2 * direction;
    common::Line2d fit_line;
    fit_line.begin_point = common::Point2d(avg_point.x(), avg_point.y());
    fit_line.end_point = common::Point2d(direction_pt.x(), direction_pt.y());
    tgt_line.begin_point = fit_line.getProjectPoint(src_line.begin_point);
    tgt_line.end_point = fit_line.getProjectPoint(src_line.end_point);

    // std::cout << "\tconfidence : " << confidence << " score1: " << score1 << " score3: " << score3 << std::endl;
    // if(confidence < 0.7) {
    //     need_check = true;
    //     std::cout << "需要检核情况05: 置信度低" << std::endl;
    //     return;
    // }
    need_check = false;
}

void ParklineOptimizer::optimate4(const PointCloudPtr& cloud, const common::Point2d& original_corner, 
                                  common::Point2d& optimized_corner, const Eigen::Vector2f& direction1, 
                                  const Eigen::Vector2f& direction2, bool& need_check) {
    float theta = std::atan2(direction1.y(), direction1.x());  // 使 direction1 与 X 轴对齐
    float cos_theta = std::cos(-theta);  
    float sin_theta = std::sin(-theta);
    common::Point2d translated_corner = original_corner;
    translated_corner.x = 0;
    translated_corner.y = 0;

    PointCloudPtr rotated_cloud(new PointCloud);
    for (const auto& point : *cloud) {
        common::Point2d translated_point;
        translated_point.x = point.x - original_corner.x;
        translated_point.y = point.y - original_corner.y;

        PointX rotated_point;
        rotated_point.x = cos_theta * translated_point.x - sin_theta * translated_point.y;
        rotated_point.y = sin_theta * translated_point.x + cos_theta * translated_point.y;
        rotated_point.z = point.z;
        rotated_point.r = point.r;
        rotated_point.g = point.g;
        rotated_point.b = point.b;
        rotated_point.intensity = point.intensity;
        rotated_cloud->push_back(rotated_point);
    }

    // PointCloudPtr direction_cloud(new PointCloud);
    Eigen::Vector2f rotated_dir1, rotated_dir2;
    rotated_dir1.x() = cos_theta * direction1.x() - sin_theta * direction1.y();
    rotated_dir1.y() = sin_theta * direction1.x() + cos_theta * direction1.y();
    rotated_dir2.x() = cos_theta * direction2.x() - sin_theta * direction2.y();
    rotated_dir2.y() = sin_theta * direction2.x() + cos_theta * direction2.y();
    bool rotated_dir1_positive_x = (rotated_dir1.x() > 0);
    bool rotated_dir2_positive_y = (rotated_dir2.y() > 0);
    // PointX pt;
    // pt.x = rotated_dir1.x();
    // pt.y = rotated_dir1.y();
    // pt.z = 1;
    // direction_cloud->push_back(pt);
    // pt.x = rotated_dir2.x();
    // pt.y = rotated_dir2.y();
    // pt.z = 1;
    // direction_cloud->push_back(pt);
    // static int i = 0;
    // std::string save_dir = "/home/guitu/CLIP_data/corner/";
    // std::string cloud_path = save_dir + std::to_string(i) + "roi.pcd";
    // pcl::io::savePCDFile(cloud_path, *rotated_cloud);
    // cloud_path = save_dir + std::to_string(i) + "direction.pcd";
    // pcl::io::savePCDFile(cloud_path, *direction_cloud);
    // i++;

    // intensity_Midfilter(rotated_cloud);

    for (auto& pt : rotated_cloud->points) {
        if (pt.intensity < 2) {
            pt.intensity = 0;
        } else if (pt.intensity > 60) {
            pt.intensity = 1.0;
        } else {
            pt.intensity = (pt.intensity - 2) * 1.0 / (60 - 2);
        }
    }

    // 计算得分
    for (int i = 0; i < _xy_trans_table.size(); ++i) {
        auto& trans_list = _xy_trans_table[i];
        for (int j = 0; j < trans_list.size(); ++j) {
            auto& item = trans_list[j];
            item.computeScore(rotated_cloud, rotated_dir1_positive_x, rotated_dir2_positive_y);
        }
    }

    // 找出得分最高的平移
    float min_value = std::numeric_limits<float>::max();
    float max_value = -std::numeric_limits<float>::max();
    int max_i = -1, max_j = -1;
    for (int i = 0; i < _xy_trans_table.size(); ++i) {
        const auto& trans_list = _xy_trans_table[i];
        for (int j = 0; j < trans_list.size(); ++j) {
            const auto& item = trans_list[j];
            if (item.score > max_value) {
                max_value = item.score;
                max_i = i;
                max_j = j;
            } 
            if (item.score < min_value) {
                min_value = item.score;
            }
        }
    }

    if (max_i != -1 && max_j != -1) {
        const auto& best_item = _xy_trans_table[max_i][max_j];
        std::cout << "tx: " << best_item.ox << " ty: " << best_item.oy << " score: " << best_item.score << std::endl;
        
        float best_tx = best_item.ox;
        float best_ty = best_item.oy;
        
        float cos_inv_theta = std::cos(theta);
        float sin_inv_theta = std::sin(theta);
        optimized_corner.x = -best_tx * cos_inv_theta + best_ty * sin_inv_theta + original_corner.x;
        optimized_corner.y = -best_tx * sin_inv_theta - best_ty * cos_inv_theta + original_corner.y;
    }

    // 设置 need_check，根据得分差异判断是否需要进一步检查
    need_check = false;
}

void ParklineOptimizer::optimate4_2(const PointCloudPtr& cloud, const common::Point2d& original_corner, 
                                  common::Point2d& optimized_corner, const Eigen::Vector2f& direction1, 
                                  const Eigen::Vector2f& direction2, bool& need_check, int corner_type, int move_direction_flag) {
    float theta = std::atan2(direction1.y(), direction1.x());  // 使 direction1 与 X 轴对齐
    float cos_theta = std::cos(-theta);  
    float sin_theta = std::sin(-theta);

    PointCloudPtr rotated_cloud(new PointCloud);
    for (const auto& point : *cloud) {
        common::Point2d translated_point;
        translated_point.x = point.x - original_corner.x;
        translated_point.y = point.y - original_corner.y;

        PointX rotated_point;
        rotated_point.x = cos_theta * translated_point.x - sin_theta * translated_point.y;
        rotated_point.y = sin_theta * translated_point.x + cos_theta * translated_point.y;
        rotated_point.z = point.z;
        rotated_point.r = point.r;
        rotated_point.g = point.g;
        rotated_point.b = point.b;
        rotated_point.intensity = point.intensity;
        rotated_cloud->push_back(rotated_point);
    }

    // PointCloudPtr direction_cloud(new PointCloud);
    Eigen::Vector2f rotated_dir1, rotated_dir2;
    rotated_dir1.x() = cos_theta * direction1.x() - sin_theta * direction1.y();
    rotated_dir1.y() = sin_theta * direction1.x() + cos_theta * direction1.y();
    rotated_dir2.x() = cos_theta * direction2.x() - sin_theta * direction2.y();
    rotated_dir2.y() = sin_theta * direction2.x() + cos_theta * direction2.y();
    bool rotated_dir1_positive_x = (rotated_dir1.x() > 0);
    bool rotated_dir2_positive_y = (rotated_dir2.y() > 0);
    // PointX pt;
    // pt.x = rotated_dir1.x();
    // pt.y = rotated_dir1.y();
    // pt.z = 1;
    // direction_cloud->push_back(pt);
    // pt.x = rotated_dir2.x();
    // pt.y = rotated_dir2.y();
    // pt.z = 1;
    // direction_cloud->push_back(pt);
    // static int i = 0;
    // std::string save_dir = "/home/guitu/CLIP_data/corner/";
    // std::string cloud_path = save_dir + std::to_string(i) + "roi.pcd";
    // pcl::io::savePCDFile(cloud_path, *rotated_cloud);
    // cloud_path = save_dir + std::to_string(i) + "direction.pcd";
    // pcl::io::savePCDFile(cloud_path, *direction_cloud);
    // i++;

    // intensity_Midfilter(rotated_cloud);

    for (auto& pt : rotated_cloud->points) {
        if (pt.intensity < 2) {
            pt.intensity = 0;
        } else if (pt.intensity > 60) {
            pt.intensity = 1.0;
        } else {
            pt.intensity = (pt.intensity - 2) * 1.0 / (60 - 2);
        }
    }

    // 计算得分
    for (int i = 0; i < _xy_trans_table.size(); ++i) {
        auto& trans_list = _xy_trans_table[i];
        for (int j = 0; j < trans_list.size(); ++j) {
            auto& item = trans_list[j];
            if (corner_type == 1) {
                item.computeLScore(rotated_cloud, rotated_dir1_positive_x, rotated_dir2_positive_y);
            } else if (corner_type == 2) {
                item.computeTScore(rotated_cloud, rotated_dir1_positive_x, rotated_dir2_positive_y, move_direction_flag);
            } else if (corner_type == 3 || corner_type == 4) {
                item.computeMultiScore(rotated_cloud, rotated_dir1_positive_x, rotated_dir2_positive_y);
            }
        }
    }

    // 找出得分最高的平移
    float min_value = std::numeric_limits<float>::max();
    float max_value = -std::numeric_limits<float>::max();
    int max_i = -1, max_j = -1;
    for (int i = 0; i < _xy_trans_table.size(); ++i) {
        const auto& trans_list = _xy_trans_table[i];
        for (int j = 0; j < trans_list.size(); ++j) {
            const auto& item = trans_list[j];
            if (item.score > max_value) {
                max_value = item.score;
                max_i = i;
                max_j = j;
            } 
            if (item.score < min_value) {
                min_value = item.score;
            }
        }
    }

    if (max_i != -1 && max_j != -1) {
        const auto& best_item = _xy_trans_table[max_i][max_j];
        // std::cout << "\t\ttx: " << best_item.ox << " ty: " << best_item.oy << " score: " << best_item.score << std::endl;
        
        float best_tx = best_item.ox;
        float best_ty = best_item.oy;
        
        float cos_inv_theta = std::cos(theta);
        float sin_inv_theta = std::sin(theta);
        optimized_corner.x = -best_tx * cos_inv_theta + best_ty * sin_inv_theta + original_corner.x;
        optimized_corner.y = -best_tx * sin_inv_theta - best_ty * cos_inv_theta + original_corner.y;
    }

    // 设置 need_check，根据得分差异判断是否需要进一步检查
    need_check = false;
}
}