// main_viewer.h隐性包含了opencv，flann会与opencv产生冲突；
// 故在opencv至前包含flann库，否则会报pcl/kdtree/kdtree_flann.h中的错误
#include "main_viewer.h"
#include <common/base/dir.h>
#include <QCompleter>
#include <QFile>
#include <QFileInfo>
#include <pcl/common/common.h>
#include <ucanvas/item.hpp>
#include <ucanvas/tool.hpp>
#include <uface/resource/uresource.h>
#include <uface/widget/uwidget.h>
#include <common/point_cloud/filter.h>
#include "parkspacegroup.h"
#include "parkline_optimizer.h"
#include "file_system.h"
// #include "bev_processor.h"
#include <QDebug>

namespace welkin::bamboo {
#define NameRole Qt::UserRole + 9
#define TextRole Qt::UserRole + 10
#define TypeRole Qt::UserRole + 11
MainViewer::MainViewer(QWidget* parent) : ucanvas::Viewer(parent) {
    this->setYFlipped(true);
    // 创建底图图层
    createBaseMapLayer();
    // 创建绘制工具栏
    createDrawToolBar();
    _file_system = new ParklotFileSystem();
}

MainViewer::~MainViewer() {}

ucanvas::RasterImageLayer *MainViewer::rasterImageLayer() const {
    return _raster_layer;
}
ucanvas::GenericLayer* MainViewer::shapeLayer() const {
    return _shape_layer;
}

void MainViewer::clearRasterImageLayer() {
    _raster_layer->reset();
}
void MainViewer::clearShapeLayer() {
    _shape_layer->removeAllItems();
}

QToolBar *MainViewer::drawToolBar() const {
    return _draw_toolbar;
}

void MainViewer::clearViewer() {
    this->clearRasterImageLayer();
    this->clearShapeLayer();
}

void MainViewer::onOptimateSlot() {
    _file_system->setRootDir("/home/guitu/CLIP_data/小昆山-proj/proj");
    common::Dir::Mkdir(_file_system->getVectorizationDir());
    QVector<ParkSpace> park_space_vec;
    for (auto item : _shape_layer->items()) {
        auto eitem = dynamic_cast<ucanvas::PolygonItem*>(item);
        if (!eitem) {continue;}
        auto polygon = eitem->scenePolygon();
        ParkSpace park_space;
        park_space.points = UFQ(polygon);
        auto length1 = park_space.getLine(0).getLength();
        auto length2 = park_space.getLine(1).getLength();
        auto min_length = std::min(length1, length2);
        auto max_length = std::max(length1, length2);
        if (max_length / min_length > 4.0) {continue;}
        park_space_vec.push_back(std::move(park_space));
    }
    // 计算出Group
    _park_space_group_vec.clear();
    while(!park_space_vec.empty()) {
        if (_park_space_group_vec.empty()) {
            ParkSpaceGroup park_space_group;
            park_space_group.addParkSpace(park_space_vec.front());
            park_space_vec.pop_front();
            _park_space_group_vec.push_back(std::move(park_space_group));
            continue;
        }
        QVector<int> valid_ids;
        auto& park_space_group = _park_space_group_vec.back();
        for (int i = 0; i < park_space_vec.size(); ++i) {
            const auto& park_space = park_space_vec.at(i);
            if (park_space_group.isIntersect(park_space)) {
                park_space_group.addParkSpace(park_space);
                valid_ids.append(i);
            }
        }
        if (valid_ids.empty()) {
            ParkSpaceGroup park_space_group;
            park_space_group.addParkSpace(park_space_vec.front());
            park_space_vec.pop_front();
            _park_space_group_vec.push_back(std::move(park_space_group));
            continue;
        }
        for (int i = valid_ids.size() - 1; i >= 0; --i) {
            park_space_vec.removeAt(valid_ids.at(i));
        }
    }
    // 计算长线方向和库位线
    for (auto& group : _park_space_group_vec) {
        group.computeLongSideDirection();
        group.computeParkLines();
        // 计算设计尺寸
        for (auto& ps : group.parkspaces) {
            ps.computeDesignSize();
        }
    }
    // 拟合
    auto dom_folder = _project->getDOMFolder();
    cv::Mat image;
    common::Point2d origin;
    double padding = 0.8; // 即允许转换之后存在80cm的误差
    // int index = 0;
    for (auto& group : _park_space_group_vec) {
        // ++index;
        
        common::Rectd rect = group.getBoudingRect();
        rect.addPadding(padding, padding);
        // std::cout << "Start succeed: " << rect << std::endl;
        if (!dom_folder->extractImageByRect(image, rect)) {
            group.valid = false;
            continue;
        }
        // std::cout << "Extract " << std::endl;
        origin = rect.getTopLeftPoint();
        // 拟合优化
        if (group.fit(image, origin, 0.4, 0.6) == -1) {
            group.valid = false;
            continue;
        }
        group.valid = true;
        rect = group.getBoudingRect();
        rect.addPadding(padding, padding);
        if (!dom_folder->extractImageByRect(image, rect)) {continue;}
        origin = rect.getTopLeftPoint();
        // AINFO << index << "/" << _park_space_group_vec.size();
        // 边缘检测
        group.edgeDetect(image, origin, 60, 100);
        // 改正库位
        group.retifyParkSpace();
    }
    _shape_layer->removeAllItems();
    // 重新设置polygon
    for (const auto& group : _park_space_group_vec) {
        if (!group.valid) {continue;}
        for (const auto& ps : group.parkspaces) {
            QPolygonF pg = UTQ(ps.points);
            _shape_layer->addPolygon(pg);
        }
    }

    std::string outputFilePath = _file_system->getCoarseParkingSlotsPath();
    std::ofstream outputFile(outputFilePath);
    std::set<std::tuple<double, double, double, double>> processed_edges;
    
    for (const auto& group : _park_space_group_vec) {
        if (!group.valid) continue;
        for (const auto& ps : group.parkspaces) {
            for (int i = 0; i < 4; ++i) {
                common::Point2d p1 = ps.points[i];
                common::Point2d p2 = ps.points[(i + 1) % 4];
                
                // 创建标准化的边表示（确保p1 < p2，便于检测重复）
                std::tuple<double, double, double, double> edge;
                if (p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y)) {
                    edge = std::make_tuple(p1.x, p1.y, p2.x, p2.y);
                } else {
                    edge = std::make_tuple(p2.x, p2.y, p1.x, p1.y);
                }
                
                // 检查这条边是否已经处理过
                if (processed_edges.find(edge) != processed_edges.end()) {
                    continue; 
                }                
                processed_edges.insert(edge);
                
                // 计算边的中心点和长度
                double center_x = (p1.x + p2.x) / 2.0;
                double center_y = (p1.y + p2.y) / 2.0;
                double length = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
                
                outputFile << p1.x << " " << p1.y << " " 
                          << p2.x << " " << p2.y << " "
                          << center_x << " " << center_y << " "
                          << length << "\n";
            }
        }
    }

    outputFile.close();
    std::cout << "Coarse optimate done" << std::endl;
    // for (auto& group : _park_space_group_vec) {
    //     for (auto& pl : group.parklines) {
    //         // if (!pl.fit_success) {continue;}
    //         QLineF pl1 = UTQ(pl.border_line1);
    //         QLineF pl2 = UTQ(pl.border_line2);
    //         QLineF pl3 = UTQ(pl.center_line);
    //         _shape_layer->addLine(pl1);
    //         _shape_layer->addLine(pl2);
    //         _shape_layer->addLine(pl3);
    //     }
    // }
}
void MainViewer::onOptimateSlot2() {
    auto local_pose_filer = _project->getLocalPoseFiler();
    common::Pose3d pose_trans = local_pose_filer->load();
    Eigen::Matrix4d matrix_trans = pose_trans.toMatrix();
// 优化
    ParklineOptimizer pl_optimizer;
    auto cloud_list_folder = _project->getOriginDataFolder()->getCloudListFolder();
    // int valid_index = 0;
    // 增加优化方法
    double offset = 0.2; // offset: 0.125
    int index = 0;
    double pixel_size = 0.05; 
    std::string reliabilityFilePath = _file_system->getReliabiltyPath();
    std::ofstream reliabilityFile(reliabilityFilePath);
    for (auto& group : _park_space_group_vec) {
        if (!group.valid) {continue;}
        for (auto& pl : group.parklines) {
            // 提取点云
            bool need_check = false;
            auto polygon = pl.getParallelArea(offset);
            auto cloud = cloud_list_folder->extractCloudInPolygon(polygon);
            common::point_cloud::RangeZFilter<PointX>(*cloud, common::Rangef(0.5, 1.5)); // 小昆山
            // common::point_cloud::RangeZFilter<PointX>(*cloud, common::Rangef(-1.3, 0.5)); // 九亭

            std::map<std::pair<int, int>, std::vector<PointX>> pixel_map;
            for (const auto& point : *cloud) {
                int pixel_x = static_cast<int>(point.x / pixel_size);
                int pixel_y = static_cast<int>(point.y / pixel_size);
                pixel_map[{pixel_x, pixel_y}].emplace_back(point); 
            }

            // 判断高度差并剔除非地面点
            float height_threshold = 0.1; // 设置阈值
            PointCloudPtr filtered_cloud(new PointCloud()); 

            for (const auto& entry : pixel_map) {
                const auto& points = entry.second;
                float min_height = std::numeric_limits<double>::max();
                float max_height = std::numeric_limits<double>::lowest();

                for (const auto& p : points) {
                    min_height = std::min(min_height, p.z);
                    max_height = std::max(max_height, p.z);
                }

                if (max_height - min_height <= height_threshold) {
                    for (const auto& p : points) {
                        filtered_cloud->emplace_back(p);
                    }
                }
            }
            // {
            //     static int i = 0;
            //     std::string save_dir = "/home/guitu/CLIP_data/optimizer/";
            //     std::string cloud_path = save_dir + std::to_string(i) + "raw.pcd";
            //     pcl::io::savePCDFile(cloud_path, *cloud);
            //     cloud_path = save_dir + std::to_string(i) + "filtered.pcd";
            //     pcl::io::savePCDFile(cloud_path, *filtered_cloud);
            //     ++i;
            // }

            {
                std::cout << "===========================================" << std::endl;
                std::cout << "Parkline index: " << ++index << std::endl;
                common::Line2d show_line = pl.center_line;
                common::Point3d bpt3(show_line.begin_point.x, show_line.begin_point.y, 0);
                bpt3.transform(matrix_trans);
                common::Point3d ept3(show_line.end_point.x, show_line.end_point.y, 0);
                ept3.transform(matrix_trans);
                show_line.begin_point = common::Point2d(bpt3.x, bpt3.y);
                show_line.end_point = common::Point2d(ept3.x, ept3.y);
                std::cout << "line: " << show_line << std::endl;
            }
            common::Line2d opt_line;
            pl_optimizer.optimate2(pl.center_line, filtered_cloud, opt_line, need_check);
            if (pl.type == ParkLine::TypeBorder) {
                std::cout << "origin center line: " << pl.center_line << std::endl;
                std::cout << "opt center line: " << opt_line << std::endl;
            }
            pl.center_line = opt_line;
            pl.line_width = 0.15;
            pl.computeBorderLine();
            pl.retify(pl.border_line1, pl.border_line2);
            pl.need_check = need_check;
            pl.test_index = index;
            if (pl.need_check) {
                // 保存需要检查的线段端点
                reliabilityFile << pl.center_line.begin_point.x << " " << pl.center_line.begin_point.y << " "
                               << pl.center_line.end_point.x << " " << pl.center_line.end_point.y << "\n";
            }
        }
        group.retifyParkSpace();
        // if (valid_index++ > 3) {
        //     break;
        // }
    }
    // valid_index = 0;
    _opt_layer->removeAllItems();
    // 重新设置polygon
    for (const auto& group : _park_space_group_vec) {
        if (!group.valid) {continue;}
        for (const auto& ps : group.parkspaces) {
            QPolygonF pg = UTQ(ps.points);
            _opt_layer->addPolygon(pg);
        }
    }

    std::string outputFilePath = _file_system->getFineEdgeParkingSlotsPath();
    std::ofstream outputFile(outputFilePath);
    std::set<std::tuple<double, double, double, double>> processed_edges;
    
    for (const auto& group : _park_space_group_vec) {
        if (!group.valid) continue;
        for (const auto& ps : group.parkspaces) {
            for (int i = 0; i < 4; ++i) {
                common::Point2d p1 = ps.points[i];
                common::Point2d p2 = ps.points[(i + 1) % 4];
                
                // 创建标准化的边表示（确保p1 < p2，便于检测重复）
                std::tuple<double, double, double, double> edge;
                if (p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y)) {
                    edge = std::make_tuple(p1.x, p1.y, p2.x, p2.y);
                } else {
                    edge = std::make_tuple(p2.x, p2.y, p1.x, p1.y);
                }
                
                // 检查这条边是否已经处理过
                if (processed_edges.find(edge) != processed_edges.end()) {
                    continue; 
                }                
                processed_edges.insert(edge);
                
                // 计算边的中心点和长度
                double center_x = (p1.x + p2.x) / 2.0;
                double center_y = (p1.y + p2.y) / 2.0;
                double length = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
                
                outputFile << p1.x << " " << p1.y << " " 
                          << p2.x << " " << p2.y << " "
                          << center_x << " " << center_y << " "
                          << length << "\n";
            }
        }
    }
    reliabilityFile.close();
    outputFile.close();

}

void MainViewer::onOptimateSlot3() {
    auto local_pose_filer = _project->getLocalPoseFiler();
    common::Pose3d pose_trans = local_pose_filer->load();
    // Eigen::Matrix4d matrix_trans = pose_trans.toMatrix();
// 优化
    ParklineOptimizer pl_optimizer;
    auto cloud_list_folder = _project->getOriginDataFolder()->getCloudListFolder();
    // int valid_index = 0;
    // 增加优化方法
    double offset = 0.05;
    bool need_check = false;
    bool need_check1 = false;
    bool need_check2 = false;
    int index = 0;
    double pixel_size = 0.05;
    for (auto& group : _park_space_group_vec) {
        if (!group.valid) {continue;}
        for (auto& pl : group.parklines) {
            // 提取点云
            auto polygon1 = pl.getBorderParallelArea(pl.border_line1, offset);
            auto polygon2 = pl.getBorderParallelArea(pl.border_line2, offset);
            auto cloud1 = cloud_list_folder->extractCloudInPolygon(polygon1);
            auto cloud2 = cloud_list_folder->extractCloudInPolygon(polygon2);
            common::point_cloud::RangeZFilter<PointX>(*cloud1, common::Rangef(0.5, 1.5));
            common::point_cloud::RangeZFilter<PointX>(*cloud2, common::Rangef(0.5, 1.5));
            // common::point_cloud::RangeZFilter<PointX>(*cloud1, common::Rangef(-1.3, 0.5));
            // common::point_cloud::RangeZFilter<PointX>(*cloud2, common::Rangef(-1.3, 0.5));

            std::map<std::pair<int, int>, std::vector<PointX>> pixel_map1, pixel_map2;
            for (const auto& point : *cloud1) {
                int pixel_x = static_cast<int>(point.x / pixel_size);
                int pixel_y = static_cast<int>(point.y / pixel_size);
                pixel_map1[{pixel_x, pixel_y}].emplace_back(point); 
            }
            for (const auto& point : *cloud2) {
                int pixel_x = static_cast<int>(point.x / pixel_size);
                int pixel_y = static_cast<int>(point.y / pixel_size);
                pixel_map2[{pixel_x, pixel_y}].emplace_back(point); 
            }

            // 判断高度差并剔除非地面点
            float height_threshold = 0.1; // 设置阈值
            PointCloudPtr filtered_cloud1(new PointCloud()); 
            PointCloudPtr filtered_cloud2(new PointCloud()); 

            for (const auto& entry : pixel_map1) {
                const auto& points = entry.second;
                float min_height = std::numeric_limits<double>::max();
                float max_height = std::numeric_limits<double>::lowest();

                for (const auto& p : points) {
                    min_height = std::min(min_height, p.z);
                    max_height = std::max(max_height, p.z);
                }

                if (max_height - min_height <= height_threshold) {
                    for (const auto& p : points) {
                        filtered_cloud1->emplace_back(p);
                    }
                }
            }
            for (const auto& entry : pixel_map2) {
                const auto& points = entry.second;
                float min_height = std::numeric_limits<double>::max();
                float max_height = std::numeric_limits<double>::lowest();

                for (const auto& p : points) {
                    min_height = std::min(min_height, p.z);
                    max_height = std::max(max_height, p.z);
                }

                if (max_height - min_height <= height_threshold) {
                    for (const auto& p : points) {
                        filtered_cloud2->emplace_back(p);
                    }
                }
            }




            { // test
                // auto polygon = pl.getParallelArea(10.0);
                // auto cloud = cloud_list_folder->extractCloudInPolygon(polygon);
                // common::point_cloud::RangeZFilter<PointX>(*cloud, common::Rangef(0.5, 1.5));
                // // common::point_cloud::RangeZFilter<PointX>(*cloud, common::Rangef(-1.3, 0.5));
                // static int i = 0;
                // std::string save_dir = "/home/guitu/CLIP_data/optimizer/";
                // std::string cloud_path = save_dir + std::to_string(i) + "whole.pcd";
                // pcl::io::savePCDFile(cloud_path, *cloud);
                // ++i;
            }


            std::cout << "===========================================" << std::endl;
            std::cout << "Parkline index: " << ++index << std::endl;
            
            need_check = false;
            common::Line2d opt_line1, opt_line2;
            std::cout << "border1" << std::endl;
            pl_optimizer.optimate3(pl.border_line1, filtered_cloud1, opt_line1, need_check1);
            std::cout << "\torigin border line: " << pl.border_line1 << std::endl;
            if (need_check1) need_check = true;
            else {
                pl.border_line1 = opt_line1;
                std::cout << "\topt border line: " << opt_line1 << std::endl;
            }
            std::cout << "border2" << std::endl;
            pl_optimizer.optimate3(pl.border_line2, filtered_cloud2, opt_line2, need_check2);
            std::cout << "\torigin border line: " << pl.border_line2 << std::endl;
            if (need_check2) need_check = true;
            else {
                pl.border_line2 = opt_line2;
                std::cout << "\topt border line: " << opt_line2 << std::endl;
            }
            pl.line_width = 0.15;
            pl.retify(pl.border_line1, pl.border_line2);
            pl.need_check = need_check;
            pl.test_index = index;
        }
        group.retifyParkSpace();
    }

    // 重新设置polygon
    _opt_layer->removeAllItems();
    for (const auto& group : _park_space_group_vec) {
        if (!group.valid) {continue;}
        for (const auto& ps : group.parkspaces) {
            QPolygonF pg = UTQ(ps.points);
            _opt_layer->addPolygon(pg);
        }
    }

    std::string outputFilePath = _file_system->getFineParkingSlotsPath();
    std::ofstream outputFile(outputFilePath);
    std::set<std::tuple<double, double, double, double>> processed_edges;
    
    for (const auto& group : _park_space_group_vec) {
        if (!group.valid) continue;
        for (const auto& ps : group.parkspaces) {
            for (int i = 0; i < 4; ++i) {
                common::Point2d p1 = ps.points[i];
                common::Point2d p2 = ps.points[(i + 1) % 4];
                
                // 创建标准化的边表示（确保p1 < p2，便于检测重复）
                std::tuple<double, double, double, double> edge;
                if (p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y)) {
                    edge = std::make_tuple(p1.x, p1.y, p2.x, p2.y);
                } else {
                    edge = std::make_tuple(p2.x, p2.y, p1.x, p1.y);
                }
                
                // 检查这条边是否已经处理过
                if (processed_edges.find(edge) != processed_edges.end()) {
                    continue; 
                }                
                processed_edges.insert(edge);
                
                // 计算边的中心点和长度
                double center_x = (p1.x + p2.x) / 2.0;
                double center_y = (p1.y + p2.y) / 2.0;
                double length = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
                
                outputFile << p1.x << " " << p1.y << " " 
                          << p2.x << " " << p2.y << " "
                          << center_x << " " << center_y << " "
                          << length << "\n";
            }
        }
    }
}



void MainViewer::recordCorner(const Eigen::Vector2f& corner, const ParkSpace& space, int parkspace_id, 
                              int corner_id, std::map<Eigen::Vector2f, CornerInfo, Vector2fCompare>& corner_info_) {
    if (corner_info_.find(corner) != corner_info_.end()) {
        return;
    }
    Eigen::Vector2f next_point1, next_point2;
    if (corner == Eigen::Vector2f(space.points[0].x, space.points[0].y)) {
        next_point1 = Eigen::Vector2f(space.points[1].x, space.points[1].y);
        next_point2 = Eigen::Vector2f(space.points[3].x, space.points[3].y);
    } else if (corner == Eigen::Vector2f(space.points[1].x, space.points[1].y)) {
        next_point1 = Eigen::Vector2f(space.points[0].x, space.points[0].y);
        next_point2 = Eigen::Vector2f(space.points[2].x, space.points[2].y);
    } else if (corner == Eigen::Vector2f(space.points[2].x, space.points[2].y)) {
        next_point1 = Eigen::Vector2f(space.points[1].x, space.points[1].y);
        next_point2 = Eigen::Vector2f(space.points[3].x, space.points[3].y);
    } else if (corner == Eigen::Vector2f(space.points[3].x, space.points[3].y)) {
        next_point1 = Eigen::Vector2f(space.points[0].x, space.points[0].y);
        next_point2 = Eigen::Vector2f(space.points[2].x, space.points[2].y);
    }

    Eigen::Vector2f direction1 = (next_point1 - corner).normalized();
    Eigen::Vector2f direction2 = (next_point2 - corner).normalized();

    // 存储该角点的位置信息及两条边的方向，并记录该角点所属的库位号
    corner_info_[corner] = {next_point1, next_point2, direction1, direction2, parkspace_id, corner_id};
}

common::Polygon2d MainViewer::handleLCorner(const Eigen::Vector2f& corner, const std::map<Eigen::Vector2f, CornerInfo, Vector2fCompare>& corner_info_, 
    Eigen::Vector2f& new_corner, Eigen::Vector2f& trans) {
    const auto& corner_info = corner_info_.at(corner);
    Eigen::Vector2f direction1 = corner_info.direction1;
    Eigen::Vector2f direction2 = corner_info.direction2;

    new_corner = corner + 0.075 * (direction1 + direction2);
    trans = 0.075 * (direction1 + direction2);

    Eigen::Vector2f pt1, pt2, pt3, pt4;
    common::Polygon2d polygon;
    pt1 = new_corner + 0.6 * (direction1 + direction2);
    pt2 = new_corner + 0.6 * (direction1 - direction2);
    pt3 = new_corner - 0.6 * (direction1 + direction2);
    pt4 = new_corner - 0.6 * (direction1 - direction2);
    polygon.addPoint(common::Point2d(pt1.x(), pt1.y()));
    polygon.addPoint(common::Point2d(pt2.x(), pt2.y()));
    polygon.addPoint(common::Point2d(pt3.x(), pt3.y()));
    polygon.addPoint(common::Point2d(pt4.x(), pt4.y()));

    return polygon;
}

common::Polygon2d MainViewer::handleTCorner(const Eigen::Vector2f& corner, 
                               const std::map<Eigen::Vector2f, CornerInfo, Vector2fCompare>& corner_info_,
                               const std::map<Eigen::Vector2f, int, Vector2fCompare>& corner_count_map, 
                               Eigen::Vector2f& new_corner, Eigen::Vector2f& trans) {
    const auto& corner_info = corner_info_.at(corner);
    Eigen::Vector2f next_point1 = corner_info.next_point1;
    Eigen::Vector2f next_point2 = corner_info.next_point2;

    Eigen::Vector2f move_direction;
    if (corner_count_map.at(next_point1) > corner_count_map.at(next_point2)) {
        move_direction = (next_point1 - corner).normalized();
    } else if(corner_count_map.at(next_point1) < corner_count_map.at(next_point2)) {
        move_direction = (next_point2 - corner).normalized();
    } else if (corner_count_map.at(next_point1) == corner_count_map.at(next_point2)) {
        float distance1 = (next_point1 - corner).norm(); 
        float distance2 = (next_point2 - corner).norm(); 
        if (distance1 > distance2) {
            move_direction = (next_point1 - corner).normalized(); // Move towards next_point1
        } else {
            move_direction = (next_point2 - corner).normalized(); // Move towards next_point2
        }
    } else {
        move_direction = Eigen::Vector2f::Zero();
    }

    new_corner = corner + 0.075 * move_direction;
    trans = 0.075 * move_direction;

    Eigen::Vector2f direction1 = corner_info.direction1;
    Eigen::Vector2f direction2 = corner_info.direction2;
    
    Eigen::Vector2f pt1, pt2, pt3, pt4;
    common::Polygon2d polygon;
    pt1 = new_corner + 0.6 * (direction1 + direction2);
    pt2 = new_corner + 0.6 * (direction1 - direction2);
    pt3 = new_corner - 0.6 * (direction1 + direction2);
    pt4 = new_corner - 0.6 * (direction1 - direction2);
    polygon.addPoint(common::Point2d(pt1.x(), pt1.y()));
    polygon.addPoint(common::Point2d(pt2.x(), pt2.y()));
    polygon.addPoint(common::Point2d(pt3.x(), pt3.y()));
    polygon.addPoint(common::Point2d(pt4.x(), pt4.y()));
    
    return polygon;
}

common::Polygon2d MainViewer::handleMultiCorner(const Eigen::Vector2f& corner, 
                                   const std::map<Eigen::Vector2f, CornerInfo, Vector2fCompare>& corner_info_, Eigen::Vector2f& new_corner) {
    const auto& corner_info = corner_info_.at(corner);
    Eigen::Vector2f direction1 = corner_info.direction1;
    Eigen::Vector2f direction2 = corner_info.direction2;

    new_corner = corner;

    Eigen::Vector2f pt1, pt2, pt3, pt4;
    common::Polygon2d polygon;
    pt1 = new_corner + 0.6 * (direction1 + direction2);
    pt2 = new_corner + 0.6 * (direction1 - direction2);
    pt3 = new_corner - 0.6 * (direction1 + direction2);
    pt4 = new_corner - 0.6 * (direction1 - direction2);
    polygon.addPoint(common::Point2d(pt1.x(), pt1.y()));
    polygon.addPoint(common::Point2d(pt2.x(), pt2.y()));
    polygon.addPoint(common::Point2d(pt3.x(), pt3.y()));
    polygon.addPoint(common::Point2d(pt4.x(), pt4.y()));

    return polygon;
}


void MainViewer::onOptimateSlot4() {
    auto local_pose_filer = _project->getLocalPoseFiler();
    common::Pose3d pose_trans = local_pose_filer->load();
    // Eigen::Matrix4d matrix_trans = pose_trans.toMatrix();
    ParklineOptimizer pl_optimizer;
    auto cloud_list_folder = _project->getOriginDataFolder()->getCloudListFolder();
    
    // 优化
    bool need_check = false;
    int index = 0;
    for (auto& group : _park_space_group_vec) {
        if (!group.valid) {continue;}
        std::map<Eigen::Vector2f, int, Vector2fCompare> corner_count_map;
        std::map<Eigen::Vector2f, CornerInfo, Vector2fCompare> corner_info_;  
        pcl::PointCloud<pcl::PointXYZ>::Ptr corner_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr optimize_cloud (new pcl::PointCloud<pcl::PointXYZ>);


        for (int i = 0; i < group.parkspaces.size(); ++i) {
            const auto& space = group.parkspaces[i];
            Eigen::Vector2f corner1 = Eigen::Vector2f(space.points[0].x, space.points[0].y);
            Eigen::Vector2f corner2 = Eigen::Vector2f(space.points[1].x, space.points[1].y);
            Eigen::Vector2f corner3 = Eigen::Vector2f(space.points[2].x, space.points[2].y);
            Eigen::Vector2f corner4 = Eigen::Vector2f(space.points[3].x, space.points[3].y);

            recordCorner(corner1, space, i, 0, corner_info_);
            recordCorner(corner2, space, i, 1, corner_info_);  
            recordCorner(corner3, space, i, 2, corner_info_);
            recordCorner(corner4, space, i, 3, corner_info_);

            corner_count_map[corner1]++;
            corner_count_map[corner2]++;
            corner_count_map[corner3]++;
            corner_count_map[corner4]++;
        }

        for (auto& space : group.parkspaces) {
            std::vector<int> l_corner_indices;
            int t_corner_index = -1;
            
            for (int i = 0; i < 4; ++i) {
                Eigen::Vector2f corner = Eigen::Vector2f(space.points[i].x, space.points[i].y);
                if (corner_count_map[corner] == 1) {
                    l_corner_indices.push_back(i);  // 记录L型角点
                } else if (corner_count_map[corner] == 2) {
                    t_corner_index = i;  // 记录T型角点
                }
            }

            if (l_corner_indices.size() == 3 && t_corner_index != -1) {
                // 找到T型角点相邻的两个L型角点
                int prev_index = (t_corner_index + 3) % 4;  // T型角点的前一个点
                int next_index = (t_corner_index + 1) % 4;  // T型角点的后一个点

                Eigen::Vector2f t_corner = Eigen::Vector2f(space.points[t_corner_index].x, space.points[t_corner_index].y);
                Eigen::Vector2f prev_corner = Eigen::Vector2f(space.points[prev_index].x, space.points[prev_index].y);
                Eigen::Vector2f next_corner = Eigen::Vector2f(space.points[next_index].x, space.points[next_index].y);

                // 计算距离
                float dist_prev = (prev_corner - t_corner).norm();
                float dist_next = (next_corner - t_corner).norm();

                // 将距离较远的点也赋为T型角点
                if (dist_prev > dist_next) {
                    corner_count_map[prev_corner] = 2;  // 将前一个点赋值为T型角点
                } else {
                    corner_count_map[next_corner] = 2;  // 将后一个点赋值为T型角点
                }
            }
        }

        for (const auto& [corner, count] : corner_count_map) {
            Eigen::Vector2f new_corner, trans;
            common::Point2d pt_corner, optimized_corner;
            common::Polygon2d polygon; 

            if (count == 1) {
                polygon = handleLCorner(corner, corner_info_, new_corner, trans);  // 处理L型角点
            } else if (count == 2) {
                polygon = handleTCorner(corner, corner_info_, corner_count_map, new_corner, trans);  // 处理T型角点
            } else if (count == 3 || count == 4) {
                polygon = handleMultiCorner(corner, corner_info_, new_corner);  // 处理3或4次出现的角点
            }
            auto cloud = cloud_list_folder->extractCloudInPolygon(polygon);
            // common::point_cloud::RangeZFilter<PointX>(*cloud, common::Rangef(0.5, 1.5)); // 小昆山
            common::point_cloud::RangeZFilter<PointX>(*cloud, common::Rangef(-1.3, 0.5));

            const auto& corner_info = corner_info_.at(corner);
            const Eigen::Vector2f& direction1 = corner_info.direction1;
            const Eigen::Vector2f& direction2 = corner_info.direction2;
            pt_corner = common::Point2d(new_corner.x(), new_corner.y());

            std::cout << "===========================================" << std::endl;
            std::cout << "Park Corner index: " << ++index << std::endl;

            pl_optimizer.optimate4(cloud, pt_corner, optimized_corner, direction1, direction2, need_check);

            pcl::PointXYZ pt;
            pt.x = pt_corner.x;
            pt.y = pt_corner.y;
            pt.z = 1;
            corner_cloud->push_back(pt);
            pt.x = optimized_corner.x;
            pt.y = optimized_corner.y;
            pt.z = 1;
            optimize_cloud->push_back(pt);

            if(count == 1 || count == 2) {
                optimized_corner.x -= trans.x();
                optimized_corner.y -= trans.y();   
            }
            std::cout << "source corner: (" << corner.x() << ", " << corner.y() << ")" << std::endl;
            std::cout << "optimized corner: (" << optimized_corner.x << ", " << optimized_corner.y << ")" << std::endl;

            group.parkspaces[corner_info.parkspace_id].points[corner_info.corner_id] = optimized_corner;

            for (auto& space : group.parkspaces) {
                for (auto& point : space.points) {
                    if (Eigen::Vector2f(point.x, point.y).isApprox(corner)) {
                        point.x = optimized_corner.x;
                        point.y = optimized_corner.y;
                    }
                }
            }
        }

        { // test
            // common::Polygon2d polygon; 
            // polygon.addPoint(group.parkspaces[0].points[0] + common::Point2d(10,10));
            // polygon.addPoint(group.parkspaces[0].points[0] + common::Point2d(-10,10));
            // polygon.addPoint(group.parkspaces[0].points[0] + common::Point2d(-10,-10));
            // polygon.addPoint(group.parkspaces[0].points[0] + common::Point2d(10,-10));
            // auto cloud = cloud_list_folder->extractCloudInPolygon(polygon);
            // common::point_cloud::RangeZFilter<PointX>(*cloud, common::Rangef(0.5, 1.5));
            // static int i = 0;
            // std::string save_dir = "/home/guitu/CLIP_data/corner/";
            // std::string cloud_path = save_dir + std::to_string(i) + "whole.pcd";
            // pcl::io::savePCDFile(cloud_path, *cloud);
            // cloud_path = save_dir + std::to_string(i) + "corner.pcd";
            // pcl::io::savePCDFile(cloud_path, *corner_cloud);
            // cloud_path = save_dir + std::to_string(i) + "optimize.pcd";
            // pcl::io::savePCDFile(cloud_path, *optimize_cloud);
            // ++i;
        }

        group.CornerRetifyParkSpace();
    }
    _opt_layer->removeAllItems();
    // 重新设置polygon
    for (const auto& group : _park_space_group_vec) {
        if (!group.valid) {continue;}
        for (const auto& ps : group.parkspaces) {
            QPolygonF pg = UTQ(ps.points);
            _opt_layer->addPolygon(pg);
        }
    }

    std::string outputFilePath = _file_system->getFineCornerParkingSlotsPath();
    std::ofstream outputFile(outputFilePath);
    std::set<std::tuple<double, double, double, double>> processed_edges;
    
    for (const auto& group : _park_space_group_vec) {
        if (!group.valid) continue;
        for (const auto& ps : group.parkspaces) {
            for (int i = 0; i < 4; ++i) {
                common::Point2d p1 = ps.points[i];
                common::Point2d p2 = ps.points[(i + 1) % 4];
                
                // 创建标准化的边表示（确保p1 < p2，便于检测重复）
                std::tuple<double, double, double, double> edge;
                if (p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y)) {
                    edge = std::make_tuple(p1.x, p1.y, p2.x, p2.y);
                } else {
                    edge = std::make_tuple(p2.x, p2.y, p1.x, p1.y);
                }
                
                // 检查这条边是否已经处理过
                if (processed_edges.find(edge) != processed_edges.end()) {
                    continue; 
                }                
                processed_edges.insert(edge);
                
                // 计算边的中心点和长度
                double center_x = (p1.x + p2.x) / 2.0;
                double center_y = (p1.y + p2.y) / 2.0;
                double length = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
                
                outputFile << p1.x << " " << p1.y << " " 
                          << p2.x << " " << p2.y << " "
                          << center_x << " " << center_y << " "
                          << length << "\n";
            }
        }
    }

    outputFile.close();
}

void MainViewer::onOptimateSlot4_2() {
    auto local_pose_filer = _project->getLocalPoseFiler();
    common::Pose3d pose_trans = local_pose_filer->load();
    ParklineOptimizer pl_optimizer;
    auto cloud_list_folder = _project->getOriginDataFolder()->getCloudListFolder();
    
    // 优化
    int max_iterations = 5; // 设置最大迭代次数
    double convergence_thres = 0.002; // 0.002mm 优化量阈值
    bool need_check = false;
    int index = 0;
    double pixel_size = 0.05;
    for (auto& group : _park_space_group_vec) {
        if (!group.valid) {continue;}
        std::map<Eigen::Vector2f, int, Vector2fCompare> corner_count_map;
        std::map<Eigen::Vector2f, CornerInfo, Vector2fCompare> corner_info_;  
        pcl::PointCloud<pcl::PointXYZ>::Ptr corner_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr optimize_cloud (new pcl::PointCloud<pcl::PointXYZ>);

        for (int i = 0; i < group.parkspaces.size(); ++i) {
            const auto& space = group.parkspaces[i];
            Eigen::Vector2f corner1 = Eigen::Vector2f(space.points[0].x, space.points[0].y);
            Eigen::Vector2f corner2 = Eigen::Vector2f(space.points[1].x, space.points[1].y);
            Eigen::Vector2f corner3 = Eigen::Vector2f(space.points[2].x, space.points[2].y);
            Eigen::Vector2f corner4 = Eigen::Vector2f(space.points[3].x, space.points[3].y);

            recordCorner(corner1, space, i, 0, corner_info_);
            recordCorner(corner2, space, i, 1, corner_info_);  
            recordCorner(corner3, space, i, 2, corner_info_);
            recordCorner(corner4, space, i, 3, corner_info_);

            corner_count_map[corner1]++;
            corner_count_map[corner2]++;
            corner_count_map[corner3]++;
            corner_count_map[corner4]++;
            
        }
        std::cout << "corner_count_map: " << corner_count_map.size() << std::endl;

        for (auto& space : group.parkspaces) {
            std::vector<int> l_corner_indices;
            int t_corner_index = -1;
            
            for (int i = 0; i < 4; ++i) {
                Eigen::Vector2f corner = Eigen::Vector2f(space.points[i].x, space.points[i].y);
                if (corner_count_map[corner] == 1) {
                    l_corner_indices.push_back(i);  // 记录L型角点
                } else if (corner_count_map[corner] == 2) {
                    t_corner_index = i;  // 记录T型角点
                }
            }

            if (l_corner_indices.size() == 3 && t_corner_index != -1) {
                // 找到T型角点相邻的两个L型角点
                int prev_index = (t_corner_index + 3) % 4;  // T型角点的前一个点
                int next_index = (t_corner_index + 1) % 4;  // T型角点的后一个点

                Eigen::Vector2f t_corner = Eigen::Vector2f(space.points[t_corner_index].x, space.points[t_corner_index].y);
                Eigen::Vector2f prev_corner = Eigen::Vector2f(space.points[prev_index].x, space.points[prev_index].y);
                Eigen::Vector2f next_corner = Eigen::Vector2f(space.points[next_index].x, space.points[next_index].y);

                // 计算距离
                float dist_prev = (prev_corner - t_corner).norm();
                float dist_next = (next_corner - t_corner).norm();

                // 将距离较远的点也赋为T型角点
                if (dist_prev > dist_next) {
                    corner_count_map[prev_corner] = 2;  // 将前一个点赋值为T型角点
                } else {
                    corner_count_map[next_corner] = 2;  // 将后一个点赋值为T型角点
                }
            }
        }

        for (const auto& [corner, count] : corner_count_map) {
            common::Point2d pt_corner, optimized_corner; 
            const auto& corner_info = corner_info_.at(corner);
            const Eigen::Vector2f& direction1 = corner_info.direction1;
            const Eigen::Vector2f& direction2 = corner_info.direction2;

            std::cout << "===========================================" << std::endl;
            std::cout << "Park Corner index: " << ++index << std::endl;

            // 根据 count 值确定角点类型
            int corner_type = 0;
            Eigen::Vector2f move_direction(0, 0);
            int move_direction_flag = 0;  

            if (count == 1) {
                corner_type = 1;  // L型角点
            } else if (count == 2) {
                corner_type = 2;  // T型角点
                
                Eigen::Vector2f next_point1 = corner_info.next_point1;
                Eigen::Vector2f next_point2 = corner_info.next_point2;
                
                if (corner_count_map.at(next_point1) > corner_count_map.at(next_point2)) {
                    move_direction = (next_point1 - corner).normalized();
                    move_direction_flag = 1;  
                } else if (corner_count_map.at(next_point1) < corner_count_map.at(next_point2)) {
                    move_direction = (next_point2 - corner).normalized();
                    move_direction_flag = 2;  
                } else if (corner_count_map.at(next_point1) == corner_count_map.at(next_point2)) {
                    float distance1 = (next_point1 - corner).norm(); 
                    float distance2 = (next_point2 - corner).norm(); 
                    if (distance1 > distance2) {
                        move_direction = (next_point1 - corner).normalized();
                        move_direction_flag = 1;  
                    } else {
                        move_direction = (next_point2 - corner).normalized();
                        move_direction_flag = 2;  
                    }
                }
            } else if (count == 3 || count == 4) {
                corner_type = 3;  
            }


            int iter_count = 0;
            bool converged = false;
            Eigen::Vector2f current_corner = corner;

            while (iter_count < max_iterations && !converged) {
                std::cout << "\tIteration: " << iter_count + 1 << std::endl;
                common::Polygon2d polygon;
                Eigen::Vector2f pt1, pt2, pt3, pt4;
                pt1 = current_corner + 0.6 * (direction1 + direction2);
                pt2 = current_corner + 0.6 * (direction1 - direction2);
                pt3 = current_corner - 0.6 * (direction1 + direction2);
                pt4 = current_corner - 0.6 * (direction1 - direction2);
                polygon.addPoint(common::Point2d(pt1.x(), pt1.y()));
                polygon.addPoint(common::Point2d(pt2.x(), pt2.y()));
                polygon.addPoint(common::Point2d(pt3.x(), pt3.y()));
                polygon.addPoint(common::Point2d(pt4.x(), pt4.y()));
                auto cloud = cloud_list_folder->extractCloudInPolygon(polygon);
                common::point_cloud::RangeZFilter<PointX>(*cloud, common::Rangef(0.5, 1.5)); // 小昆山
                // common::point_cloud::RangeZFilter<PointX>(*cloud, common::Rangef(-1.3, 0.5));

                std::map<std::pair<int, int>, std::vector<PointX>> pixel_map;
                for (const auto& point : *cloud) {
                    int pixel_x = static_cast<int>(point.x / pixel_size);
                    int pixel_y = static_cast<int>(point.y / pixel_size);
                    pixel_map[{pixel_x, pixel_y}].emplace_back(point); 
                }

                // 判断高度差并剔除非地面点
                float height_threshold = 0.1; // 设置阈值
                PointCloudPtr filtered_cloud(new PointCloud()); 

                for (const auto& entry : pixel_map) {
                    const auto& points = entry.second;
                    float min_height = std::numeric_limits<double>::max();
                    float max_height = std::numeric_limits<double>::lowest();

                    for (const auto& p : points) {
                        min_height = std::min(min_height, p.z);
                        max_height = std::max(max_height, p.z);
                    }

                    if (max_height - min_height <= height_threshold) {
                        for (const auto& p : points) {
                            filtered_cloud->emplace_back(p);
                        }
                    }
                }


                
                pl_optimizer.optimate4_2(filtered_cloud, common::Point2d(current_corner.x(), current_corner.y()), optimized_corner, direction1, direction2, need_check, corner_type, move_direction_flag);

                float delta_x = std::abs(optimized_corner.x - current_corner.x());
                float delta_y = std::abs(optimized_corner.y - current_corner.y());

                std::cout << "\t\tdelta: (" << delta_x << ", " << delta_y << ")" << std::endl;

                if (delta_x < convergence_thres && delta_y < convergence_thres) {
                    converged = true; 
                }

                current_corner = Eigen::Vector2f(optimized_corner.x, optimized_corner.y);

                iter_count++;
            }
            std::cout << "Source corner: (" << corner.x() << ", " << corner.y() << ")" << std::endl;
            std::cout << "Optimized corner: (" << optimized_corner.x << ", " << optimized_corner.y << ")" << std::endl;
            // pcl::PointXYZ pt;
            // pt.x = pt_corner.x;
            // pt.y = pt_corner.y;
            // pt.z = 1;
            // corner_cloud->push_back(pt);
            // pt.x = optimized_corner.x;
            // pt.y = optimized_corner.y;
            // pt.z = 1;
            // optimize_cloud->push_back(pt);


            group.parkspaces[corner_info.parkspace_id].points[corner_info.corner_id] = optimized_corner;

            for (auto& space : group.parkspaces) {
                for (auto& point : space.points) {
                    if (Eigen::Vector2f(point.x, point.y).isApprox(corner)) {
                        point.x = optimized_corner.x;
                        point.y = optimized_corner.y;
                    }
                }
            }
        }

        { // test
            // common::Polygon2d polygon; 
            // polygon.addPoint(group.parkspaces[0].points[0] + common::Point2d(10,10));
            // polygon.addPoint(group.parkspaces[0].points[0] + common::Point2d(-10,10));
            // polygon.addPoint(group.parkspaces[0].points[0] + common::Point2d(-10,-10));
            // polygon.addPoint(group.parkspaces[0].points[0] + common::Point2d(10,-10));
            // auto cloud = cloud_list_folder->extractCloudInPolygon(polygon);
            // // common::point_cloud::RangeZFilter<PointX>(*cloud, common::Rangef(0.5, 1.5));
            // common::point_cloud::RangeZFilter<PointX>(*cloud, common::Rangef(-1.3, 0.5));
            // static int i = 0;
            // std::string save_dir = "/home/guitu/CLIP_data/corner/";
            // std::string cloud_path = save_dir + std::to_string(i) + "whole.pcd";
            // pcl::io::savePCDFile(cloud_path, *cloud);
            // cloud_path = save_dir + std::to_string(i) + "corner.pcd";
            // pcl::io::savePCDFile(cloud_path, *corner_cloud);
            // cloud_path = save_dir + std::to_string(i) + "optimize.pcd";
            // pcl::io::savePCDFile(cloud_path, *optimize_cloud);
            // ++i;
        }

        group.CornerRetifyParkSpace();
    }
    _opt_layer->removeAllItems();
    // 重新设置polygon
    for (const auto& group : _park_space_group_vec) {
        if (!group.valid) {continue;}
        for (const auto& ps : group.parkspaces) {
            QPolygonF pg = UTQ(ps.points);
            _opt_layer->addPolygon(pg);
        }
    }

    std::string outputFilePath = _file_system->getFineCornerParkingSlotsPath();
    std::ofstream outputFile(outputFilePath);
    std::set<std::tuple<double, double, double, double>> processed_edges;
    
    for (const auto& group : _park_space_group_vec) {
        if (!group.valid) continue;
        for (const auto& ps : group.parkspaces) {
            for (int i = 0; i < 4; ++i) {
                common::Point2d p1 = ps.points[i];
                common::Point2d p2 = ps.points[(i + 1) % 4];
                
                // 创建标准化的边表示（确保p1 < p2，便于检测重复）
                std::tuple<double, double, double, double> edge;
                if (p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y)) {
                    edge = std::make_tuple(p1.x, p1.y, p2.x, p2.y);
                } else {
                    edge = std::make_tuple(p2.x, p2.y, p1.x, p1.y);
                }
                
                // 检查这条边是否已经处理过
                if (processed_edges.find(edge) != processed_edges.end()) {
                    continue; 
                }                
                processed_edges.insert(edge);
                
                // 计算边的中心点和长度
                double center_x = (p1.x + p2.x) / 2.0;
                double center_y = (p1.y + p2.y) / 2.0;
                double length = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
                
                outputFile << p1.x << " " << p1.y << " " 
                          << p2.x << " " << p2.y << " "
                          << center_x << " " << center_y << " "
                          << length << "\n";
            }
        }
    }

    outputFile.close();
}




void MainViewer::onAddPolygonSlot() {
    auto polygon_draw_tool = _draw_toolbar->getPolygonDrawTool();
    _shape_layer->addPolygon(polygon_draw_tool->polygon());
}

void MainViewer::onAddRectSlot() {
    auto rect_draw_tool = _draw_toolbar->getRectDrawTool();
    QPolygonF polygon(rect_draw_tool->rect());
    polygon.pop_back(); // 最后一个点与第一个点重合，删除
    _shape_layer->addPolygon(polygon);
}
void MainViewer::createBaseMapLayer() {
    // 底图图层
    _raster_layer = new ucanvas::RasterImageLayer(this);
    _raster_layer->setName("raster");
    _raster_layer->setText(tr("Raster"));
    _raster_layer->setZValue(-1.0);
    this->addLayer(_raster_layer);

    // 设置画笔
    QPen pen;
    pen.setColor(Qt::red);
    pen.setWidthF(5);
    pen.setCosmetic(true);
    QBrush brush(Qt::NoBrush);
    _shape_layer = new ucanvas::GenericLayer(this);
    _shape_layer->setName("shape");
    _shape_layer->setText(tr("Shape"));
    _shape_layer->setPen(pen);
    _shape_layer->setBrush(brush);
    this->addLayer(_shape_layer);

    pen.setColor(Qt::green);
    pen.setWidthF(5.0);
    pen.setCosmetic(true);
    _opt_layer = new ucanvas::GenericLayer(this);
    _opt_layer->setName("Opt");
    _opt_layer->setText(tr("Opt"));
    _opt_layer->setPen(pen);
    _opt_layer->setBrush(brush);
    this->addLayer(_opt_layer);
}
void MainViewer::createDrawToolBar() {
    // 增加绘制工具栏
    _draw_toolbar = new ucanvas::DrawToolBar(this);
    _draw_toolbar->setViewer(this);
    // // 将线绘制、成组和解组按钮隐藏
    // _draw_toolbar->setActionVisible(ucanvas::LineDrawKey, false);
    // _draw_toolbar->setActionVisible(ucanvas::GroupKey, false);
    // _draw_toolbar->setActionVisible(ucanvas::UnGroupKey, false);
    // _draw_toolbar->setActionActived(ucanvas::SelectKey, true);
    QStringList keys;
    keys << ucanvas::RectDrawKey << ucanvas::PolygonDrawKey
        << ucanvas::SelectKey << ucanvas::MoveKey
        << ucanvas::RotateKey << ucanvas::ScaleKey
        << ucanvas::EditKey << ucanvas::DeleteKey;
    _draw_toolbar->setActionsVisible(keys);
    _draw_toolbar->setActionActived(ucanvas::SelectKey, true);

    // 建立绘制工具的信号-槽
    auto polygon_draw_tool = _draw_toolbar->getPolygonDrawTool();
    connect(polygon_draw_tool, SIGNAL(finished()), this, SLOT(onAddPolygonSlot()));
    auto rect_draw_tool = _draw_toolbar->getRectDrawTool();
    connect(rect_draw_tool, SIGNAL(finished()), this, SLOT(onAddRectSlot()));
}
}
