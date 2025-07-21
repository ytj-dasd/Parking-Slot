#include "parkspacegroup.h"
#include "parkline.h"
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include "bamboo/utils/canny.h"
#include "bamboo/utils/cloud_algorithm.h"
#include "bamboo/utils/image_algorithm.h"
#include "bamboo/utils/segmentation.h"
#include <pcl/io/pcd_io.h>
#include <QDebug>

namespace welkin::bamboo {
// 库位线关系(共三种)
enum LineRelationType {
    TypeIndepend = 0,  // 无关
    TypeOverlap = 1,   // 重叠
    TypeExtension = 2  // 延长线关系
};
double epsilon = 0.08; // 1cm的限差
double iou_thresh = 0.2; // 若为重叠关系，则iou至少大于0.2
// 判断两条直线的关系
LineRelationType computeLineRelation(const common::Line2d& l1, const common::Line2d& l2) {
    auto proj1 = l1.getProjectPoint(l2.begin_point);
    auto proj2 = l1.getProjectPoint(l2.end_point);
    // 两条线无关
    if (proj1.distanceTo(l2.begin_point) > epsilon 
            || proj2.distanceTo(l2.end_point) > epsilon) {
        return TypeIndepend;
    }
    const auto& origin = l1.begin_point;
    auto dir = l1.getDirection();
    double r1 = 0.0; 
    double r2 = getLineRatio(origin, dir, l1.end_point);
    double min_v = std::min(r1, r2); 
    double max_v = std::max(r1, r2);
    common::Ranged range1(min_v, max_v);
    r1 = getLineRatio(origin, dir, l2.begin_point);
    r2 = getLineRatio(origin, dir, l2.end_point);
    min_v = std::min(r1, r2);
    max_v = std::max(r1, r2);
    common::Ranged range2(min_v, max_v);
    return (range1.IOU(range2) > iou_thresh) ? TypeOverlap : TypeExtension;
}

double ParkSpaceGroup::res = 0.02;
ParkSpaceGroup::ParkSpaceGroup() {}
ParkSpaceGroup::~ParkSpaceGroup() {}

void ParkSpaceGroup::addParkSpace(const ParkSpace& parkspace) {
    parkspaces.push_back(parkspace);
}
bool ParkSpaceGroup::isIntersect(const ParkSpace& parkspace) const {
    for (const auto& ps : parkspaces) {
        if (parkspace.isIntersect(ps)) {return true;}
    }
    return false;
}
bool ParkSpaceGroup::isIntersectImage(const ParkSpace& parkspace, bool is_matched) const {
    for (const auto& ps : parkspaces) {
        if (is_matched) {
            if (parkspace.isIntersectImage(ps, true)) {return true;}
        } else {
            if (parkspace.isIntersectImage(ps)) {return true;}
        }
    }
    return false;
}
common::Rectd ParkSpaceGroup::getBoudingRect() const {
    common::Rectd rect;
    for (const auto& parkspace : parkspaces) {
        rect.fitRect(parkspace.getBoundingRect());
    }
    return rect;
}
common::Line2d ParkSpaceGroup::getLine(int index) const {
    int space_index = index / 4;
    int line_index = index % 4;
    return parkspaces[space_index].getLine(line_index);
}
const ParkSpace& ParkSpaceGroup::getParkSpace(int index) const {
    return parkspaces.at(index);
}
void ParkSpaceGroup::computeLongSideDirection() {
    double epsilon = 0.9;
    std::vector<common::Point2d> dirs;
    std::vector<int> nums;
    bool flag = false;
    for (auto& ps : parkspaces) {
        flag = false;
        auto dir = ps.getLongSideDirection();
        for (int i = 0; i < dirs.size(); ++i) {
            if (std::abs(dirs[i].dot(dir)) > epsilon) {
                ++nums[i];
                flag = true;
            }
        }
        if (!flag) {
            dirs.push_back(dir);
            nums.push_back(1);
        }
    }
    auto max_iter = std::max_element(nums.begin(), nums.end());
    int max_index = std::distance(nums.begin(), max_iter);
    long_side_direction = dirs[max_index];
}
// 计算库位线
void ParkSpaceGroup::computeParkLines() {
    parklines.clear();
    parkline_indexs.resize(parkspaces.size() * 4, -1);
    std::vector<bool> flags(parkspaces.size() * 4, false);
    for (int i = 0; i < parkspaces.size(); ++i) {
        auto& ps1 = parkspaces[i];
        for (int j = i + 1; j < parkspaces.size(); ++j) {
            auto& ps2 = parkspaces[j];
            for (int m = 0; m < ps1.size(); ++m) {
                common::Line2d l1 = ps1.getLine(m);
                for (int n = 0; n < ps2.size(); ++n) {
                    common::Line2d l2 = ps2.getLine(n);
                    auto relation = computeLineRelation(l1, l2);
                    if (relation == TypeIndepend) {continue;}
                    int index1 = i * 4 + m; int index2 = j * 4 + n;
                    // 设置标识为真
                    flags[index1] = true; 
                    flags[index2] = true;
                    bool has_added = false;
                    for (int k = 0; k < parklines.size(); ++k) {
                        auto& parkline = parklines[k];
                        if (!parkline.hasIndex(index1) && !parkline.hasIndex(index2)) {
                            continue;
                        }
                        if (parkline.hasIndex(index1)) {
                            parkline.addIndex(index2);
                        } else if (parkline.hasIndex(index2)) {
                            parkline.addIndex(index1);
                        }
                        has_added = true;
                        parkline_indexs[index1] = k;
                        parkline_indexs[index2] = k;
                        // 修改库位线类型
                        if (parkline.type == ParkLine::TypeBorder && relation == TypeOverlap) {
                            parkline.type = ParkLine::TypeMiddle;
                        }
                        break;
                    }
                    if (!has_added) {
                        ParkLine pl(this);
                        pl.type = (relation == TypeOverlap ? ParkLine::TypeMiddle : ParkLine::TypeBorder);
                        pl.addIndex(index1);
                        pl.addIndex(index2);
                        parklines.push_back(pl);
                        parkline_indexs[index1] = parklines.size() - 1;
                        parkline_indexs[index2] = parklines.size() - 1;
                    }
                }
            }
        }
    }
    for (int i = 0; i < flags.size(); ++i) {
        if (flags[i]) {continue;}
        ParkLine pl(this);
        pl.type = ParkLine::TypeBorder;
        pl.addIndex(i);
        parklines.push_back(pl);
        parkline_indexs[i] = parklines.size() - 1;
    }
    // 计算
    for (auto& pl : parklines) {
        pl.computeCenterLine();
        pl.computeBorderLine();
        // 方向内积大于0.5(即方向夹角小于45度)
        if (std::abs(pl.center_line.getDirection().dot(long_side_direction)) > 0.5) {
            pl.is_long_side = true;
        } else {
            pl.is_long_side = false;
        }
    }
}
int ParkSpaceGroup::fit(const cv::Mat& image, const common::Point2d& origin, double thresh1, double thresh2) {
    // 如果有效像素比例小于0.5，则相当于匹配失败(不满足匹配条件)
    if (getValidPixelRatio(image) < 0.5) {return -1;}
    cv::Mat result;
    matchTemplate(image, result);
    double dMaxVal; //分数最大值
	cv::Point ptMaxLoc; //最大值坐标
	cv::minMaxLoc(result, 0, &dMaxVal, 0, &ptMaxLoc); //寻找结果矩阵中的最大值
    if (dMaxVal < thresh1) {return 0;} // 匹配得分很低
    auto brect = this->getBoudingRect();
    auto borigin = brect.getTopLeftPoint();
    common::Point2d offset;
    offset.x = ptMaxLoc.x * res;
    offset.y = ptMaxLoc.y * res;
    offset = origin + offset - borigin;
    translate(offset);
    return dMaxVal < thresh2 ? 1 : 2;
}

void ParkSpaceGroup::edgeDetect(
        const cv::Mat& image, const common::Point2d& origin, 
        float thresh1, float thresh2) {
    // 求得旋转角度(短线沿X轴，长线沿Y轴)
    double angle = std::atan2(-long_side_direction.x, long_side_direction.y) * 180.0 / M_PI;
    cv::Point2f center(image.cols * 0.5, image.rows * 0.5);
    cv::Mat rot_image;
    // 图像旋转 (为了后续能够沿X轴和Y轴做不同比例缩放，先转正)
    cv::Mat cv_rot_matrix = Rotate(image, rot_image, center, angle);
    Eigen::Matrix<float, 2, 3> matrix23;
    cv::cv2eigen(cv_rot_matrix, matrix23);
    Eigen::Matrix3d rot_matrix; // 采用Eigen形式表达旋转矩阵
    rot_matrix.setIdentity();  // Note: 注意初始将rot_matrix设置为单位阵
    rot_matrix.block(0, 0, 2, 3) = matrix23.cast<double>();
    
    // 拉伸比例关系
    double stretch_ratio = 3.0; 
    // 根据库位线宽确定直线拟合点范围: lineWidth * 0.5 + padding
    double delta = 0.18 * 0.5 + 0.08;
    // 库位线宽
    common::Ranged line_width_range(0.1, 0.25);
    // 原始图像坐标系到世界坐标系的变换关系
    Eigen::Matrix3d matrix1 = getScaleMatrix2D(origin, res, res);
    int inliers_nums = 50; // 内点个数需大于50
    
    ///// 长线检测 /////
    cv::Mat edge;
    Canny(rot_image, edge, thresh1, thresh2, EDGE_ANGLE_90, 3, true);

    // cv::Mat mat_show;
    // cv::cvtColor(rot_image, mat_show, cv::COLOR_GRAY2BGR);
    // { // 测试
    //     static int i = 0;
    //     cv::imwrite("/home/scene/guitu/data/canny/" + std::to_string(i) + ".jpg", rot_image);
    //     cv::imwrite("/home/scene/guitu/data/canny/" + std::to_string(i) + "_edge_ver.jpg", edge);
    //     for (int r = 0; r < mat_show.rows; ++r) {
    //         for (int c= 0; c < mat_show.cols; ++c) {
    //             if (edge.at<uchar>(r, c) == 255) {
    //                 mat_show.at<cv::Vec3b>(r, c)[0] = 0;
    //                 mat_show.at<cv::Vec3b>(r, c)[1] = 0;
    //                 mat_show.at<cv::Vec3b>(r, c)[2] = 255;
    //             }
    //         }
    //     }
    //     ++i;
    // }
    // 设置最终检测时的比例关系
    double scalex = res * stretch_ratio; double scaley = res;
    // 图像转点云
    PointCloud::Ptr cloud(new PointCloud());
    convertBinImageToCloud(edge, *cloud);
    // 点云缩放
    scaleCloud(*cloud, scalex, scaley);
    // 旋转后图像坐标系到分割点云坐标系的变换关系
    Eigen::Matrix3d matrix2 = getScaleMatrix2D(
        common::Point2d(0.0, 0.0), scalex, scaley);
    // 世界坐标系到分割点云坐标系的变换关系
    Eigen::Matrix3d trans_matrix = matrix2 * rot_matrix * matrix1.inverse();
    Eigen::Matrix3d trans_matrix_inv = trans_matrix.inverse();
    
    // 取出线周围点
    PointCloud::Ptr cloud_left(new PointCloud());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    common::Line2d center_line, line1, line2;
    for (auto& pl : parklines) {
        if (!pl.is_long_side) {continue;} // 如果非长线，跳过
        center_line = pl.center_line;
        transformLine2d(center_line, trans_matrix);
        extractCloudByLine2d(cloud, cloud_left, center_line, common::Ranged(0.0, delta * stretch_ratio));
        if (cloud_left->size() < inliers_nums) {continue;}
        // 拟合第一条直线
        if (!ParallelLine2dSACSegmentation<PointT>(cloud_left, center_line, *inliers, line1, 0.05, 5.0, 200) 
            || inliers->indices.size() < inliers_nums) {continue;}

        ExtractCloud<PointT>(cloud_left, cloud_left, inliers, true);
        extractCloudByLine2d(cloud_left, cloud_left, line1, line_width_range * stretch_ratio);
        if (cloud_left->size() < inliers_nums) {continue;}
        // 拟合第二条直线
        if (!ParallelLine2dSACSegmentation<PointT>(cloud_left, line1, *inliers, line2, 0.05, 1.0, 200)
            || inliers->indices.size() < inliers_nums) {continue;}
        
        transformLine2d(line1, trans_matrix_inv);
        transformLine2d(line2, trans_matrix_inv); 
        line1.begin_point = line1.getProjectPoint(pl.center_line.begin_point);
        line1.end_point = line1.getProjectPoint(pl.center_line.end_point);
        line2.begin_point = line2.getProjectPoint(pl.center_line.begin_point);
        line2.end_point = line2.getProjectPoint(pl.center_line.end_point);
        // 线宽检核
        double delta1 = line1.begin_point.distanceTo(line2.begin_point);
        double delta2 = line1.end_point.distanceTo(line2.end_point);
        if (line_width_range.isIn(delta1) && line_width_range.isIn(delta2) 
                && std::fabs(delta1 - delta2) < 0.05) {
            // 利用边线改正库位线
            pl.fit_success = true;
            pl.retify(line1, line2);
        }
    }


    ///// 短线检测 /////
    Canny(rot_image, edge, thresh1, thresh2, EDGE_ANGLE_0, 3, true);
    // { // 测试
    //     static int j = 0;
    //     cv::imwrite("/home/scene/guitu/data/canny/" + std::to_string(j) + "_edge_hor.jpg", edge);
    //     for (int r = 0; r < mat_show.rows; ++r) {
    //         for (int c= 0; c < mat_show.cols; ++c) {
    //             if (edge.at<uchar>(r, c) == 255) {
    //                 mat_show.at<cv::Vec3b>(r, c)[0] = 0;
    //                 mat_show.at<cv::Vec3b>(r, c)[1] = 0;
    //                 mat_show.at<cv::Vec3b>(r, c)[2] = 255;
    //             }
    //         }
    //     }
    //     cv::imwrite("/home/scene/guitu/data/canny/" + std::to_string(j) + "_edge.jpg", mat_show);
    //     ++j;
    // }
    // 设置最终检测时的比例关系
    scalex = res; scaley = res * stretch_ratio;
    // 图像转点云
    cloud->clear();
    convertBinImageToCloud(edge, *cloud);
    // 点云缩放
    scaleCloud(*cloud, scalex, scaley);
    // 旋转后图像坐标系到分割点云坐标系的变换关系
    matrix2 = getScaleMatrix2D(common::Point2d(0.0, 0.0), scalex, scaley);
    // 世界坐标系到分割点云坐标系的变换关系
    trans_matrix = matrix2 * rot_matrix * matrix1.inverse();
    trans_matrix_inv = trans_matrix.inverse();
    
    // 取出线周围点
    for (auto& pl : parklines) {
        if (pl.is_long_side) {continue;} // 如果非短线，跳过
        center_line = pl.center_line;
        transformLine2d(center_line, trans_matrix);
        extractCloudByLine2d(cloud, cloud_left, center_line, common::Ranged(0.0, delta * stretch_ratio));
        if (cloud_left->size() < inliers_nums) {continue;}
        // 拟合第一条直线
        if (!ParallelLine2dSACSegmentation<PointT>(cloud_left, center_line, *inliers, line1, 0.05, 5.0, 200) 
            || inliers->indices.size() < inliers_nums) {continue;}

        ExtractCloud<PointT>(cloud_left, cloud_left, inliers, true);
        extractCloudByLine2d(cloud_left, cloud_left, line1, line_width_range * stretch_ratio);
        if (cloud_left->size() < inliers_nums) {continue;}
        // 拟合第二条直线
        if (!ParallelLine2dSACSegmentation<PointT>(cloud_left, line1, *inliers, line2, 0.05, 1.0, 200)
            || inliers->indices.size() < inliers_nums) {continue;}
        
        transformLine2d(line1, trans_matrix_inv);
        transformLine2d(line2, trans_matrix_inv); 
        pl.fit_success = true;
        line1.begin_point = line1.getProjectPoint(pl.center_line.begin_point);
        line1.end_point = line1.getProjectPoint(pl.center_line.end_point);
        line2.begin_point = line2.getProjectPoint(pl.center_line.begin_point);
        line2.end_point = line2.getProjectPoint(pl.center_line.end_point);
        // 线宽检核
        double delta1 = line1.begin_point.distanceTo(line2.begin_point);
        double delta2 = line1.end_point.distanceTo(line2.end_point);
        if (line_width_range.isIn(delta1) && line_width_range.isIn(delta2) 
                && std::fabs(delta1 - delta2) < 0.05) {
            // 利用边线改正库位线
            pl.fit_success = true;
            pl.retify(line1, line2);
            // std::cout << "pl linewidth: " << pl.line_width << std::endl;;
        }
    }

}
void ParkSpaceGroup::retifyParkSpace() {
    auto getLine = [this](int index) -> common::Line2d {
        const auto& pl0 = parklines[parkline_indexs[index]];
        return pl0.type == ParkLine::TypeBorder ? pl0.border_line : pl0.center_line;
    };
    for (int i = 0; i < parkspaces.size(); ++i) {
        auto line0 = getLine(i * 4);
        auto line1 = getLine(i * 4 + 1);
        auto line2 = getLine(i * 4 + 2);
        auto line3 = getLine(i * 4 + 3);
        parkspaces[i].points[0] = line3.getCrossPoint(line0);
        parkspaces[i].points[1] = line0.getCrossPoint(line1);
        parkspaces[i].points[2] = line1.getCrossPoint(line2);
        parkspaces[i].points[3] = line2.getCrossPoint(line3);
        for (int j = 0; j < 4; ++j) { // 四舍五入
            parkspaces[i].points[j].x = std::round(parkspaces[i].points[j].x * 1000) / 1000.0;
            parkspaces[i].points[j].y = std::round(parkspaces[i].points[j].y * 1000) / 1000.0;
        }
    }
    for (auto& pl : parklines) {
        // if (!pl.fit_success) {continue;}
        pl.computeCenterLine();
        
        // TODO: 后续修改改正方法
        pl.border_line1.begin_point = pl.border_line1.getProjectPoint(pl.center_line.begin_point);
        pl.border_line1.end_point = pl.border_line1.getProjectPoint(pl.center_line.end_point);
        pl.border_line2.begin_point = pl.border_line2.getProjectPoint(pl.center_line.begin_point);
        pl.border_line2.end_point = pl.border_line2.getProjectPoint(pl.center_line.end_point);
        pl.retify(pl.border_line1, pl.border_line2);
    }
}

void ParkSpaceGroup::CornerRetifyParkSpace() {
    for (auto& pl : parklines) {
        pl.computeCenterLine();
        
        pl.border_line1.begin_point = pl.border_line1.getProjectPoint(pl.center_line.begin_point);
        pl.border_line1.end_point = pl.border_line1.getProjectPoint(pl.center_line.end_point);
        pl.border_line2.begin_point = pl.border_line2.getProjectPoint(pl.center_line.begin_point);
        pl.border_line2.end_point = pl.border_line2.getProjectPoint(pl.center_line.end_point);
        pl.retify(pl.border_line1, pl.border_line2);
    }
}

void ParkSpaceGroup::drawTemplate(cv::Mat& templ) {
    auto brect = this->getBoudingRect();
    auto origin = brect.getTopLeftPoint();
    int image_width = std::ceil(brect.getLengthX() / res);
    int image_height = std::ceil(brect.getLengthY() / res);
    templ = cv::Mat::zeros(image_height, image_width, CV_8UC1);
    std::vector<common::Polygon2d> polygons;
    common::Polygon2d polygon;
    for (const auto& pl : parklines) {
        auto line1 = pl.center_line.getParallelLine(-pl.line_width * 0.5);
        auto line2 = pl.center_line.getParallelLine(pl.line_width * 0.5);
        polygon.clear();
        polygon.addPoint(line1.begin_point);
        polygon.addPoint(line1.end_point);
        polygon.addPoint(line2.end_point);
        polygon.addPoint(line2.begin_point);
        polygons.push_back(polygon);
    }
    for (const auto& polygon : polygons) {
        auto cvpolygon = getCvPolygon(polygon, origin);
        cv::fillPoly(templ, cvpolygon, cv::Scalar(255));
    }
}
void ParkSpaceGroup::matchTemplate(const cv::Mat& image, cv::Mat& result) {
    cv::Mat templ;
    this->drawTemplate(templ);
    // 二值化
    cv::Mat otsu;
    cv::threshold(image, otsu, 128, 255, cv::THRESH_OTSU | cv::THRESH_BINARY);
    result = cv::Mat(otsu.rows - templ.rows + 1, otsu.cols - templ.cols + 1, CV_32FC1); //构建结果矩阵
	cv::matchTemplate(otsu, templ, result, cv::TM_CCOEFF_NORMED); //模板匹配
}
void ParkSpaceGroup::translate(const common::Point2d& offset) {
    for (auto& ps : parkspaces) {
        ps.translate(offset);
    }
    for (auto& pl : parklines) { // 重新计算库位线
        pl.computeCenterLine();
        pl.computeBorderLine();
    }
}
double ParkSpaceGroup::getValidPixelRatio(const cv::Mat& image) const {
    int sum = 0;
    for (int r = 0; r < image.rows; ++r) {
        for (int c = 0; c < image.cols; ++c) {
            if (image.at<uchar>(r, c) != 0) {
                ++sum;
            }
        }
    }
    return sum * 1.0 / (image.rows * image.cols);
}
std::vector<cv::Point> ParkSpaceGroup::getCvPolygon(
        const common::Polygon2d& polygon, const common::Point2d& origin) const {
    std::vector<cv::Point> cvpolygon;
    for (auto& pt : polygon.points) {
        int x = static_cast<int>((pt.x - origin.x) / res);
        int y = static_cast<int>((pt.y - origin.y) / res);
        cvpolygon.push_back(std::move(cv::Point(x, y)));
    }
    return cvpolygon;
}
void ParkSpaceGroup::convertImageToCloud(const cv::Mat& image, 
        const common::Point2d& origin, PointCloud& cloud) const {
    cloud.clear();
    for (int r = 0; r < image.rows; ++r) {
        for (int c = 0; c < image.cols; ++c) {
            if (image.at<uchar>(r, c) == 255) {
                auto x = origin.x + c * res;
                auto y = origin.y + r * res;
                cloud.push_back(PointT(x, y, 0.0));
            }
        }
    }
}
void ParkSpaceGroup::convertImageToCloud(const cv::Mat& image, 
        const cv::Mat& rotateImage, const Eigen::Matrix3f& rotateMatrix,
        const common::Point2d& origin, PointCloud& cloud) const {
    cloud.clear();
    for (int r = 0; r < rotateImage.rows; ++r) {
        for (int c = 0; c < rotateImage.cols; ++c) {
            if (rotateImage.at<uchar>(r, c) == 255) {
                Eigen::Vector3f tgt = rotateMatrix * Eigen::Vector3f(c, r, 1.0);
                float imgX = tgt[0]; float imgY = tgt[1];
                if (imgX < 0 || imgX >= image.cols 
                        || imgY < 0 || imgY >= image.rows) {
                    continue;
                }
                auto x = origin.x + imgX * res;
                auto y = origin.y + imgY * res;
                cloud.push_back(PointT(x, y, 0.0));
            }
        }
    }
}

void ParkSpaceGroup::convertBinImageToCloud(
        const cv::Mat& binary, PointCloud& cloud) const {
    cloud.clear();
    for (int r = 0; r < binary.rows; ++r) {
        for (int c = 0; c < binary.cols; ++c) {
            if (binary.at<uchar>(r, c) == 255) {
                double x = static_cast<double>(c);
                double y = static_cast<double>(r);
                cloud.push_back(PointT(x, y, 0.0));
            }
        }
    }
}
void ParkSpaceGroup::convertBinImageToCloud(
        const cv::Mat& binary, PointCloud& cloud,
        const common::Point2d& origin, double res) const {
    cloud.clear();
    for (int r = 0; r < binary.rows; ++r) {
        for (int c = 0; c < binary.cols; ++c) {
            if (binary.at<uchar>(r, c) == 255) {
                auto x = origin.x + c * res;
                auto y = origin.y + r * res;
                cloud.push_back(PointT(x, y, 0.0));
            }
        }
    }
}
void ParkSpaceGroup::scaleCloud(
        PointCloud& cloud, double scalex, double scaley, double scalez) const {
    for (auto& pt : cloud.points) {
        pt.x *= scalex;
        pt.y *= scaley;
        pt.z *= scalez;
    }
}
void ParkSpaceGroup::transformLine2d(
        common::Line2d& line, const Eigen::Matrix3d& matrix) const {
    line.begin_point.transform(matrix);
    line.end_point.transform(matrix);
}

Eigen::Matrix3d ParkSpaceGroup::getScaleMatrix2D(
        const common::Point2d& origin, double scalex, double scaley) const {
    Eigen::Matrix3d scale_matrix;
    scale_matrix.setIdentity();
    scale_matrix(0, 0) *= scalex;
    scale_matrix(1, 1) *= scaley;
    scale_matrix(0, 2) = origin.x;
    scale_matrix(1, 2) = origin.y;
    return scale_matrix;
}
void ParkSpaceGroup::extractCloudByLine2d(
        const PointCloud::Ptr& cloud_in, PointCloud::Ptr& cloud_out, 
        const common::Line2d& line, const common::Ranged& dist_range) const {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    common::Point2d point, proj;
    for (int i = 0; i < cloud_in->size(); ++i) {
        const auto& pt = cloud_in->points.at(i);
        point = common::Point2d(pt.x, pt.y);
        proj = line.getProjectPoint(point);
        if (dist_range.isIn(proj.distanceTo(point))) {
            inliers->indices.push_back(i);
        }
    }
    ExtractCloud<PointT>(cloud_in, cloud_out, inliers, false);
}
}
