#pragma once
#include "parkspace.h"
#include "parkline.h"
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace welkin::bamboo {
class ParkSpaceGroup {
public:
    ParkSpaceGroup();
    virtual ~ParkSpaceGroup();
    bool isIntersect(const ParkSpace& parkspace) const;
    void addParkSpace(const ParkSpace& parkspace);
    common::Rectd getBoudingRect() const;

    common::Line2d getLine(int index) const;
    const ParkSpace& getParkSpace(int index) const;
    void computeLongSideDirection();
    // 计算库位线
    void computeParkLines();
    // 拟合
    // 0 - 得分过低，不拟合
    // 1 - 得分低，拟合结果置信度低
    // 2 - 得分较高，拟合结果置信度高
    int fit(const cv::Mat& image, const common::Point2d& origin, 
        double thresh1 = 0.4, double thresh2 = 0.6);
    // 边缘检测
    void edgeDetect(const cv::Mat& image, const common::Point2d& origin, 
        float thresh1 = 60, float thresh2 = 100);
    // 改正库位
    void retifyParkSpace();
    // 库位
    std::vector<ParkSpace> parkspaces;
    // 库位线
    std::vector<ParkLine> parklines;
    // 长线方向
    common::Point2d long_side_direction;
    bool valid = true;
    // 分辨率
    static double res;

    using PointT = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<PointT>;
private:
    void drawTemplate(cv::Mat& templ);
    void matchTemplate(const cv::Mat& image, cv::Mat& result);
    void translate(const common::Point2d& offset);
    double getValidPixelRatio(const cv::Mat& image) const;
    std::vector<cv::Point> getCvPolygon(
        const common::Polygon2d& polygon, const common::Point2d& origin) const;
    
    // 
    void convertImageToCloud(const cv::Mat& image, 
        const common::Point2d& origin, PointCloud& cloud) const;
    void convertImageToCloud(const cv::Mat& image, 
        const cv::Mat& rotateImage, const Eigen::Matrix3f& rotateMatrix,
        const common::Point2d& origin, PointCloud& cloud) const;
    
    // 将二值图转化为点云
    void convertBinImageToCloud(const cv::Mat& binary, PointCloud& cloud) const;
    void convertBinImageToCloud(const cv::Mat& binary, PointCloud& cloud,
        const common::Point2d& origin, double res = 0.02) const;
    // 点云缩放
    void scaleCloud(PointCloud& cloud, 
        double scalex = 0.5, double scaley = 0.5, double scalez = 1.0) const;
    // 坐标转换
    void transformLine2d(common::Line2d& line, const Eigen::Matrix3d& matrix) const;
    Eigen::Matrix3d getScaleMatrix2D(const common::Point2d& origin, 
        double scalex = 0.02, double scaley = 0.02) const;
    // 点云提取
    void extractCloudByLine2d(
        const PointCloud::Ptr& cloud_in, PointCloud::Ptr& cloud_out, 
        const common::Line2d& line, const common::Ranged& dist_range) const;
private:
    std::vector<int> parkline_indexs; // 库位线列表
};
}