#include "origin_data_folder.h"
#include "common/math/solver.h"

namespace welkin::bamboo {
//////////////
void OriginDataCloudListFolder::applyPose(const common::Pose3d& delta_pose) {
    std::size_t child_count = this->getChildSize();
    for (std::size_t i = 0; i < child_count; ++i) {
        UProgressTextValue(QObject::tr("Process..."), i, child_count);
        auto child = this->getChild(i);
        child->applyPose(delta_pose);
        child->reset();
    }
}
common::Boxd OriginDataCloudListFolder::getBoundingBox() {
    std::size_t child_count = this->getChildSize();
    common::Boxd bbox;
    for (int i = 0; i < child_count; ++i) {
        auto child = this->getChild(i);
        auto sub_box = child->getBoundingBox();
        CONTINUE_IF(!sub_box.isValid())
        if (bbox.isValid()) {
            bbox.fitBox(sub_box);
        } else {
            bbox = sub_box;
        }
    }
    return bbox;
}
PointCloudPtr OriginDataCloudListFolder::stitchAndFilter(
        float leaf_x, float leaf_y, float leaf_z) {
    if (!this->isValid()) {
        UWarn(QObject::tr("No child!"));
        PointCloudPtr cloud_empty(new PointCloud());
        return cloud_empty;
    }
    int child_count = this->getChildSize();

    common::point_cloud::AccVoxelFilter<PointX> vf;
    common::Boxd bbox = this->getBoundingBox();
    vf.init(bbox.cast<float>(), leaf_x, leaf_y, leaf_z);
    for (int i = 0; i < child_count; ++i) {
        UProgressTextValue(QObject::tr("Process..."), i, child_count);
        auto child = this->getChild(i);
        auto sub_cloud = child->getSourceCloud();
        vf.filter(sub_cloud);
        child->reset();
    }
    return vf.getCloud();
}
PointCloudPtr OriginDataCloudListFolder::extractCloudInPolygon(
        const common::Polygon2d& polygon) {
    if (!this->isValid()) {
        UWarn(QObject::tr("No child!"));
        PointCloudPtr cloud_empty(new PointCloud());
        return cloud_empty;
    }
    if (!polygon.isValid()) {
        UWarn(QObject::tr("Polygon is not valid!"));
        PointCloudPtr cloud_empty(new PointCloud());
        return cloud_empty;
    }
    common::Rectd brect = polygon.getBoundingRect();
    int child_count = this->getChildSize();
    PointCloudPtr cloud_ptr(new PointCloud());
    PointCloudPtr sub_cloud = nullptr;
    static std::mutex s_mutex;
    for (int i = 0; i < child_count; ++i) {
        UProgressTextValue(QObject::tr("Extract "), i, child_count);
        {
            std::lock_guard<std::mutex> lock(s_mutex); // origin data
            auto child = this->getChild(i);
            auto child_rect = child->getBoundingRect();
            CONTINUE_IF(!child_rect.isIntersect(brect))
            sub_cloud = child->getSourceCloud();
            child->reset();
        }
        CONTINUE_IF(!sub_cloud.get() || sub_cloud->empty())
        cloud_ptr->points.reserve(cloud_ptr->size() + sub_cloud->size());
        for (auto& pt : sub_cloud->points) {
            CONTINUE_IF(!brect.isIn(pt.x, pt.y))
            CONTINUE_IF(!polygon.isIn(pt.x, pt.y))
            cloud_ptr->emplace_back(pt);
        }
    }
    return cloud_ptr;
}
PointCloudPtr OriginDataCloudListFolder::extractCloudInCircle(
        const common::Circled& circle) {
    if (!this->isValid()) {
        UWarn(QObject::tr("No child!"));
        PointCloudPtr cloud_empty(new PointCloud());
        return cloud_empty;
    }
    if (!circle.isValid()) {
        UWarn(QObject::tr("Polygon is not valid!"));
        PointCloudPtr cloud_empty(new PointCloud());
        return cloud_empty;
    }
    common::Rectd brect = circle.getBoundingRect();
    int child_count = this->getChildSize();
    PointCloudPtr cloud_ptr(new PointCloud());
    PointCloudPtr sub_cloud = nullptr;
    static std::mutex s_mutex;
    for (int i = 0; i < child_count; ++i) {
        UProgressTextValue(QObject::tr("Extract "), i, child_count);
        {
            std::lock_guard<std::mutex> lock(s_mutex); // origin data
            auto child = this->getChild(i);
            auto child_rect = child->getBoundingRect();
            CONTINUE_IF(!child_rect.isIntersect(brect))
            sub_cloud = child->getSourceCloud();
            child->reset();
        }
        CONTINUE_IF(!sub_cloud.get() || sub_cloud->empty())
        cloud_ptr->points.reserve(cloud_ptr->size() + sub_cloud->size());
        for (auto& pt : sub_cloud->points) {
            CONTINUE_IF(!circle.isIn(pt.x, pt.y))
            cloud_ptr->emplace_back(pt);
        }
    }
    return cloud_ptr;
}

void OriginDataCloudListFolder::projectToImages(
        cv::Mat& image_rgb, cv::Mat& image_d, cv::Mat& image_i,
        PointCloud& cloud_out, const common::Pose3d& pose,
        float cx, float cy, float fx, float fy, int width, int height, 
        float min_depth, float max_depth, float cloud_res) {
    min_depth = std::max(min_depth, 0.1f);
    float fxy = std::hypot(fx, fy);
    cloud_out.clear();

    image_rgb = cv::Mat::zeros(height, width, CV_8UC3);
    image_i = cv::Mat::zeros(height, width, CV_32FC1);
    image_d = cv::Mat::ones(height, width, CV_32FC1);
    image_d *= 1e5; // make max

    Eigen::Matrix4d mat = pose.toMatrix();
    common::Boxd view_bbox;
    float x1 = (width - cx) * max_depth / fx;
    float x2 = (0 - cx) * max_depth / fx;
    float y1 = (height - cy) * max_depth / fy;
    float y2 = (0 - cy) * max_depth / fy;
    view_bbox.fitPoint(0, 0, 0);
    view_bbox.fitPoint(x1, y1, max_depth);
    view_bbox.fitPoint(x1, y2, max_depth);
    view_bbox.fitPoint(x2, y1, max_depth);
    view_bbox.fitPoint(x2, y2, max_depth);
    //std::cout << "view bbox: " << view_bbox << std::endl;

    int child_count = this->getChildSize();
    PointCloudPtr cloud_ptr = nullptr;
    static std::mutex s_mutex;
    for (int i = 0; i < child_count; ++i) {
        {
            std::lock_guard<std::mutex> lock(s_mutex);
            auto child = this->getChild(i);
            auto bbox = child->getBoundingBox();
            auto tbox = bbox.transformed(mat);
            //std::cout << i << " " << tbox << std::endl;
            CONTINUE_IF(!tbox.isIntersect(view_bbox))
            cloud_ptr = child->getTransformCloud(mat);
            child->reset();
            CONTINUE_IF(!cloud_ptr.get())
            //std::cout << "valid: " << i << std::endl;
        }
        for (auto& pt : cloud_ptr->points) {
            CONTINUE_IF(pt.z < min_depth || pt.z > max_depth) // min value
            int c = std::round(pt.x * fx / pt.z + cx);
            CONTINUE_IF(c < 0 || c >= width)
            int r = std::round(pt.y * fy / pt.z + cy);
            CONTINUE_IF(r < 0 || r >= height)
            float tmp_z = image_d.at<float>(r, c);
            // 将视口内全部推入
            if (std::fabs(pt.z < tmp_z - 2.0)) {
                cloud_out.emplace_back(pt);
            }
            CONTINUE_IF(tmp_z < pt.z)
            image_rgb.at<cv::Vec3b>(r, c) = cv::Vec3b(pt.b, pt.g, pt.r);
            image_i.at<float>(r, c) = pt.intensity;
            image_d.at<float>(r, c) = pt.z;
            int tmp_r = std::floor(cloud_res * fxy / pt.z);
            CONTINUE_IF(tmp_r < 1)
            // std::cout << tmp_r << " " << pt.z << std::endl;
            for (int m = -tmp_r; m <= tmp_r; ++m) {
                for (int n = -tmp_r; n <= tmp_r; ++n) {
                    CONTINUE_IF(std::hypot(float(m), float(n)) > float(tmp_r))
                    int tr = r + m;
                    int tc = c + n;
                    CONTINUE_IF(tc < 0 || tc >= width)
                    CONTINUE_IF(tr < 0 || tr >= height)
                    tmp_z = image_d.at<float>(tr, tc);
                    CONTINUE_IF(tmp_z < pt.z)
                    image_rgb.at<cv::Vec3b>(tr, tc) = cv::Vec3b(pt.b, pt.g, pt.r);
                    image_i.at<float>(tr, tc) = pt.intensity;
                    image_d.at<float>(tr, tc) = pt.z;
                }
            }
        }
    }
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            CONTINUE_IF(image_d.at<float>(i, j) < max_depth)
            image_d.at<float>(i, j) = 0.f;
        }
    }
    // 去除多余点
    int pt_index = 0;
    for (auto& pt : cloud_out.points) {
        int c = std::round(pt.x * fx / pt.z + cx);
        CONTINUE_IF(c < 0 || c >= width)
        int r = std::round(pt.y * fy / pt.z + cy);
        CONTINUE_IF(r < 0 || r >= height)
        float tmp_z = image_d.at<float>(r, c);
        CONTINUE_IF(std::fabs(tmp_z - pt.z) > 2.0)
        cloud_out.points[pt_index++] = pt;
    }
    cloud_out.resize(pt_index);
}
/////////////////////
bool OriginDataDOMFolder::extractImageByRect(cv::Mat& image, const common::Rectd& rect) {
    auto child_size = this->getChildSize();
    double res_x = 0.02, res_y = 0.02;
    bool is_intersect = false;
    for (int i = 0; i < child_size; ++i) {
        auto sub_folder = this->getChild(i);
        auto image_rect = sub_folder->getRect();
        if (i == 0) {
            auto image_mat = sub_folder->getImage();
            res_x = image_rect.getLengthX() / image_mat.cols;
            res_y = image_rect.getLengthY() / image_mat.rows;
            auto image_cols = std::ceil(rect.getLengthX() / res_x);
            auto image_rows = std::ceil(rect.getLengthY() / res_y);
            image = cv::Mat(image_rows, image_cols, CV_8UC1, cv::Scalar(0));
        }
        if (!rect.isIntersect(image_rect)) {continue;}
        auto image_mat = sub_folder->getImage();
        common::Rectd intersection = getIntersection(rect, image_rect);
        auto cvrect1 = getCvRect(rect.getTopLeftPoint(), intersection, res_x, res_y);
        auto cvrect2 = getCvRect(image_rect.getTopLeftPoint(), intersection, res_x, res_y);
        // std::cout << "cvrect1: " << cvrect1.x << ", " << cvrect1.y 
        //     << ", " << cvrect1.width << ", " << cvrect1.height << std::endl;
        // std::cout << "cvrect2: " << cvrect2.x << ", " << cvrect2.y 
        //     << ", " << cvrect2.width << ", " << cvrect2.height << std::endl;
        if (image_mat.type() == CV_8UC1) {
            image_mat(cvrect2).copyTo(image(cvrect1));
        } else if (image_mat.type() == CV_8UC3) {
            cv::cvtColor(image_mat(cvrect2), image(cvrect1), cv::COLOR_BGR2GRAY);
        } else if (image_mat.type() == CV_8UC4) {
            cv::cvtColor(image_mat(cvrect2), image(cvrect1), cv::COLOR_BGRA2GRAY);
        } else {
            UWarn(QObject::tr("Image type is %1, not supprot convert to gray").arg(image_mat.type()));
        }
        is_intersect = true;
    }
    return is_intersect;
}
common::Rectd OriginDataDOMFolder::getIntersection(const common::Rectd& r1, const common::Rectd& r2) const {
    auto minx = std::max(r1.range_x.min_v, r2.range_x.min_v);
    auto maxx = std::min(r1.range_x.max_v, r2.range_x.max_v);
    auto miny = std::max(r1.range_y.min_v, r2.range_y.min_v);
    auto maxy = std::min(r1.range_y.max_v, r2.range_y.max_v);
    return common::Rectd(minx, maxx, miny, maxy);
}
cv::Rect OriginDataDOMFolder::getCvRect(const common::Point2d& origin, 
        const common::Rectd& rect, double res_x, double res_y) const {
    int cols = std::ceil(rect.getLengthX() / res_x);
    int rows = std::ceil(rect.getLengthY() / res_y);
    common::Rectd new_rect = rect;
    new_rect.addOffset(-origin.x, -origin.y);
    int x = static_cast<int>(new_rect.getMinX() / res_x);
    int y = static_cast<int>(new_rect.getMinY() / res_y);
    return cv::Rect(x, y, cols, rows);
}

////////////////////
void OriginDataControlPointListFolder::toPointPairList(
        common::Point3dList& src_points, 
        common::Point3dList& tgt_points) {
    int child_count = this->getChildSize();
    src_points.resize(child_count);
    tgt_points.resize(child_count);
    for (int i = 0; i < child_count; ++i) {
        auto child = this->getChild(i);
        src_points[i] = child->getSourcePoint();
        tgt_points[i] = child->getTargetPoint();
    }
}
void OriginDataControlPointListFolder::toPointPairList(
        common::Point2dList& src_points, 
        common::Point2dList& tgt_points) {
    int child_count = this->getChildSize();
    src_points.resize(child_count);
    tgt_points.resize(child_count);
    for (int i = 0; i < child_count; ++i) {
        auto child = this->getChild(i);
        auto src_pt = child->getSourcePoint();
        src_points[i].x = src_pt.x;
        src_points[i].y = src_pt.y;
        auto tgt_pt = child->getTargetPoint();
        tgt_points[i].x = tgt_pt.x;
        tgt_points[i].y = tgt_pt.y;
    }
}
bool OriginDataControlPointListFolder::solvePose3d(
        common::Pose3d& pose_out, std::vector<double>& error_list) {
    common::Point3dList src_points, tgt_points;
    this->toPointPairList(src_points, tgt_points);
    RETURN_FALSE_IF(!common::math::SolveTransformSVD(src_points, tgt_points, pose_out))
    common::math::ComputeResidualError(src_points, tgt_points, pose_out, error_list);
    return true;
}
bool OriginDataControlPointListFolder::solvePose2d(
        common::Pose2d& pose_out, std::vector<double>& error_list) {
    common::Point2dList src_points, tgt_points;
    this->toPointPairList(src_points, tgt_points);
    RETURN_FALSE_IF(!common::math::SolveTransformSVD(src_points, tgt_points, pose_out))
    common::math::ComputeResidualError(src_points, tgt_points, pose_out, error_list);
    return true;
}

////////////////////
void OriginDataFolder::projectToBirdView(double res_x, double res_y,
        ProjectType type, double min_value, double max_value, int color_map) {
    auto full_cloud_folder = this->getFullCloudFolder();
    if (!full_cloud_folder->isValid()) {return;}
    auto bbox = full_cloud_folder->getBoundingBox();
    auto cloud = full_cloud_folder->getSourceCloud();
    bbox.addPadding(2.0, 2.0, 2.0);
    cv::Mat image_out;
    common::point_cloud::ImageProjector<PointX>(
        bbox.cast<float>(), *cloud, image_out, res_x, res_y,
        type, min_value, max_value, color_map);
    auto bird_view_folder = this->getBirdViewFolder();
    double origin_x = bbox.getMinX();
    double origin_y = bbox.getMinY();
    bird_view_folder->setData(image_out, origin_x, origin_y, res_x, res_y);
}
void OriginDataFolder::projectToFrontView(double res_x, double res_y,
        ProjectType type, double min_value, double max_value, int color_map) {
    auto full_cloud_folder = this->getFullCloudFolder();
    if (!full_cloud_folder->isValid()) {return;}
    Eigen::Matrix4d swap_mat; swap_mat.setIdentity();
    swap_mat(1, 1) = 0; swap_mat(2, 2) = 0;
    swap_mat(1, 2) = 1; swap_mat(2, 1) = -1;
    // 将Y/Z对调
    auto cloud = full_cloud_folder->getTransformCloud(swap_mat);
    auto bbox = common::point_cloud::ComputeBoundingBox(*cloud);
    bbox.addPadding(2.0, 2.0, 2.0);
    // auto bbox = full_cloud_folder->getBoundingBox();
    // common::Boxd swap_box = bbox;
    // swap_box.range_y.min_v = bbox.range_z.min_v;
    // swap_box.range_y.max_v = bbox.range_z.max_v;
    // swap_box.range_z.min_v = -bbox.range_y.max_v;
    // swap_box.range_z.max_v = -bbox.range_y.min_v;
    cv::Mat image_out;
    common::point_cloud::ImageProjector<PointX>(
        bbox, *cloud, image_out, res_x, res_y,
        type, min_value, max_value, color_map);
    auto front_view_folder = this->getFrontViewFolder();
    double origin_x = bbox.getMinX();
    double origin_y = bbox.getMinY();
    front_view_folder->setData(image_out, origin_x, origin_y, res_x, res_y);
}
//////
void OriginDataFolder::projectToDOM(
        double min_z, double max_z, double res_x, double res_y,
        ProjectType type, double min_value, double max_value, int color_map) {
    auto dom_folder = this->getDOMFolder();
    dom_folder->removeAllChildren();
    auto cloud_list_folder = this->getCloudListFolder();
    //
    int child_count = cloud_list_folder->getChildSize();
    std::atomic<int> finished_count{0};
    std::mutex tmp_mutex;
#pragma omp parallel for num_threads(6)
    for (int i = 0; i < child_count; ++i) {
        auto cloud_folder = cloud_list_folder->getChild(i);
        auto name = cloud_folder->getName();
        auto cloud = cloud_folder->getSourceCloud();
        auto bbox = cloud_folder->getBoundingBox();
        bbox.range_z.min_v = min_z;
        bbox.range_z.max_v = max_z;
        cloud_folder->reset();
        cv::Mat image_out;
        common::point_cloud::ImageProjector<PointX>(
            bbox.cast<float>(), *cloud, image_out, 
            res_x, res_y, type, min_value, max_value, color_map);
        double origin_x = bbox.getMinX();
        double origin_y = bbox.getMinY();
        {
            std::lock_guard<std::mutex> lock(tmp_mutex);
            auto image_folder = dom_folder->addChild("_" + name);
            image_folder->setData(image_out, origin_x, origin_y, res_x, res_y);
            image_folder->reset();
        }
        ++finished_count;
        UProgressTextValue(QObject::tr("Process DOM... "), finished_count.load(), child_count);
    }
}

}
