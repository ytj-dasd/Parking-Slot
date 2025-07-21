#include "data_preprocessor.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <fstream>
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <omp.h>
#include <common/base/dir.h>
#include "bamboo/file_system/project.h"
#include "file_system.h"

namespace welkin::bamboo {
DataPreProcessor::DataPreProcessor(ParklotFileSystem* fs) : _file_system(fs) {
    Q_ASSERT(fs);
}
DataPreProcessor::~DataPreProcessor() {}
void DataPreProcessor::setProject(Project* proj) {
    _project = proj;
}

bool DataPreProcessor::process() {
    // 创建预处理文件夹
    common::Dir::Mkdir(_file_system->getPreprocessDir());

    auto cloud_folder = _project->getCloudListFolder();
    auto child_size = cloud_folder->getChildSize();

    double global_min_x = std::numeric_limits<float>::max();
    double global_max_x = std::numeric_limits<float>::lowest();
    double global_min_y = std::numeric_limits<float>::max();
    double global_max_y = std::numeric_limits<float>::lowest();
    
    UProgressTextValue(QObject::tr("Merging cloud"), 0, 4);
    PointCloudPtr map_cloud(new PointCloud()); 
    for (int i = 0; i < child_size; ++i) {
        auto sub_folder = cloud_folder->getChild(i);
        auto cloud_rect = sub_folder->getBoundingRect();
        auto cloud = sub_folder->getSourceCloud();
        
        global_min_x = std::min(global_min_x, cloud_rect.getMinX());
        global_max_x = std::max(global_max_x, cloud_rect.getMaxX());
        global_min_y = std::min(global_min_y, cloud_rect.getMinY());
        global_max_y = std::max(global_max_y, cloud_rect.getMaxY());

        *map_cloud += *cloud;
    }

    UProgressTextValue(QObject::tr("Bev transforming"), 1, 4);
    // 生成重叠分块图（用于角点检测）
    bevTransform(map_cloud, global_min_x, global_max_x, global_min_y, global_max_y);
    
    // 生成强度整图
    auto dom_folder = _project->getDOMFolder();
    auto dom_child_size = dom_folder->getChildSize();

    common::Rectd brect;
    double res_x = -0.1, res_y = -0.1;
    for (int i = 0; i < dom_child_size; ++i) {
        auto dom_sub_folder = dom_folder->getChild(i);
        auto image_rect = dom_sub_folder->getRect();
        brect.fitRect(image_rect);
        if (i == 0) {
            auto image_mat = dom_sub_folder->getImage();
            res_x = image_rect.getLengthX() / image_mat.cols;
            res_y = image_rect.getLengthY() / image_mat.rows;
        }
    }
    if (!brect.isValid() || res_x < 0.0 || res_y < 0.0) {
        UWarn(QObject::tr("Cloud bounding rect is invalid"));
        return false;
    }
    
    UProgressTextValue(QObject::tr("Patcing images"), 2, 4);
    int width = int(brect.getLengthX() / res_x);
    int height = int(brect.getLengthY() / res_y);
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
    for (int i = 0; i < child_size; ++i) {
        auto dom_sub_folder = dom_folder->getChild(i);
        auto image_rect = dom_sub_folder->getRect();

        auto col = int((image_rect.range_x.min_v - brect.range_x.min_v) / res_x);
        auto row = int((image_rect.range_y.min_v - brect.range_y.min_v) / res_y);
        auto image_mat = dom_sub_folder->getImage();
        cv::Rect rect(col, row, image_mat.cols, image_mat.rows);
        if (image_mat.type() == CV_8UC3) {
            image_mat.copyTo(image(rect));
        } else if (image_mat.type() == CV_8UC1) {
            cv::cvtColor(image_mat, image(rect), cv::COLOR_GRAY2BGR);
        } else if (image_mat.type() == CV_8UC4) {
            cv::cvtColor(image_mat, image(rect), cv::COLOR_BGRA2BGR);
        } else {
            UWarn(QObject::tr("Image type is %1, not supprot convert to rgb").arg(image_mat.type()));
        }
    }
    
    UProgressTextValue(QObject::tr("Saving images"), 3, 4);
    cv::flip(image, image, 0);
    std::string intensity_path = _file_system->getIntensityImagePath();
    cv::imwrite(intensity_path, image);
    UInfo(QObject::tr("Intensity Image: %1").arg(UTQ(intensity_path)));

    cv::Mat binary_intensity_image;
    cv::threshold(image, binary_intensity_image, 70, 255, cv::THRESH_BINARY); 
    // cv::threshold(image, binary_intensity_image, 130, 255, cv::THRESH_BINARY); // 小昆山 九里亭

    // cv::Mat gray_image;
    // cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    // cv::threshold(gray_image, binary_intensity_image, 128, 255, cv::THRESH_OTSU | cv::THRESH_BINARY);
    std::string binary_intensity_path = _file_system->getBinaryIntensityImagePath();
    cv::imwrite(binary_intensity_path, binary_intensity_image);
    UInfo(QObject::tr("Binary Intensity Image: %1").arg(UTQ(binary_intensity_path)));

    UInfo(QObject::tr("Data Preprocess Done"));
    return true;
}
void DataPreProcessor::bevTransform(const PointCloudPtr& cloud, 
        double min_x, double max_x, double min_y, double max_y) {
    int width = static_cast<int>(std::floor((max_x - min_x) / _resolution));
    int height = static_cast<int>(std::floor((max_y - min_y) / _resolution));

    std::cout << "min_x: " << min_x << ", max_x: " << max_x << std::endl;
    std::cout << "min_y: " << min_y << ", max_y: " << max_y << std::endl;
    std::cout << "Width: " << width << ", Height: " << height << std::endl;

    cv::Mat height_diff_image(height, width, CV_8UC1, cv::Scalar(0));  // 高差过滤图
    cv::Mat rgb_image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));    // RGB 图

    std::vector<std::vector<BevPixel>> grid(height, std::vector<BevPixel>(width));

    for (const auto& point : cloud->points) {
        int img_x = static_cast<int>((point.x - min_x) / _resolution);
        int img_y = static_cast<int>((max_y - point.y) / _resolution);
        if (img_x >= 0 && img_x < width && img_y >= 0 && img_y < height) {
            BevPixel& pixel = grid[img_y][img_x];
            if (point.z <= 3.0) { // 小昆山
            // if (point.z <= 1.5) { // 九里亭 金浩园
                pixel.diff_min_z = std::min(pixel.diff_min_z, point.z);
                if (point.z > pixel.diff_max_z) {
                    pixel.diff_max_z = point.z;
                    pixel.r = point.r;
                    pixel.g = point.g;
                    pixel.b = point.b;
                }                
            }
        }
    }

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            BevPixel& pixel = grid[y][x];
            if (pixel.diff_max_z != std::numeric_limits<float>::lowest() &&
                pixel.diff_min_z != std::numeric_limits<float>::max()) {
                float height_diff = pixel.diff_max_z - pixel.diff_min_z;
                if (height_diff > _height_threshold) {
                    height_diff_image.at<uchar>(y, x) = 255;  
                }
            }
            rgb_image.at<cv::Vec3b>(y, x) = cv::Vec3b(pixel.b, pixel.g, pixel.r);
        }
    }


    std::string height_diff_path = _file_system->getHeightDiffImagePath();
    cv::imwrite(height_diff_path, height_diff_image);
    std::string rgb_path = _file_system->getRgbImagePath();
    cv::imwrite(rgb_path, rgb_image);
    UInfo(QObject::tr("Image saved:\nHeight difference image %1.\nRgb image %2.")
        .arg(UTQ(height_diff_path)).arg(UTQ(rgb_path)));
    splitImage(rgb_image);
}

void DataPreProcessor::splitImage(const cv::Mat& image, int window_size, int step_size) {
    int img_height = image.rows;
    int img_width = image.cols;

    std::string output_dir = _file_system->getPatchImagesDir();
    common::Dir::Mkdir(output_dir);

    std::vector<cv::Mat> windows; 
    for (int y = 0; y < img_height; y += step_size) {
        for (int x = 0; x < img_width; x += step_size) {
            if (y + window_size > img_height || x + window_size > img_width) {
                continue;
            }

            cv::Mat window = image(cv::Rect(x, y, window_size, window_size));
            windows.push_back(window);

            std::string position_filename = output_dir + "/" + std::to_string(windows.size() - 1) + "_position.txt";
            std::ofstream pos_file(position_filename);
            pos_file << "Top-left corner: (" << x << ", " << y << ")";
            pos_file.close();
        }
    }

    #pragma omp parallel for
    for (int i = 0; i < windows.size(); ++i) {
        cv::Mat window = windows[i];
        cv::Mat mask;
        cv::inRange(window, cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0), mask);  
        cv::Mat inpainted_window = findNearestPixel(window, mask);

        std::string window_filename = output_dir + "/" + std::to_string(i) + ".png";
        cv::imwrite(window_filename, inpainted_window);
        // std::cout << "Processed and saved: " << window_filename << std::endl;
    }
    std::cout << "Images processed and saved to: " << output_dir << std::endl;
}

cv::Mat DataPreProcessor::findNearestPixel(const cv::Mat& img, const cv::Mat& mask, int max_dist) {
    cv::Mat output_img = img.clone();
    cv::Mat dist_transform;
    cv::distanceTransform(mask, dist_transform, cv::DIST_L2, 5);

    int rows = img.rows;
    int cols = img.cols;

    #pragma omp parallel for collapse(2)
    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            if (mask.at<uchar>(row, col) == 255) {
                if (dist_transform.at<float>(row, col) < max_dist) {
                    float min_dist = std::numeric_limits<float>::infinity();
                    cv::Vec3b nearest_pixel = cv::Vec3b(0, 0, 0);  

                    for (int i = std::max(-max_dist, -row); i < std::min(max_dist, rows - row); i++) {
                        for (int j = std::max(-max_dist, -col); j < std::min(max_dist, cols - col); j++) {
                            if (mask.at<uchar>(row + i, col + j) == 0 &&
                                dist_transform.at<float>(row + i, col + j) < min_dist) {
                                min_dist = dist_transform.at<float>(row + i, col + j);
                                nearest_pixel = img.at<cv::Vec3b>(row + i, col + j);
                            }
                        }
                    }

                    if (min_dist < std::numeric_limits<float>::infinity()) {
                        output_img.at<cv::Vec3b>(row, col) = nearest_pixel;
                    }
                } else {
                    output_img.at<cv::Vec3b>(row, col) = img.at<cv::Vec3b>(row, col);
                }
            }
        }
    }

    return output_img;
}
}