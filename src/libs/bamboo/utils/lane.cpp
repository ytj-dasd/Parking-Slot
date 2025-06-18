#include <flann/flann.h>
#include "lane.h"
#include "ground_extraction.h"
#include "DBSCAN_kdtree.h"
#include <pcl/common/common.h>
#include <pcl/point_types_conversion.h>

namespace welkin::bamboo {
void FilterYellowLaneCloud(
        const PointCloudPtr cloud_in, PointCloudPtr& cloud_out) {
    using PointCloudRGB = pcl::PointCloud<pcl::PointXYZRGB>;
    PointCloudRGB::Ptr cloud_tmp(new PointCloudRGB());
    pcl::copyPointCloud(*cloud_in, *cloud_tmp);

    Ground_Extraction ground_extraction;
    PointCloudRGB::Ptr cloud_ground(new PointCloudRGB());
    PointCloudRGB::Ptr cloud_unground(new PointCloudRGB());
    Bounds bounds; 
    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(*cloud_tmp, min_pt, max_pt);
    bounds.min_x = min_pt.x;
    bounds.min_y = min_pt.y;
    bounds.min_z = min_pt.z;
    bounds.max_x = max_pt.x;
    bounds.max_y = max_pt.y;
    bounds.max_z = max_pt.z;
    CenterPoint center_pt;
    ground_extraction.Extract_ground_pts(cloud_tmp, cloud_ground, cloud_unground, bounds, center_pt);

    using PointCloudHSV = pcl::PointCloud<pcl::PointXYZHSV>;
    PointCloudHSV::Ptr cloud_hsv(new PointCloudHSV());
    pcl::PointCloudXYZRGBtoXYZHSV(*cloud_ground, *cloud_hsv);
    
    // 黄色
    PointCloudRGB::Ptr cloud_filter(new PointCloudRGB());
    for (int i = 0; i < cloud_hsv->size(); ++i) {
        auto pthsv = cloud_hsv->points[i];
        auto h = pthsv.h * 0.5;
        auto s = pthsv.s * 255;
        auto v = pthsv.v * 255;
        if (h >= 10 && h <= 50 && s >= 46 && s <= 255 && v >= 43 && v <= 255) {
            cloud_filter->push_back(cloud_ground->points[i]);
        }
    }

    std::vector<pcl::PointIndices> cluster_indices;
    DBSCANKdtreeCluster<pcl::PointXYZRGB> ec;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    kdtree->setInputCloud(cloud_filter);
    ec.setCorePointMinPts(10);
    ec.setClusterTolerance(0.15);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000000);
    ec.setSearchMethod(kdtree);
    ec.setInputCloud(cloud_filter);
    ec.extract(cluster_indices);
    
    cloud_out->clear();
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            PointX pt;
            pt.x = cloud_filter->points[*pit].x;
            pt.y = cloud_filter->points[*pit].y;
            pt.z = cloud_filter->points[*pit].z;
            cloud_out->push_back(pt);
        }
    }
}

void FilterWhiteLaneCloud(
        const PointCloudPtr cloud_in, PointCloudPtr& cloud_out) {
    using PointCloudRGB = pcl::PointCloud<pcl::PointXYZRGB>;
    PointCloudRGB::Ptr cloud_tmp(new PointCloudRGB());
    pcl::copyPointCloud(*cloud_in, *cloud_tmp);

    Ground_Extraction ground_extraction;
    PointCloudRGB::Ptr cloud_ground(new PointCloudRGB());
    PointCloudRGB::Ptr cloud_unground(new PointCloudRGB());
    Bounds bounds; 
    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(*cloud_tmp, min_pt, max_pt);
    bounds.min_x = min_pt.x;
    bounds.min_y = min_pt.y;
    bounds.min_z = min_pt.z;
    bounds.max_x = max_pt.x;
    bounds.max_y = max_pt.y;
    bounds.max_z = max_pt.z;
    CenterPoint center_pt;
    ground_extraction.Extract_ground_pts(cloud_tmp, cloud_ground, cloud_unground, bounds, center_pt);

    // 白
    PointCloudRGB::Ptr cloud_filter(new PointCloudRGB());
    for (int i = 0; i < cloud_ground->size(); ++i) {
        auto pt = cloud_ground->points[i];
        if (pt.r >= 128 && pt.g >= 128 && pt.b >= 128) {
            cloud_filter->push_back(pt);
        }
    }

    std::vector<pcl::PointIndices> cluster_indices;
    DBSCANKdtreeCluster<pcl::PointXYZRGB> ec;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    kdtree->setInputCloud(cloud_filter);
    ec.setCorePointMinPts(10);
    ec.setClusterTolerance(0.15);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000000);
    ec.setSearchMethod(kdtree);
    ec.setInputCloud(cloud_filter);
    ec.extract(cluster_indices);
    
    cloud_out->clear();
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            PointX pt;
            pt.x = cloud_filter->points[*pit].x;
            pt.y = cloud_filter->points[*pit].y;
            pt.z = cloud_filter->points[*pit].z;
            cloud_out->push_back(pt);
        }
    }
}
}