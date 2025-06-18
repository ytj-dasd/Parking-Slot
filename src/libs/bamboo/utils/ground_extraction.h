#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>

//Reference: Two-step adaptive extraction method for ground points and breaklines from lidar point clouds, Bisheng Yang, Ronggang Huang, et al. ISPRS Journal of Photogrammetry and Remote Sensing
namespace welkin::bamboo {
struct CenterPoint {
	double x;
	double y;
	double z;
	CenterPoint(double x = 0, double y = 0, double z = 0) :
		x(x), y(y), z(z)
	{
		z = 0.0;
		x = y = 0.0;
	}

};

struct Bounds {
	double min_x;
	double min_y;
	double min_z;
	double max_x;
	double max_y;
	double max_z;
	Bounds() {
		min_x = min_y = min_z = max_x = max_y = max_z = 0.0;
	}
};

struct Voxel {
	std::vector<int> point_id;
	float min_z;
	float max_z;
	float dertaz;
	float min_z_x;
	float min_z_y;
	float NeighborMin_z;
	int PointsNumber;
	float mean_z;
	Voxel() {
		min_z = min_z_x = min_z_y = NeighborMin_z = mean_z = 0.f;
		PointsNumber = 1;
		dertaz = 0.0;
	}
};

class Ground_Extraction {
public:

	Ground_Extraction();
	void Extract_ground_pts(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr no_ground_cloud,
		Bounds bounds, CenterPoint center_pt);
	
protected:

private:

	void Get_grid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		double max_x, double max_y, double min_x, double min_y,
		int row, int list, int num_voxel, Voxel* grid, pcl::PointCloud<pcl::PointXYZRGB>::Ptr no_ground_cloud);

	void Seg_ground_nground_pts(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr no_ground_cloud,
		Voxel* grid, int num_voxel);

	float grid_res_;
	int   min_pt_num_grid_;
	float max_height_diff_;
	float max_nei_height_diff_; 
};
}