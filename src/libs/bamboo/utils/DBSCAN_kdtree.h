#pragma once

#include <pcl/point_types.h>
#include "bamboo/utils/DBSCAN_simple.h"

// DBScan聚类算法
namespace welkin::bamboo {
template <typename PointT>
class DBSCANKdtreeCluster: public DBSCANSimpleCluster<PointT> {
protected:
    virtual int radiusSearch (
            int index, double radius, std::vector<int> &k_indices,
            std::vector<float> &k_sqr_distances) const {
        return this->search_method_->radiusSearch(index, radius, k_indices, k_sqr_distances);
    }
}; // class DBSCANCluster
}