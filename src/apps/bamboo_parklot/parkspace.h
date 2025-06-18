#pragma once
#include <common/geometry/point2.h>
#include <common/geometry/line2.h>
#include <common/geometry/polygon2.h>
#include <opencv2/opencv.hpp>

namespace welkin::bamboo {
class ParkSpace : public common::Polygon2d {
public:
    ParkSpace();
    virtual ~ParkSpace();
    bool isIntersect(const ParkSpace& other) const;
    const common::Point2d& getPoint(int index) const;
    common::Line2d getLine(int index) const;
    common::Point2d getLongSideDirection() const; // 长线方向
    void computeDesignSize();
    // 设计尺寸
    double design_width = 5.2;
    double design_height = 2.4;
    // 优良等级
    void computeLineLevel(std::vector<int>& deltas, std::vector<int>& levels) const;
};
}
