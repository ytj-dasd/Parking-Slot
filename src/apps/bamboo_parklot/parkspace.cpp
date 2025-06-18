#include "parkspace.h"
#include <QLineF>

namespace welkin::bamboo {
// double ParkSpace::res = 0.02;
// double ParkSpace::linewidth = 0.18;
ParkSpace::ParkSpace() : common::Polygon2d() {
    points.resize(4);
    // edge_types.resize(4, EdgeBorder);
}
ParkSpace::~ParkSpace() {}
bool ParkSpace::isIntersect(const ParkSpace& other) const {
    double padding = 0.02; // 2cm
    common::Rectd brect = this->getBoundingRect();
    brect.addPadding(padding, padding);

    common::Rectd br = other.getBoundingRect();
    br.addPadding(padding, padding);
    if (!brect.isIntersect(br)) {return false;}
    for (const auto& p1 : this->points) {
        for (const auto& p2 : other.points) {
            if (p1.distanceTo(p2) < padding) {return true;}
        }
    }
    return false;
}
const common::Point2d& ParkSpace::getPoint(int index) const {
    return points.at(index);
}
common::Line2d ParkSpace::getLine(int index) const {
    const auto& p1 = points[index % points.size()];
    const auto& p2 = points[(index + 1) % points.size()];
    return common::Line2d(p1, p2);
}
common::Point2d ParkSpace::getLongSideDirection() const {
    auto l1 = getLine(0); auto l2 = getLine(1);
    return l1.getLength() > l2.getLength() ? l1.getDirection() : l2.getDirection();
}
void ParkSpace::computeDesignSize() {
    auto l1 = getLine(0); auto l2 = getLine(1);
    if (l1.getLength() > l2.getLength()) {
        design_width = l1.getLength();
        design_height = l2.getLength();
        return;
    }
    if (l1.getLength() < l2.getLength()) {
        design_width = l2.getLength();
        design_height = l1.getLength();
        return;
    }
}

void ParkSpace::computeLineLevel(std::vector<int>& deltas, std::vector<int>& levels) const {
    auto l1 = getLine(0); auto l2 = getLine(1);
    std::vector<double> designs;
    if (l1.getLength() > l2.getLength()) {
        designs = {design_width, design_height, design_width, design_height};
    } else {
        designs = {design_height, design_width, design_height, design_width};
    }
    deltas.resize(4, 0);
    levels.resize(4, 1);
    for (int i = 0; i < 4; ++i) {
        auto delta = int((getLine(i).getLength() - designs.at(i)) * 100);
        deltas[i] = delta;
        if (std::abs(delta) == 0) {
            levels[i] = 1;
        } else if (std::abs(delta) == 1) {
            levels[i] = 2;
        } else if (std::abs(delta) == 2) {
            levels[i] = 3;
        } else if (std::abs(delta) == 3) {
            levels[i] = 4;
        } else if (std::abs(delta) == 4) {
            levels[i] = 5;
        } else {
            levels[i] = 6;
        }
    }
}
}
