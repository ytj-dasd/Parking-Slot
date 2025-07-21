#include "parkline.h"
#include "parkspacegroup.h"
namespace welkin::bamboo {
// 获取线比例值
double getLineRatio(const common::Point2d& origin, const common::Point2d& dir, const common::Point2d& pt) {
    common::Point2d delta = pt - origin;
    return dir.dot(delta);
}
// 从平行分段线中获取完整线段
common::Line2d getLineFromParallelSegment(const std::vector<common::Line2d>& lines) {
    if (lines.empty()) {return common::Line2d();}
    if (lines.size() == 1) {return lines.front();}
    const auto& front_line = lines.front();
    auto origin = front_line.begin_point;
    auto dir = front_line.getDirection();
    std::vector<double> ratios;
    ratios.push_back(0.0);
    ratios.push_back(getLineRatio(origin, dir, front_line.end_point));
    for (int i = 1; i < lines.size(); ++i) {
        ratios.push_back(getLineRatio(origin, dir, lines[i].begin_point));
        ratios.push_back(getLineRatio(origin, dir, lines[i].end_point));
    }
    auto max_iter = std::max_element(ratios.begin(), ratios.end());
    auto min_iter = std::min_element(ratios.begin(), ratios.end());
    int max_index = std::distance(ratios.begin(), max_iter);
    int min_index = std::distance(ratios.begin(), min_iter);
    common::Line2d res_line;
    auto getPoint = [](const common::Line2d& line, int index) -> common::Point2d {
        return index == 0 ? line.begin_point : line.end_point;
    };
    res_line.begin_point = getPoint(lines[min_index / 2], min_index % 2);
    res_line.end_point = getPoint(lines[max_index / 2], max_index % 2);
    return res_line;
}
// ParkLine
// 存在线索引
bool ParkLine::hasIndex(int index) const {
    return (std::find(indexs.begin(), indexs.end(), index) != indexs.end());
}
void ParkLine::addIndex(int index) {
    indexs.push_back(index);
}
// 计算中心线
// offset_type: -1-往负方向偏移；0-不偏移；1-往正方向偏移
void ParkLine::computeCenterLine() {
    std::vector<common::Line2d> lines;
    // 获取线列表
    for (auto& index : indexs) {
        lines.push_back(_group->getLine(index));
    }
    if (type == TypeMiddle) {
        center_line = getLineFromParallelSegment(lines);
    } else {
        const auto& index = indexs.front();
        auto space_idx = index / 4;
        auto line_idx = index % 4;
        // 取一个非线上的点
        auto point = _group->getParkSpace(space_idx).getPoint((line_idx + 2) % 4);
        border_line = getLineFromParallelSegment(lines);
        int flag = (border_line.signDistanceTo(point) > 0) ? 1 : -1;
        center_line = border_line.getParallelLine(flag * line_width * 0.5);
        outer_border_signed = -flag;
    }
}
void ParkLine::computeBorderLine() {
    border_line1 = center_line.getParallelLine(-line_width * 0.5);
    border_line2 = center_line.getParallelLine(line_width * 0.5);
}
common::Line2d ParkLine::getSideLine(int signed_type) const {
    return center_line.getParallelLine(signed_type * line_width * 0.5);
}
void ParkLine::retify(const common::Line2d& line1, const common::Line2d& line2) {
    border_line1 = line1;
    border_line2 = line2;
    center_line.begin_point = (line1.begin_point + line2.begin_point) * 0.5;
    center_line.end_point = (line1.end_point + line2.end_point) * 0.5;
    line_width = 0.5 * line1.begin_point.distanceTo(line2.begin_point)
        + 0.5 * line1.end_point.distanceTo(line2.end_point);
    // std::cout << "linewidth:" << line_width << std::endl;
    int border1_signed = (center_line.signDistanceTo(border_line1.begin_point) > 0 ? 1 : -1);
    if (border1_signed == outer_border_signed) {
        border_line = border_line1;
    } else {
        border_line = border_line2;
    }
}
common::Polygon2d ParkLine::getParallelArea(double offset) {
    auto line1 = center_line.getParallelLine(-offset);
    auto line2 = center_line.getParallelLine(offset);
    common::Polygon2d polygon;
    polygon.addPoint(line1.begin_point);
    polygon.addPoint(line1.end_point);
    polygon.addPoint(line2.end_point);
    polygon.addPoint(line2.begin_point);
    return polygon;
}

common::Polygon2d ParkLine::getBorderParallelArea(const common::Line2d& line, double offset) {
    auto line1 = line.getParallelLine(-offset);
    auto line2 = line.getParallelLine(offset);
    common::Polygon2d polygon;
    polygon.addPoint(line1.begin_point);
    polygon.addPoint(line1.end_point);
    polygon.addPoint(line2.end_point);
    polygon.addPoint(line2.begin_point);

    return polygon;
}
}