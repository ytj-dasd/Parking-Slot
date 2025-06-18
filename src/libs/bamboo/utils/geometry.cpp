#include "geometry.h"
#include "clipper.hpp"
namespace welkin::bamboo {
bool MakeBufferByPolyline(const common::Polyline2d& polyline, 
        common::Polygon2d& polygon, double delta, double precision) {
    ClipperLib::Path subject;
    subject.resize(polyline.size());
    int x, y;
    for (int i = 0; i < polyline.size(); ++i) {
        x = std::round(polyline.points[i].x / precision);
        y = std::round(polyline.points[i].y / precision);
        subject[i] = ClipperLib::IntPoint(x, y);
    }

    // 初始化 ClipperOffset 实例
    ClipperLib::ClipperOffset offset;
    // offset.AddPath(subject, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
    offset.AddPath(subject, ClipperLib::jtMiter, ClipperLib::etOpenButt);

    // 设置偏移距离（这里设置为10个单位）
    double d = delta / precision;
    ClipperLib::Paths solution;
    // 执行偏移操作
    offset.Execute(solution, d);

    if (solution.empty()) {
        polygon.clear();
        return false;
    }
    auto& path = solution[0];
    polygon.points.resize(path.size());
    for (int i = 0; i < path.size(); ++i) {
        auto& point = polygon.points[i];
        point.x = static_cast<double>(path.at(i).X * precision);
        point.y = static_cast<double>(path.at(i).Y * precision);
    }
    return true;
}
}