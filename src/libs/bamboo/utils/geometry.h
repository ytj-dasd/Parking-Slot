#pragma once
#include "bamboo/base/macro.h"
#include <common/geometry/polyline2.h>
#include <common/geometry/polygon2.h>
namespace welkin::bamboo {
// 根据多线构建缓冲区
// precision = 0.001 精确到毫米
bool BAMBOO_EXPORT MakeBufferByPolyline(const common::Polyline2d& polyline, 
    common::Polygon2d& polygon, double delta = 0.5, double precision = 0.001);
}