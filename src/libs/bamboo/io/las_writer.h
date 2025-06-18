#pragma once
#include <string>
#include <common/geometry/box.h>
#include <common/geometry/point3.h>
#include "bamboo/base/macro.h"
#include "bamboo/io/las_point.h"

namespace welkin::bamboo {
class LasWriterPrivate;
class BAMBOO_EXPORT LasWriter {
public:
    LasWriter();
    virtual ~LasWriter();
    // 默认1mm
    bool open(const std::string& filename, const common::Boxd& bbox, double scale = 0.001);
    size_t pointCount() const;
    void writePoint(const LasPoint& point);
    void close();
private:
    LasWriterPrivate* _ptr = nullptr;
};
}