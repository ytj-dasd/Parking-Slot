#pragma once
#include <string>
#include <common/geometry/box.h>
#include <common/geometry/point3.h>
#include "bamboo/base/macro.h"
#include "bamboo/io/las_point.h"

namespace welkin::bamboo {
class LasReaderPrivate;
class BAMBOO_EXPORT LasReader {
public:
    LasReader();
    virtual ~LasReader();
    bool open(const std::string& filename);
    void close();
    size_t pointCount() const;
    common::Boxd getBBox() const;
    common::Point3d getOffset() const;
    void readPoint(LasPoint& point);
private:
    LasReaderPrivate* _ptr = nullptr;
};
}