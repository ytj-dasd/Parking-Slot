#pragma once
#include <common/geometry/polyline3.h>
#include <common/geometry/polygon3.h>
#include "bamboo/base/macro.h"

namespace welkin::bamboo {
class DxfReaderPrivate;
class BAMBOO_EXPORT DxfReader {
public:
    DxfReader();
    virtual ~DxfReader();

    bool open(const std::string& filepath);
    void close();
    const std::vector<common::Polygon3d>& getPolygonVec() const;
    const std::vector<common::Polyline3d>& getPolylineVec() const;

private:
    DxfReaderPrivate* _ptr = nullptr;
};
}
