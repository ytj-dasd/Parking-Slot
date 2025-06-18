#pragma once
#include <QString>
#include <common/geometry/polygon2.h>
#include <common/geometry/polyline2.h>
#include <common/geometry/polygon3.h>
#include <common/geometry/layer.h>
#include "bamboo/base/macro.h"

namespace welkin::bamboo {
////
// DXFFile dxf_file;
// if (!dxf_file.open(filename)) {return false;}
// ....
// dxf_file.close();
////
class DXFFilePrivate;
class BAMBOO_EXPORT DXFFile {
public:
    DXFFile();
    virtual ~DXFFile();
    using LayerList = std::vector<std::string>;
    bool open(const std::string& filename, const LayerList& layers = LayerList());

    void writePoint3(const common::Point3d& pt, const std::string& layer_id = "0");
    void writePolyline2(const std::vector<common::Point2d>& points,
        double default_z = 0, bool closed = false, const std::string& layer_id = "0");
    void writePolyline3(const std::vector<common::Point3d>& points, 
        bool closed = false, const std::string& layer_id = "0");

    void writeLayer(const common::Layerd& layer, const std::string& layer_id = "0");
    void writeImage(
        const std::string& filename, 
        int width, int height,
        const common::Point3d& insert_point, 
        const common::Point3d& uvec, 
        const common::Point3d& vvec,
        const std::string& layer_id = "0");
    void close();
private:
    DXFFilePrivate* _ptr = nullptr;
};
}
