// 采用libdxfrw库实现
#pragma once
#include <common/geometry/circle.h>
#include <common/geometry/arc.h>
#include <common/geometry/polygon2.h>
#include <common/geometry/polyline2.h>
#include <common/geometry/polygon3.h>
#include <common/geometry/polyline3.h>
#include <common/geometry/line3.h>
#include <common/geometry/layer.h>

namespace welkin::bamboo {
class DxfInputPrivate;
class DxfInput {
public:
    DxfInput();
    virtual ~DxfInput();

    bool open(const std::string& filepath);
    bool close();
    const std::vector<common::Point3d>& getPointVec() const;
    const std::vector<common::Line3d>& getLineVec() const;
    const std::vector<common::Polygon3d>& getPolygonVec() const;
    const std::vector<common::Polyline3d>& getPolylineVec() const;
    const std::vector<common::Circled>& getCircleVec() const;
    const std::vector<common::Arcd>& getArcVec() const;

private:
    DxfInputPrivate* _ptr = nullptr;
};

class DxfOutputPrivate;
class DxfOutput {
public:
    DxfOutput();
    virtual ~DxfOutput();
    using LayerList = std::vector<std::string>;
    bool open(const std::string& filename);
    bool close();

    // 创建图层
    // color = 3, 绿色
    void writeLayerIds(const LayerList& layer_ids = LayerList(), int color = 3);

    // color = 256 颜色随图层
    void writePoint3(const common::Point3d& pt, 
        int color = 256, const std::string& layer_id = "0");
    void writeLine3(const common::Line3d& line, 
        int color = 256, const std::string& layer_id = "0");
    void writePolyline2(const std::vector<common::Point2d>& points,
        double default_z = 0, bool closed = false, 
        int color = 256, const std::string& layer_id = "0");
    void writePolyline3(
        const std::vector<common::Point3d>& points, bool closed = false, 
        int color = 256, const std::string& layer_id = "0");
    void writeText(const std::string& str, const common::Line2d& line,
        double angle, double height,
        int color = 256, const std::string& layer_id = "0");
    void writeLayer(const common::Layerd& layer, 
        int color = 256, const std::string& layer_id = "0");
    void writeImage(
        const std::string& filename,
        int width, int height,
        const common::Point3d& insert_point,
        const common::Point3d& uvec,
        const common::Point3d& vvec,
        const std::string& layer_id = "0");
private:
    DxfOutputPrivate* _ptr = nullptr;
};
}
