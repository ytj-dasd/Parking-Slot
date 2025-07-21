#pragma once
#include <vector>
#include <common/geometry/line2.h>
#include "parkspace.h"
namespace welkin::bamboo {
// 获取线比例值
double getLineRatio(const common::Point2d& origin, const common::Point2d& dir, const common::Point2d& pt);
// 从平行分段线中获取完整线段
common::Line2d getLineFromParallelSegment(const std::vector<common::Line2d>& lines);

class ParkSpaceGroup;
// 库位线
class ParkLine {
public:
    ParkLine(ParkSpaceGroup* group) {_group = group;}
    virtual ~ParkLine() {}

    enum Type {
        TypeBorder = 0,   // 边界线
        TypeMiddle = 1    // 中间线
    };
    Type type = TypeBorder;
    bool is_long_side = false; // 是否为长线
    // 独立库位某条线索引数组
    std::vector<int> indexs;

    double line_width = 0.15; // 线宽
    common::Line2d center_line; // 中心线
    int outer_border_signed = 1; // 外边界线符号（相对于中心线）
    common::Line2d border_line; // 边界线

    // 拟合成功
    bool fit_success = false;
    common::Line2d border_line1; // 边界线1
    common::Line2d border_line2; // 边界线2
    
    // 是否需要检核
    bool need_check = false;
    int test_index = -1;
    
    // 存在线索引
    bool hasIndex(int index) const;
    void addIndex(int index);
    // 计算中心线
    void computeCenterLine();
    // 计算边界线
    void computeBorderLine();
    // 获得边线: type 负方向=-1，正方向=1
    common::Line2d getSideLine(int signed_type) const;
    // 计算
    // 改正库位线
    void retify(const common::Line2d& line1, const common::Line2d& line2);
    common::Polygon2d getParallelArea(double offset);
    common::Polygon2d getBorderParallelArea(const common::Line2d& line, double offset);
private:
    ParkSpaceGroup* _group;
};
}