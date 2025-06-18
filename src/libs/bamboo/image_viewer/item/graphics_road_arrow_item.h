#pragma once
#include "bamboo/base/macro.h"
#include "bamboo/image_viewer/item/graphics_text_polygon_item.h"

namespace welkin::bamboo {
class BAMBOO_EXPORT GraphicsRoadArrowItem : public GraphicsTextPolygonItem {
public:
    GraphicsRoadArrowItem(QGraphicsItem* parent = nullptr);
    virtual ~GraphicsRoadArrowItem();
    
    enum RoadArrowType {
        TypeUnknown = 0,               // 未知
        TypeForward = 1,               // 直行
        TypeLeftTurn = 2,              // 左转
        TypeRightTurn = 3,             // 右转
        TypeUTurn = 4,                 // 掉头
        TypeForwardAndLeftTurn = 5,    // 直行左转
        TypeForwardAndRightTurn = 6,    // 直行右转
        TypeLeftRightTurn = 7,         // 左右转
        TypeThreeDirection = 8         // 三向(左右转直行)
    };
    RoadArrowType roadArrowType() const;
    void setRoadArrowType(RoadArrowType type);
protected:
    RoadArrowType _road_arrow_type = TypeUnknown;
};
}
