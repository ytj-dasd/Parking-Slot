#include "graphics_road_arrow_item.h"
#include <QFont>

namespace welkin::bamboo {
GraphicsRoadArrowItem::GraphicsRoadArrowItem(QGraphicsItem* parent) 
        : GraphicsTextPolygonItem(parent) {}
GraphicsRoadArrowItem::~GraphicsRoadArrowItem() {}
GraphicsRoadArrowItem::RoadArrowType GraphicsRoadArrowItem::roadArrowType() const {
    return _road_arrow_type;
}
void GraphicsRoadArrowItem::setRoadArrowType(RoadArrowType type) {
    if (_road_arrow_type == type) {return;}
    _road_arrow_type = type;
    switch (_road_arrow_type)
    {
    case TypeUnknown:
        setText(QObject::tr("Unknown Arrow"));
        break;
    case TypeForward:
        setText(QObject::tr("Forward Arrow"));
        break;
    case TypeLeftTurn:
        setText(QObject::tr("Left Turn Arrow"));
        break;
    case TypeRightTurn:
        setText(QObject::tr("Right Turn Arrow"));
        break;
    case TypeUTurn:
        setText(QObject::tr("U Turn Arrow"));
        break;
    case TypeForwardAndLeftTurn:
        setText(QObject::tr("Forward and Left Turn Arrow"));
        break;
    case TypeForwardAndRightTurn:
        setText(QObject::tr("Forward and Right Turn Arrow"));
        break;
    case TypeLeftRightTurn:
        setText(QObject::tr("Left and Right Turn Arrow"));
        break;
    case TypeThreeDirection:
        setText(QObject::tr("Three Direction Arrow"));
        break;
    default:
        setText(QObject::tr("Unknown Arrow"));
        break;
    }
}
}
