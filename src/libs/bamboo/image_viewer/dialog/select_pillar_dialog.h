#pragma once
#include "bamboo/image_viewer/dialog/image_pair_dialog.h"

namespace welkin::bamboo {
class BAMBOO_EXPORT SelectPillarDialog : public ImagePairDialog {
    Q_OBJECT
public:
    explicit SelectPillarDialog(QWidget* parent = nullptr);
    virtual ~SelectPillarDialog();

    common::Polygon2d getPolygon() const;
    common::Ranged getRangeZ() const;

public Q_SLOTS:
    void onUpdatePolygon();
    void onUpdateRect();
    void hideRectItem();
    void hidePolygonItem();
protected:
    void clearData() override;
    void setRectHelper(const QRectF &rect);
    void setPolygonHelper(const QPolygonF& polygon);

protected:
    uface::canvas2d::RectItem* _rect_item = nullptr;
    uface::canvas2d::PolygonItem* _polygon_item = nullptr;
    uface::canvas2d::GenericLayer* _rect_layer = nullptr;
    uface::canvas2d::GenericLayer* _polygon_layer = nullptr;
    uface::canvas2d::RectDrawTool* _rect_draw_tool = nullptr;
    uface::canvas2d::PolygonDrawTool* _polygon_draw_tool = nullptr;
};
}
