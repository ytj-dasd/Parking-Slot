#pragma once
#include "bamboo/image_viewer/dialog/rect_image_dialog.h"

namespace welkin::bamboo {
class BAMBOO_EXPORT SelectPolygonDialog : public RectImageDialog {
    Q_OBJECT
public:
    explicit SelectPolygonDialog(QWidget* parent = nullptr);
    virtual ~SelectPolygonDialog();

    common::Polygon2d getPolygon() const;

protected Q_SLOTS:
    void onUpdateRect();
    void onUpdatePolygon();
    void hidePolygonItem();
protected:
    void clearData() override;
    void setPolygonHelper(const QPolygonF &polygon);
protected:
    uface::canvas2d::PolygonItem* _polygon_item = nullptr;
    uface::canvas2d::GenericLayer* _polygon_layer = nullptr;
    uface::canvas2d::RectDrawTool* _rect_draw_tool = nullptr;
    uface::canvas2d::PolygonDrawTool* _polygon_draw_tool = nullptr;
};
}
