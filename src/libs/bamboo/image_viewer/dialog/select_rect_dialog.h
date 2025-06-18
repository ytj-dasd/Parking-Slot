#pragma once
#include "bamboo/image_viewer/dialog/rect_image_dialog.h"

namespace welkin::bamboo {
class BAMBOO_EXPORT SelectRectDialog : public RectImageDialog {
    Q_OBJECT
public:
    explicit SelectRectDialog(QWidget* parent = nullptr);
    virtual ~SelectRectDialog();

    common::Rectd getRect() const;

protected Q_SLOTS:
    void onUpdateRect();
    void hideRectItem();
protected:
    void clearData() override;
    void setRectHelper(const QRectF &rect);
protected:
    uface::canvas2d::RectItem* _rect_item = nullptr;
    uface::canvas2d::GenericLayer* _rect_layer = nullptr;
    uface::canvas2d::RectDrawTool* _rect_draw_tool = nullptr;
};
}
