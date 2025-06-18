#pragma once
#include "bamboo/image_viewer/dialog/rect_image_dialog.h"

namespace welkin::bamboo {
class BAMBOO_EXPORT SelectLineDialog : public RectImageDialog {
    Q_OBJECT
public:
    explicit SelectLineDialog(QWidget* parent = nullptr);
    virtual ~SelectLineDialog();

    bool isValid() const;
    common::Line2d getLine() const;

public Q_SLOTS:
    void hideLineItem();
protected Q_SLOTS:
    void onUpdateLine2d();
protected:
    void clearData() override;
    void setLineHelper(const QLineF& line);
protected:
    uface::canvas2d::LineItem* _line_item = nullptr;
    uface::canvas2d::LineDrawTool* _line_draw_tool = nullptr;
    uface::canvas2d::GenericLayer* _line_layer = nullptr;
};
}
