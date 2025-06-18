#pragma once
#include <QDialog>
#include <uface/resource/uresource.h>
#include <uface/canvas2d/layer.hpp>
#include <uface/canvas2d/view.hpp>
#include <uface/canvas2d/action.hpp>
#include "bamboo/base/macro.h"

namespace welkin::bamboo {
class BAMBOO_EXPORT ImagePairDialog : public QDialog {
    Q_OBJECT
public:
    explicit ImagePairDialog(QWidget* parent = nullptr);
    virtual ~ImagePairDialog();

    bool isOK() const {return _is_ok;}

    void setRectImage(
        common::file_system::RectImageFolder* rect_image_1,
        common::file_system::RectImageFolder* rect_image_2);

    // 将Y轴取反
    void setInverseY(bool is_inv_1, bool is_inv_2);
protected Q_SLOTS:
    void onOK();
    void onCancel();

protected:
    virtual void clearData() {}
    void keyPressEvent(QKeyEvent *event) override;
    void showEvent(QShowEvent *event) override;
protected:
    bool _is_ok = false;
    QToolBar* _confirm_toolbar = nullptr;
    uface::canvas2d::Viewer* _viewer_1 = nullptr;
    uface::canvas2d::DrawToolBar* _draw_toolbar_1 = nullptr;
    uface::canvas2d::RectImageLayer* _rect_image_layer_1 = nullptr;
    common::file_system::RectImageFolder *_rect_image_1 = nullptr;
    
    uface::canvas2d::Viewer* _viewer_2 = nullptr;
    uface::canvas2d::DrawToolBar* _draw_toolbar_2 = nullptr;
    uface::canvas2d::RectImageLayer* _rect_image_layer_2 = nullptr;
    common::file_system::RectImageFolder *_rect_image_2 = nullptr;
};
}
