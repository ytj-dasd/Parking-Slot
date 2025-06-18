#pragma once
#include <QDialog>
#include <uface/resource/uresource.h>
#include <uface/canvas2d/layer.hpp>
#include <uface/canvas2d/view.hpp>
#include <uface/canvas2d/action.hpp>
#include "bamboo/base/macro.h"

namespace welkin::bamboo {
class BAMBOO_EXPORT RectImageDialog : public QDialog {
    Q_OBJECT
public:
    explicit RectImageDialog(QWidget* parent = nullptr);
    virtual ~RectImageDialog();

    bool isOK() const {return _is_ok;}
    void setRectImage(common::file_system::RectImageFolder* rect_image);

    // 将Y轴取反
    void setInverseY(bool is_inv);

    uface::canvas2d::Viewer* getImageViewer() {
        return _image_viewer;
    }
    uface::canvas2d::RectImageLayer* getRectImageLayer() {
        return _rect_image_layer;
    }
protected Q_SLOTS:
    void onOK();
    void onCancel();

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void showEvent(QShowEvent *event) override;
    virtual void clearData() {}
protected:
    bool _is_ok = false;
    QToolBar* _confirm_toolbar = nullptr;
    common::Rectd _image_rect;
    uface::canvas2d::Viewer* _image_viewer = nullptr;
    uface::canvas2d::DrawToolBar* _draw_toolbar = nullptr;
    uface::canvas2d::RectImageLayer* _rect_image_layer = nullptr;
};
}
