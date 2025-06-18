#include "image_pair_dialog.h"
#include <QGridLayout>
#include <uface/base/uconverter.h>

namespace welkin::bamboo {
ImagePairDialog::ImagePairDialog(QWidget* parent) : QDialog(parent) {
    this->setMinimumSize(1440, 810);
    // 开启最大最小按钮
    this->setWindowFlags(Qt::Dialog
        | Qt::WindowMinMaxButtonsHint | Qt::WindowCloseButtonHint);

    _confirm_toolbar = new QToolBar(this);
    _viewer_1 = new uface::canvas2d::Viewer(this);
    _draw_toolbar_1 = new uface::canvas2d::DrawToolBar(this);
    _draw_toolbar_1->setActionsVisible(QList<QString>());
    _draw_toolbar_1->setViewer(_viewer_1);
    _rect_image_layer_1 = _viewer_1->addLayer<uface::canvas2d::RectImageLayer>(
        "image", tr("Image"), -1.0);
    _viewer_2 = new uface::canvas2d::Viewer(this);
    _draw_toolbar_2 = new uface::canvas2d::DrawToolBar(this);
    _draw_toolbar_2->setActionsVisible(QList<QString>());
    _draw_toolbar_2->setViewer(_viewer_2);
    _rect_image_layer_2 = _viewer_2->addLayer<uface::canvas2d::RectImageLayer>(
        "image", tr("Image"), -1.0);
    
    QGridLayout* layout = new QGridLayout(this);
    //layout->setMargin(1);
    layout->setContentsMargins(1, 1, 1, 1);
    layout->setSpacing(0);
    _confirm_toolbar->addAction(UIcon("item_icon/ok.png"),
                       tr("OK"), this, SLOT(onOK()));
    _confirm_toolbar->addSeparator();
    _confirm_toolbar->addAction(UIcon("item_icon/cancel.png"),
                       tr("Cancel"), this, SLOT(onCancel()));
    // 2 * 8
    layout->addWidget(_draw_toolbar_1, 0, 0, 1, 4, Qt::AlignLeft);
    layout->addWidget(_draw_toolbar_2, 0, 4, 1, 3, Qt::AlignLeft);
    layout->addWidget(_confirm_toolbar, 0, 7, 1, 1, Qt::AlignRight);
    layout->addWidget(_viewer_1, 1, 0, 1, 4);
    layout->addWidget(_viewer_2, 1, 4, 1, 4);
}

ImagePairDialog::~ImagePairDialog() {}

void ImagePairDialog::setRectImage(
        common::file_system::RectImageFolder* rect_image_1,
        common::file_system::RectImageFolder* rect_image_2) {
    _is_ok = false;
    this->clearData();

    _rect_image_1 = rect_image_1;
    _rect_image_layer_1->setRectImage(_rect_image_1);

    _rect_image_2 = rect_image_2;
    _rect_image_layer_2->setRectImage(_rect_image_2);
}
void ImagePairDialog::setInverseY(bool is_inv_1, bool is_inv_2) {
    _viewer_1->setYFlipped(is_inv_1);
    _viewer_2->setYFlipped(is_inv_2);
}

void ImagePairDialog::onOK() {
    _is_ok = true;
    this->close();
}

void ImagePairDialog::onCancel() {
    _is_ok = false;
    this->close();
}

void ImagePairDialog::keyPressEvent(QKeyEvent *event) {
    // 屏蔽Esc键关闭对话框功能
    if (event->key() == Qt::Key_Escape) {
        return;
    }
    QDialog::keyPressEvent(event);
}

void ImagePairDialog::showEvent(QShowEvent *event) {
    QDialog::showEvent(event);
    if (_rect_image_1) {
        auto rect = _rect_image_1->getRect();
        _viewer_1->onFitInView(UTQ(rect));
    }
    if (_rect_image_2) {
        auto rect = _rect_image_2->getRect();
        _viewer_2->onFitInView(UTQ(rect));
    }
}
}
