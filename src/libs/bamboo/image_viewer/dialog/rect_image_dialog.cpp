#include "rect_image_dialog.h"
#include <QGridLayout>
#include <uface/base/uconverter.h>

namespace welkin::bamboo {
RectImageDialog::RectImageDialog(QWidget* parent) : QDialog(parent) {
    this->setMinimumSize(1440, 810);
    this->setSizeGripEnabled(true);
    // 开启最大最小按钮
    this->setWindowFlags(Qt::Dialog
        | Qt::WindowMinMaxButtonsHint | Qt::WindowCloseButtonHint);

    _image_viewer = new uface::canvas2d::Viewer(this);
    _draw_toolbar = new uface::canvas2d::DrawToolBar(this);
    _draw_toolbar->setViewer(_image_viewer);
    _draw_toolbar->setActionsVisible(QList<QString>());
    _rect_image_layer = _image_viewer->addLayer<uface::canvas2d::RectImageLayer>(
        "image", tr("Image"), 0.0);

    QGridLayout* layout = new QGridLayout(this);
    //layout->setMargin(1);
    layout->setContentsMargins(1, 1, 1, 1);
    layout->setSpacing(0);
//    _toolbar = new QToolBar(this);
    _confirm_toolbar = new QToolBar(this);
    _confirm_toolbar->addAction(UIcon("item_icon/ok.png"),
                       tr("OK"), this, SLOT(onOK()));
    _confirm_toolbar->addSeparator();
    _confirm_toolbar->addAction(UIcon("item_icon/cancel.png"),
                       tr("Cancel"), this, SLOT(onCancel()));
    // 2 * 4
//    layout->addWidget(_toolbar, 0, 0, Qt::AlignLeft);
    layout->addWidget(_draw_toolbar, 0, 0, 1, 3, Qt::AlignLeft);
    layout->addWidget(_confirm_toolbar, 0, 3, 1, 1, Qt::AlignRight);
    layout->addWidget(_image_viewer, 1, 0, 1, 4);
}

RectImageDialog::~RectImageDialog() {
    // std::cout << "release " << std::endl;
}

void RectImageDialog::setRectImage(
        common::file_system::RectImageFolder *rect_image) {
    _is_ok = false;
    this->clearData();
    if (!rect_image) {return;}
    _image_rect = rect_image->getRect();
    _rect_image_layer->setRectImage(rect_image);
}

void RectImageDialog::setInverseY(bool is_inv) {
    _image_viewer->setYFlipped(is_inv);
}

void RectImageDialog::onOK() {
    _is_ok = true;
    this->close();
}

void RectImageDialog::onCancel() {
    _is_ok = false;
    this->close();
}

void RectImageDialog::keyPressEvent(QKeyEvent *event) {
    // 屏蔽Esc键关闭对话框功能
    if (event->key() == Qt::Key_Escape) {
        return;
    }
    QDialog::keyPressEvent(event);
}

void RectImageDialog::showEvent(QShowEvent *event) {
    QDialog::showEvent(event);
    if (_image_rect.isValid()) {
        _image_viewer->onFitInView(UTQ(_image_rect));
    }
}
}
