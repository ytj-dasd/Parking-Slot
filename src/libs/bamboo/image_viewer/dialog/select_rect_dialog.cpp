#include "select_rect_dialog.h"
#include <uface/base/uconverter.h>
#include <uface/canvas2d/tool.hpp>

namespace welkin::bamboo {
SelectRectDialog::SelectRectDialog(QWidget* parent) : RectImageDialog(parent) {
    this->setWindowTitle(tr("Rect Selector"));

    QList<QString> keys;
    keys << uface::canvas2d::RectDrawKey << uface::canvas2d::ViewKey
         << uface::canvas2d::MoveKey << uface::canvas2d::EditKey;
    _draw_toolbar->setActionsVisible(keys);
    _draw_toolbar->setActionActived(uface::canvas2d::RectDrawKey, true);

    QPen pen(Qt::cyan, 2.0); pen.setCosmetic(true);
    _rect_layer = _image_viewer->addLayer(
        "rect", tr("Rect"), 10.0, pen, Qt::NoBrush);
    _rect_item = _rect_layer->addRect(QRectF());
    
    _rect_draw_tool = _draw_toolbar->getRectDrawTool();
    connect(_rect_draw_tool, SIGNAL(started()), this, SLOT(hideRectItem()));
    connect(_rect_draw_tool, SIGNAL(finished()), this, SLOT(onUpdateRect()));
}

SelectRectDialog::~SelectRectDialog() {

}
common::Rectd SelectRectDialog::getRect() const {
    if (!_rect_item->isVisible()) {
        return common::Rectd();
    }
    QRectF rect = _rect_item->mapRectToScene(_rect_item->rect());
    return UFQ(rect);
}

void SelectRectDialog::onUpdateRect() {
    setRectHelper(_rect_draw_tool->rect());
}

void SelectRectDialog::hideRectItem() {
    _rect_item->setPos(0.0, 0.0);
    _rect_item->setRotation(0.0);
    _rect_item->setRect(QRectF());
    _rect_item->hide();
}

void SelectRectDialog::clearData() {
    hideRectItem();
}

void SelectRectDialog::setRectHelper(const QRectF &rect) {
    if (!_rect_item->isVisible()) {
        _rect_item->setVisible(true);
    }
    _rect_item->setPos(QPointF(0.0, 0.0));
    _rect_item->setRotation(0.0);
    _rect_item->setRect(rect);
}
}
