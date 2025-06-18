#include "select_line_dialog.h"
#include <uface/base/uconverter.h>
#include <uface/canvas2d/item.hpp>
#include <uface/canvas2d/tool.hpp>

namespace welkin::bamboo {
SelectLineDialog::SelectLineDialog(QWidget* parent) : RectImageDialog(parent) {
    this->setWindowTitle(tr("Line Selector"));
    QList<QString> keys;
    keys << uface::canvas2d::LineDrawKey << uface::canvas2d::ViewKey
         << uface::canvas2d::MoveKey << uface::canvas2d::EditKey;
    _draw_toolbar->setActionsVisible(keys);
    _draw_toolbar->setActionActived(uface::canvas2d::LineDrawKey, true);

    QPen pen(Qt::cyan, 2.0); pen.setCosmetic(true);
    _line_layer = _image_viewer->addLayer(
        "line", tr("Line"), 10.0, pen, Qt::NoBrush);
    _line_item = _line_layer->addLine(QLineF());
    
    _line_draw_tool = _draw_toolbar->getLineDrawTool();
    connect(_line_draw_tool, SIGNAL(started()), this, SLOT(hideLineItem()));
    connect(_line_draw_tool, SIGNAL(finished()), this, SLOT(onUpdateLine2d()));
}

SelectLineDialog::~SelectLineDialog() {

}

bool SelectLineDialog::isValid() const {
    return getLine().isValid();
}

common::Line2d SelectLineDialog::getLine() const {
    //如果不可见，则为无效线
    if (!_line_item->isVisible()) {
        return common::Line2d();
    }
    QLineF line = _line_item->line();
    QLineF scene_line;
    scene_line.setP1(_line_item->mapToScene(line.p1()));
    scene_line.setP2(_line_item->mapToScene(line.p2()));
    return UFQ(scene_line);
}

void SelectLineDialog::hideLineItem() {
    _line_item->setPos(0.0, 0.0);
    _line_item->setRotation(0.0);
    _line_item->setLine(QLineF());
    _line_item->hide();
}

void SelectLineDialog::onUpdateLine2d() {
    setLineHelper(_line_draw_tool->line());
}

void SelectLineDialog::clearData() {
    hideLineItem();
}

void SelectLineDialog::setLineHelper(const QLineF &line) {
    if (!_line_item->isVisible()) {
        _line_item->setVisible(true);
    }
    _line_item->setPos(QPointF(0.0, 0.0));
    _line_item->setRotation(0.0);
    _line_item->setLine(line);
}
}
