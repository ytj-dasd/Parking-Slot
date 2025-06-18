#include "dom_param_dialog.h"
#include <QLabel>
#include "bamboo/base/types.h"
namespace welkin::bamboo {
DOMParamDialog::DOMParamDialog(QWidget *parent) : SelectRectDialog(parent) {
    _draw_toolbar->addSeparator();
    _combobox_type = new QComboBox(_draw_toolbar);
    _combobox_type->setMaximumHeight(25);
    _combobox_type->addItem(tr("NUM"), QVariant(bamboo::ProjectType::PROJECT_NUM));
    _combobox_type->addItem(tr("RGB"), QVariant(bamboo::ProjectType::PROJECT_RGB));
    _combobox_type->addItem(tr("Intensity"), QVariant(bamboo::ProjectType::PROJECT_INTENSITY));
    _combobox_type->setCurrentIndex(1);
    _draw_toolbar->addWidget(_combobox_type);
    _combobox_colormap = new QComboBox(_draw_toolbar);
    _combobox_colormap->setMaximumHeight(25);
    _combobox_colormap->addItem(tr("NONE"), QVariant(-1));
    _combobox_colormap->addItem(tr("HOT"), QVariant(cv::COLORMAP_HOT));
    _combobox_colormap->addItem(tr("RAINBOW"), QVariant(cv::COLORMAP_RAINBOW));
    _combobox_colormap->addItem(tr("DEEPGREEN"), QVariant(cv::COLORMAP_DEEPGREEN));
    _combobox_colormap->setCurrentIndex(0);
    _draw_toolbar->addWidget(_combobox_colormap);
    
    QLabel* label_res = new QLabel(tr("Res:"), _draw_toolbar);
    label_res->setMaximumHeight(25);
    _draw_toolbar->addWidget(label_res);
    _spinbox_res = new QDoubleSpinBox(_draw_toolbar);
    _spinbox_res->setMaximumHeight(25);
//    _spinbox_res->setRange(0.01, 2.0);
    _spinbox_res->setDecimals(3);
    _spinbox_res->setRange(0.001, 2.0);
    _spinbox_res->setSingleStep(0.01);
    _spinbox_res->setValue(0.02);
    _draw_toolbar->addWidget(_spinbox_res);

    _draw_toolbar->addSeparator();
    QLabel* label_min = new QLabel(tr("Min:"), _draw_toolbar);
    label_min->setMaximumHeight(25);
    _draw_toolbar->addWidget(label_min);
    _spinbox_min_value = new QSpinBox(_draw_toolbar);
    _spinbox_min_value->setMaximumHeight(25);
    _spinbox_min_value->setRange(0, 255);
    _spinbox_min_value->setSingleStep(1);
    _spinbox_min_value->setValue(2);
    _draw_toolbar->addWidget(_spinbox_min_value);

    QLabel* label_max = new QLabel(tr("Max:"), _draw_toolbar);
    label_max->setMaximumHeight(25);
    _draw_toolbar->addWidget(label_max);
    _spinbox_max_value = new QSpinBox(this);
    _spinbox_max_value->setMaximumHeight(25);
    _spinbox_max_value->setRange(0, 255);
    _spinbox_max_value->setSingleStep(1);
    // _spinbox_max_value->setValue(128);
    _spinbox_max_value->setValue(60);
    _draw_toolbar->addWidget(_spinbox_max_value);
}

DOMParamDialog::~DOMParamDialog() {
    
}
//
int DOMParamDialog::getType() const {
    return _combobox_type->currentData().toInt();
}
int DOMParamDialog::getColorMap() const {
    return _combobox_colormap->currentData().toInt();
}
double DOMParamDialog::getRes() const {
    return _spinbox_res->value();
}
int DOMParamDialog::getMinValue() const {
    return _spinbox_min_value->value();
}
int DOMParamDialog::getMaxValue() const {
    return _spinbox_max_value->value();
}
}
