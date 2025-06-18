#include "model_param_dialog.h"
#include "ui_model_param_dialog.h"

#include "bamboo/base/osg_utils.h"

ModelParamDialog::ModelParamDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ModelParamDialog) {
    
    ui->setupUi(this);
    
    this->setWindowTitle(tr("Model Param Dialog"));

    using ColorMode = welkin::bamboo::point_cloud::ColorRender::Mode;
    ui->comboBox_model_colormode->addItem(tr("Origin"), QVariant(ColorMode::MODE_ORIGIN));
    ui->comboBox_model_colormode->addItem(tr("FieldX"), QVariant(ColorMode::MODE_FIELD_X));
    ui->comboBox_model_colormode->addItem(tr("FieldY"), QVariant(ColorMode::MODE_FIELD_Y));
    ui->comboBox_model_colormode->addItem(tr("FieldZ"), QVariant(ColorMode::MODE_FIELD_Z));
    ui->comboBox_model_colormode->addItem(tr("FieldI"), QVariant(ColorMode::MODE_FIELD_INTENSITY));
    ui->comboBox_model_colormode->addItem(tr("Rand"), QVariant(ColorMode::MODE_RANDOM));
    ui->comboBox_model_colormode->setCurrentIndex(0);
}

ModelParamDialog::~ModelParamDialog() {
    delete ui;
}
int ModelParamDialog::getColorMode() const {
    return ui->comboBox_model_colormode->currentData().toInt();
}
double ModelParamDialog::getMinRatio() const {
    return ui->doubleSpinBox_model_min_ratio->value();
}
double ModelParamDialog::getMaxRatio() const {
    return ui->doubleSpinBox_model_max_ratio->value();
}
