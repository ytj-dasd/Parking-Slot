#include "cloud_convertor_widget.h"
#include "ui_cloud_convertor_widget.h"

CloudConvertorWidget::CloudConvertorWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CloudConvertorWidget)
{
    ui->setupUi(this);
}

CloudConvertorWidget::~CloudConvertorWidget()
{
    delete ui;
}
