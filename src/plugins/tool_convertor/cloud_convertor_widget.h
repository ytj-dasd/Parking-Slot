#ifndef CLOUD_CONVERTOR_WIDGET_H
#define CLOUD_CONVERTOR_WIDGET_H

#include <QWidget>

namespace Ui {
class CloudConvertorWidget;
}

class CloudConvertorWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CloudConvertorWidget(QWidget *parent = nullptr);
    ~CloudConvertorWidget();

private:
    Ui::CloudConvertorWidget *ui;
};

#endif // CLOUD_CONVERTOR_WIDGET_H
