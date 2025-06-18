#ifndef MODEL_PARAM_DIALOG_H
#define MODEL_PARAM_DIALOG_H

#include <QDialog>
#include "bamboo/base/macro.h"

namespace Ui {
class ModelParamDialog;
}

class BAMBOO_EXPORT ModelParamDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ModelParamDialog(QWidget *parent = nullptr);
    ~ModelParamDialog();

    int getColorMode() const;
    double getMinRatio() const;
    double getMaxRatio() const;

private:
    Ui::ModelParamDialog *ui;
};

#endif // MODEL_PARAM_DIALOG_H
