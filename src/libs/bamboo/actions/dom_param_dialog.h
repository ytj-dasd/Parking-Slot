#ifndef DOM_PARAM_DIALOG_H
#define DOM_PARAM_DIALOG_H

#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include "bamboo/image_viewer/dialog/select_rect_dialog.h"

namespace welkin::bamboo {
class BAMBOO_EXPORT DOMParamDialog : public SelectRectDialog {
    Q_OBJECT

public:
    explicit DOMParamDialog(QWidget *parent = nullptr);
    ~DOMParamDialog();

    int getType() const;
    int getColorMap() const;
    double getRes() const;
    int getMinValue() const;
    int getMaxValue() const;
    
private:
    QComboBox* _combobox_type = nullptr;
    QComboBox* _combobox_colormap = nullptr;

    QDoubleSpinBox* _spinbox_res = nullptr;

    QSpinBox* _spinbox_min_value = nullptr;
    QSpinBox* _spinbox_max_value = nullptr;
};
}
#endif // MODEL_PARAM_DIALOG_H
