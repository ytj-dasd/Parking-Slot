#pragma once
#include "bamboo/base/macro.h"
#include <QComboBox>
namespace welkin::bamboo {
class BAMBOO_EXPORT ColorMapComboBox : public QComboBox {
    Q_OBJECT
public:
    ColorMapComboBox(QWidget* parent);
    virtual ~ColorMapComboBox();
    int getCurrentType() const;
signals:
    void currentTypeChanged(int type);
private slots:
    void onCurrentIndex(int /*index*/);
};
}
