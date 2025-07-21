#ifndef WELKIN_BAMBOO_AUTO_DETECTION_DIALOG_H
#define WELKIN_BAMBOO_AUTO_DETECTION_DIALOG_H

#include <QDialog>

namespace welkin {
namespace bamboo {

namespace Ui {
class AutoDetectionDialog;
}

class AutoDetectionDialog : public QDialog {
    Q_OBJECT

public:
    explicit AutoDetectionDialog(QWidget *parent = nullptr);
    ~AutoDetectionDialog();
    // 获取推理模式
    QString getInferenceMode() const;
private:
    Ui::AutoDetectionDialog *ui;
};
} // namespace bamboo
} // namespace welkin
#endif // WELKIN_BAMBOO_AUTO_DETECTION_DIALOG_H
