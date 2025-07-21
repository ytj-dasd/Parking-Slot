#include "auto_detection_dialog.h"
#include "ui_auto_detection_dialog.h"

namespace welkin {
namespace bamboo {
AutoDetectionDialog::AutoDetectionDialog(QWidget *parent) :
        QDialog(parent), ui(new Ui::AutoDetectionDialog) {
    ui->setupUi(this);
}

AutoDetectionDialog::~AutoDetectionDialog() {
    delete ui;
}
QString AutoDetectionDialog::getInferenceMode() const {
    auto index = ui->comboBox_inferenceMode->currentIndex();
    return index == 0 ? "CPU" : "GPU";
}
} // namespace bamboo
} // namespace welkin
