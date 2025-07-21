#include "export_layer_as_dxf_dialog.h"
#include "ui_export_layer_as_dxf_dialog.h"
#include <uface/widget/uwidget.h>

namespace welkin {
namespace bamboo {
ExportLayerAsDxfDialog::ExportLayerAsDxfDialog(QWidget *parent) :
        QDialog(parent), ui(new Ui::ExportLayerAsDxfDialog) {
    ui->setupUi(this);

    this->setWindowTitle(tr("Save Layer As DXF"));
    connect(ui->radioButton_none, SIGNAL(clicked(bool)),
            this, SLOT(onNoneRatio(bool)));
    connect(ui->radioButton_local, SIGNAL(clicked(bool)),
            this, SLOT(onLocalRatio(bool)));
    connect(ui->radioButton_global, SIGNAL(clicked(bool)),
            this, SLOT(onGlobalRatio(bool)));
    connect(ui->toolButton_save_path, SIGNAL(clicked()),
            this, SLOT(onSavePath()));
}

ExportLayerAsDxfDialog::~ExportLayerAsDxfDialog() {
    delete ui;
}

void ExportLayerAsDxfDialog::setPath(const QString& path) {
    ui->lineEdit_save_path->setText(path);
}
void ExportLayerAsDxfDialog::setLocalPose(const common::Pose3d& pose) {
    _local_pose = pose;
}
void ExportLayerAsDxfDialog::setGlobalPose(const common::Pose3d& pose) {
    _global_pose = pose;
}

QString ExportLayerAsDxfDialog::getSavePath() const {
    return ui->lineEdit_save_path->text();
}
common::Pose3d ExportLayerAsDxfDialog::getPose3d() const {
    if (ui->radioButton_none->isChecked()) {
        return _eye_pose;
    } else if (ui->radioButton_local->isChecked()) {
        return _local_pose;
    } else if (ui->radioButton_global->isChecked()) {
        return _global_pose;
    } else {
        return _eye_pose;
    }
}
void ExportLayerAsDxfDialog::onNoneRatio(bool checked) {
    if (!checked) {return;}
    this->setPose(_eye_pose);
    this->setPoseReadOnly(true);
}
void ExportLayerAsDxfDialog::onLocalRatio(bool checked) {
    if (!checked) {return;}
    this->setPose(_local_pose);
    this->setPoseReadOnly(true);
}
void ExportLayerAsDxfDialog::onGlobalRatio(bool checked) {
    if (!checked) {return;}
    this->setPose(_global_pose);
    this->setPoseReadOnly(true);
}
void ExportLayerAsDxfDialog::onSavePath() {
    QString save_path = ui->lineEdit_save_path->text();
    save_path = UWidget::GetSaveFileName(
        tr("Save path"), save_path, "DXF File(*.dxf)", this);
    if (!save_path.isEmpty()) {
        ui->lineEdit_save_path->setText(save_path);
    }
}
void ExportLayerAsDxfDialog::setPoseReadOnly(bool read_only) {
    ui->doubleSpinBox_X->setReadOnly(read_only);
    ui->doubleSpinBox_Y->setReadOnly(read_only);
    ui->doubleSpinBox_Z->setReadOnly(read_only);
    ui->doubleSpinBox_roll->setReadOnly(read_only);
    ui->doubleSpinBox_pitch->setReadOnly(read_only);
    ui->doubleSpinBox_yaw->setReadOnly(read_only);
}
void ExportLayerAsDxfDialog::setPose(const common::Pose3d& pose) {
    double ext[6];
    pose.toEulerExtrinsic(ext);
    ui->doubleSpinBox_roll->setValue(ext[0] * M_PI / 180.0);
    ui->doubleSpinBox_pitch->setValue(ext[1] * M_PI / 180.0);
    ui->doubleSpinBox_yaw->setValue(ext[2] * M_PI / 180.0);
    ui->doubleSpinBox_X->setValue(ext[3]);
    ui->doubleSpinBox_Y->setValue(ext[4]);
    ui->doubleSpinBox_Z->setValue(ext[5]);
}
} // namespace bamboo
} // namespace welkin
