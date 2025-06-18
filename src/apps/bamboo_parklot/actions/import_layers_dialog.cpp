#include "import_layers_dialog.h"
#include "ui_import_layers_dialog.h"
#include <QFileInfo>
#include <QDir>
#include <uface/widget/uwidget.h>

namespace welkin {
namespace bamboo {
ImportLayersDialog::ImportLayersDialog(QWidget *parent) :
        QDialog(parent), ui(new Ui::ImportLayersDialog) {
    ui->setupUi(this);

    this->setWindowTitle(tr("Save Layer As DXF"));
    connect(ui->radioButton_none, SIGNAL(clicked(bool)),
            this, SLOT(onNoneRatio(bool)));
    connect(ui->radioButton_local, SIGNAL(clicked(bool)),
            this, SLOT(onLocalRatio(bool)));
    connect(ui->radioButton_global, SIGNAL(clicked(bool)),
            this, SLOT(onGlobalRatio(bool)));
    connect(ui->toolButton_layer_filepaths, SIGNAL(clicked()),
            this, SLOT(onSavePath()));
}

ImportLayersDialog::~ImportLayersDialog() {
    delete ui;
}

void ImportLayersDialog::setHistoryPath(const QString &history_path) {
    _history_path = history_path;
}

void ImportLayersDialog::setPaths(const QStringList& paths) {
    _paths = paths;
    ui->lineEdit_layer_filepaths->setText(_paths.join(";"));
}
void ImportLayersDialog::setLocalPose(const common::Pose3d& pose) {
    _local_pose = pose;
}
void ImportLayersDialog::setGlobalPose(const common::Pose3d& pose) {
    _global_pose = pose;
}

QStringList ImportLayersDialog::getLayerFilePaths() const {
    return _paths;
}
common::Pose3d ImportLayersDialog::getPose3d() const {
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
void ImportLayersDialog::onNoneRatio(bool checked) {
    if (!checked) {return;}
    this->setPose(_eye_pose);
    this->setPoseReadOnly(true);
}
void ImportLayersDialog::onLocalRatio(bool checked) {
    if (!checked) {return;}
    this->setPose(_local_pose);
    this->setPoseReadOnly(true);
}
void ImportLayersDialog::onGlobalRatio(bool checked) {
    if (!checked) {return;}
    this->setPose(_global_pose);
    this->setPoseReadOnly(true);
}
void ImportLayersDialog::onSavePath() {
    _paths = UWidget::GetOpenFileNames(
        tr("Save path"), _history_path,
        "DXF files (*.dxf);;DWG files (*.dwg)", this);
    if (!_paths.isEmpty()) {
        this->setPaths(_paths);
        setHistoryPath(QFileInfo(_paths.front()).dir().absolutePath());
    }
}
void ImportLayersDialog::setPoseReadOnly(bool read_only) {
    ui->doubleSpinBox_X->setReadOnly(read_only);
    ui->doubleSpinBox_Y->setReadOnly(read_only);
    ui->doubleSpinBox_Z->setReadOnly(read_only);
    ui->doubleSpinBox_roll->setReadOnly(read_only);
    ui->doubleSpinBox_pitch->setReadOnly(read_only);
    ui->doubleSpinBox_yaw->setReadOnly(read_only);
}
void ImportLayersDialog::setPose(const common::Pose3d& pose) {
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
} // namespace welin
