#include "import_detection_dialog.h"
#include "ui_import_detection_dialog.h"
#include <QFileInfo>
#include <QDir>
#include <uface/widget/uwidget.h>

namespace welkin {
namespace bamboo {
ImportDetectionDialog::ImportDetectionDialog(QWidget *parent) :
        QDialog(parent), ui(new Ui::ImportDetectionDialog) {
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

ImportDetectionDialog::~ImportDetectionDialog() {
    delete ui;
}

void ImportDetectionDialog::setHistoryPath(const QString &history_path) {
    _history_path = history_path;
}

void ImportDetectionDialog::setPaths(const QStringList& paths) {
    _paths = paths;
    ui->lineEdit_layer_filepaths->setText(_paths.join(";"));
}
void ImportDetectionDialog::setLocalPose(const common::Pose3d& pose) {
    _local_pose = pose;
}
void ImportDetectionDialog::setGlobalPose(const common::Pose3d& pose) {
    _global_pose = pose;
}

QStringList ImportDetectionDialog::getLayerFilePaths() const {
    return _paths;
}
common::Pose3d ImportDetectionDialog::getPose3d() const {
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
void ImportDetectionDialog::onNoneRatio(bool checked) {
    if (!checked) {return;}
    this->setPose(_eye_pose);
    this->setPoseReadOnly(true);
}
void ImportDetectionDialog::onLocalRatio(bool checked) {
    if (!checked) {return;}
    this->setPose(_local_pose);
    this->setPoseReadOnly(true);
}
void ImportDetectionDialog::onGlobalRatio(bool checked) {
    if (!checked) {return;}
    this->setPose(_global_pose);
    this->setPoseReadOnly(true);
}
void ImportDetectionDialog::onSavePath() {
    _paths = UWidget::GetOpenFileNames(
        tr("Save path"), _history_path,
        "TXT files (*.txt);;DWG files (*.dwg)", this);
    if (!_paths.isEmpty()) {
        this->setPaths(_paths);
        setHistoryPath(QFileInfo(_paths.front()).dir().absolutePath());
    }
}
void ImportDetectionDialog::setPoseReadOnly(bool read_only) {
    ui->doubleSpinBox_X->setReadOnly(read_only);
    ui->doubleSpinBox_Y->setReadOnly(read_only);
    ui->doubleSpinBox_Z->setReadOnly(read_only);
    ui->doubleSpinBox_roll->setReadOnly(read_only);
    ui->doubleSpinBox_pitch->setReadOnly(read_only);
    ui->doubleSpinBox_yaw->setReadOnly(read_only);
}
void ImportDetectionDialog::setPose(const common::Pose3d& pose) {
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
