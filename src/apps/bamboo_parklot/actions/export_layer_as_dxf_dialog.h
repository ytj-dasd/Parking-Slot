#ifndef WELKIN_BAMBOO_EXPORT_LAYER_AS_DXF_DIALOG_H
#define WELKIN_BAMBOO_EXPORT_LAYER_AS_DXF_DIALOG_H

#include <QDialog>
#include <common/geometry/pose3.h>

namespace welkin {
namespace bamboo {

namespace Ui {
class ExportLayerAsDxfDialog;
}

class ExportLayerAsDxfDialog : public QDialog {
    Q_OBJECT

public:
    explicit ExportLayerAsDxfDialog(QWidget *parent = nullptr);
    ~ExportLayerAsDxfDialog();
    void setPath(const QString& path);
    void setLocalPose(const common::Pose3d& pose);
    void setGlobalPose(const common::Pose3d& pose);

    QString getSavePath() const;
    common::Pose3d getPose3d() const;
private slots:
    void onNoneRatio(bool checked);
    void onLocalRatio(bool checked);
    void onGlobalRatio(bool checked);
    void onSavePath();
private:
    void setPoseReadOnly(bool readonly);
    void setPose(const common::Pose3d& pose);
private:
    Ui::ExportLayerAsDxfDialog *ui;
    common::Pose3d _eye_pose;
    common::Pose3d _local_pose;
    common::Pose3d _global_pose;
};

} // namespace bamboo
} // namespace welkin
#endif // WELKIN_BAMBOO_EXPORT_LAYER_AS_DXF_DIALOG_H
