#ifndef WELIN_BAMBOO_IMPORT_DETECTION_DIALOG_H
#define WELIN_BAMBOO_IMPORT_DETECTION_DIALOG_H

#include <QDialog>
#include <common/geometry/pose3.h>

namespace welkin {
namespace bamboo {

namespace Ui {
class ImportDetectionDialog;
}

class ImportDetectionDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ImportDetectionDialog(QWidget *parent = nullptr);
    ~ImportDetectionDialog();

    void setHistoryPath(const QString& history_path);
    void setPaths(const QStringList& paths);
    void setLocalPose(const common::Pose3d& pose);
    void setGlobalPose(const common::Pose3d& pose);

    QStringList getLayerFilePaths() const;
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
    Ui::ImportDetectionDialog *ui;
    common::Pose3d _eye_pose;
    common::Pose3d _local_pose;
    common::Pose3d _global_pose;
    QStringList _paths;
    QString _history_path;
};


} // namespace bamboo
} // namespace welin
#endif // WELIN_BAMBOO_IMPORT_LAYERS_DIALOG_H
