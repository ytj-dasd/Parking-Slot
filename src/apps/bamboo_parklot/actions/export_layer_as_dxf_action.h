#pragma once
#include <bamboo/widget/bproject_action.h>
#include <bamboo/io/dxf_io.h>

namespace welkin::bamboo {
class MainViewer;
class ExportLayerAsDxfAction : public BProjectAction {
    Q_OBJECT
public:
    explicit ExportLayerAsDxfAction(BProjectWidget* project_widget = nullptr);
    virtual ~ExportLayerAsDxfAction();
    QString getID() override {
        return QString("export_layer_as_dxf");
    }
    void setMainViewer(MainViewer* main_viewer);
protected:
    bool isDataPrepared() override;
    bool prepareParameter() override;
    void exportTerrainToDxf(DxfOutput& dxf_file);
    void process() override;
private:
    QString _save_path = QString();
    common::Pose3d _pose_trans;
    MainViewer* _main_viewer;
};
}
