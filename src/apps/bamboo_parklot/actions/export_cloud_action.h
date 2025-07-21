#pragma once
#include <bamboo/widget/bproject_action.h>

namespace welkin::bamboo {
class MainViewer;
class ExportCloudAction : public BProjectAction {
    Q_OBJECT
public:
    explicit ExportCloudAction(BProjectWidget* project_widget = nullptr);
    virtual ~ExportCloudAction();
    QString getID() override {
        return QString("export_cloud");
    }
    void setMainViewer(MainViewer* main_viewer);
protected:
    bool isDataPrepared() override;
    bool prepareParameter() override;
    void process() override;
private:
    QString _save_path = QString();
    MainViewer* _main_viewer;
};
}