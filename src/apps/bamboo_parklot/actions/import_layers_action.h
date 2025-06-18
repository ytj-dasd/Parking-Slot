#ifndef WELKIN_BAMBOO_IMPORTLAYERSACTION_H
#define WELKIN_BAMBOO_IMPORTLAYERSACTION_H
#include <bamboo/widget/bproject_action.h>
#include <common/geometry/pose3.h>

namespace welkin {
namespace bamboo {
class MainViewer;
class ImportLayersAction : public BProjectAction {
    Q_OBJECT
public:
    explicit ImportLayersAction(BProjectWidget* project_widget = nullptr);
    virtual ~ImportLayersAction();
    QString getID() override {
        return QString("import_layers");
    }
    void setMainViewer(MainViewer* main_viewer);
protected:
    bool isDataPrepared() override;
    bool prepareParameter() override;
    void process() override;
private:
    QStringList _layer_filepaths = QStringList();
    common::Pose3d _pose_trans;
    MainViewer* _main_viewer;
};
} // namespace bamboo
} // namespace welkin

#endif // WELKIN_BAMBOO_IMPORTLAYERSACTION_H
