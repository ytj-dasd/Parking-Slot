#pragma once
#include <bamboo/widget/bproject_widget.h>
#include "main_viewer.h"

namespace welkin::bamboo {
class EpsdLayerManager;
class AutoDetectionAction;
class ImportLayersAction;
class ImportDetectionAction;
class ExportLayerAsDxfAction;
class ExportCloudAction;
class ProjectWidget : public BProjectWidget {
    Q_OBJECT
public:
    explicit ProjectWidget(QWidget* parent = nullptr);
    virtual ~ProjectWidget();

    void setMainViewer(MainViewer* main_viewer);
    MainViewer* getMainViewer() {return _main_viewer;}

public Q_SLOTS:
    void onInitMainViewerPlot();
protected:
    void fillItemMenu(int32_t type, QMenu* menu) override;
    void procCheckStateChangedItem(QTreeWidgetItem* tree_item, UItem* uface_item) override;
private:
    void initGlobalMenu();
private:
    MainViewer* _main_viewer = nullptr;
    AutoDetectionAction* _auto_detection_action = nullptr;
    ImportLayersAction* _import_layers_action = nullptr;
    ImportDetectionAction* _import_detection_action = nullptr;
    ExportLayerAsDxfAction* _export_layer_action = nullptr;
    ExportCloudAction* _export_cloud_action = nullptr;
};
}
