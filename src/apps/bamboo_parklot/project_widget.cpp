#include "project_widget.h"
#include <common/base/uuid.h>
#include "actions/import_layers_action.h"
#include "actions/export_layer_as_dxf_action.h"

namespace welkin::bamboo {
ProjectWidget::ProjectWidget(QWidget *parent)
        : BProjectWidget(parent) {
    _is_item_checkable = true;

    ////////////
    this->addVisibleItem(ITEM_ORIGIN_DATA_FOLDER);
    this->addVisibleItem(ITEM_ORIGIN_DATA_DOM_FOLDER);
    _import_layers_action = new ImportLayersAction(this);
    this->registeAction(_import_layers_action);
    _export_layer_action = new ExportLayerAsDxfAction(this);
    this->registeAction(_export_layer_action);

    _toolbar_project->setVisible(true);
    _toolbar_project->addSeparator();
    this->addActionToToolBar("import_layers", _toolbar_project, false);
    this->addActionToToolBar("export_layer_as_dxf", _toolbar_project, false);
    
    connect(this, SIGNAL(projectOpened()), this, SLOT(onInitMainViewerPlot()));
    this->initGlobalMenu();
}

ProjectWidget::~ProjectWidget() {

}
void ProjectWidget::setMainViewer(MainViewer* main_viewer) {
    _main_viewer = main_viewer;
    _import_layers_action->setMainViewer(main_viewer);
    _export_layer_action->setMainViewer(main_viewer);
}

void ProjectWidget::onInitMainViewerPlot() {
    // 测时使用
    _main_viewer->init(this->getProject());
}

void ProjectWidget::fillItemMenu(int32_t type, QMenu *menu) {
    Q_UNUSED(type)
    Q_UNUSED(menu)
}

void ProjectWidget::procCheckStateChangedItem(
        QTreeWidgetItem *tree_item, UItem *uface_item) {
    int32_t item_type = uface_item->getType();
    bool is_checked = bool(tree_item->checkState(0) == Qt::Checked);
    if (item_type == ITEM_ORIGIN_DATA_DOM_FOLDER) {
        this->showRasterImage(_main_viewer->rasterImageLayer(),
            tree_item, uface_item, is_checked);
    }
}
void ProjectWidget::initGlobalMenu() {
    // 文件
    auto file_menu = IMainWindow::Instance()->getFileMenu();
    file_menu->addAction(this->getOpenAction());
    file_menu->addAction(this->getCreateAction());
    file_menu->addAction(this->getCloseAction());
    file_menu->addAction(this->getRefreshAction());
}
}
