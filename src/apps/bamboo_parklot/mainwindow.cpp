#include "mainwindow.h"
#include <QDir>
#include <QMenuBar>
#include <QDockWidget>

namespace welkin::bamboo {
MainWindow::MainWindow(QWidget* parent) : UMainWindow(parent) {
    this->addCopyRightInfo(tr("Shanghai Gtop industrial development group co.LTD."));
}
MainWindow::~MainWindow() {}

bool MainWindow::initSurface() {
    _main_viewer = new MainViewer(this);
    this->setCentralWidget(_main_viewer);

    _project_widget = new ProjectWidget(this);
    addAsDockWidget(_project_widget, tr("Project"), Qt::LeftDockWidgetArea);
    _project_widget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
    _project_widget->setMainViewer(_main_viewer);

    // 工具栏
    auto proj_toolbar = _project_widget->getProjectToolBar();
    proj_toolbar->setVisible(true);
    this->addToolBar(proj_toolbar);
    auto draw_toolbar = _main_viewer->drawToolBar();
    this->addToolBar(Qt::RightToolBarArea, draw_toolbar);

    // 清除画布
    connect(_project_widget, SIGNAL(projectClosed()), _main_viewer, SLOT(clearViewer()));

    auto optimate_action = proj_toolbar->addAction(
        QIcon(":/bamboo/images/optimate.png"), tr("Optimate"));
    connect(optimate_action, SIGNAL(triggered()), _main_viewer, SLOT(onOptimateSlot()));
    return true;
}
}  // namespace welkin::uface
