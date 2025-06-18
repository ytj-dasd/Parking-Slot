#pragma once
#include <uface/uface.hpp>
#include "project_widget.h"
#include "main_viewer.h"

namespace welkin::bamboo {
class MainWindow : public UMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    virtual ~MainWindow();

protected:
    bool initSurface() override;

private:
    MainViewer* _main_viewer = nullptr;
    ProjectWidget* _project_widget = nullptr;
};
}
