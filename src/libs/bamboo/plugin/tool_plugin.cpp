#include "tool_plugin.h"
#include <uface/uface.hpp>
namespace welkin::bamboo {
ToolPlugin::ToolPlugin(QObject* parent)
        : QObject(parent) {

}
ToolPlugin::~ToolPlugin() {
    this->release();
}
bool ToolPlugin::init() {
    auto imw = IMainWindow::Instance();
    auto tool_menu = imw->getToolMenu();
    tool_menu->addAction(this->getIcon(), this->getText(),
        this,  [&](){this->getWidget()->show();});

    return true;
}
void ToolPlugin::release() {

}
}
