#include "plugin.h"
#include <uface/uface.hpp>
namespace welkin::bamboo::tool_convertor {
ToolConvertorPlugin::ToolConvertorPlugin(QObject* parent)
        : ToolPlugin(parent) {

}
ToolConvertorPlugin::~ToolConvertorPlugin() {
    
}
QIcon ToolConvertorPlugin::getIcon() const {
    return UIcon("logo.png");
}
QString ToolConvertorPlugin::getText() const {
    return QString(tr("CloudConvertor"));
}
QWidget* ToolConvertorPlugin::getWidget() {
    if (_widget) {
        auto mw = IMainWindow::Instance()->getMainWindow();
        _widget = new CloudConvertorWidget(mw);
    }
    return _widget;
}
}
