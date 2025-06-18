#pragma once
#include <QIcon>
#include <QString>
#include <QAction>
#include <QObject>
#include <QWidget>
#include "bamboo/plugin/tool_plugin.h"

#include "cloud_convertor_widget.h"
namespace welkin::bamboo::tool_convertor {
class ToolConvertorPlugin : public ToolPlugin {
    Q_OBJECT
    // Q_PLUGIN_METADATA(IID "bamboo.tool.convertor" FILE "extrafilters.json")
    Q_PLUGIN_METADATA(IID "bamboo.tool.convertor")
    Q_INTERFACES(welkin::bamboo::IPlugin)
public:
    ToolConvertorPlugin(QObject* parent = nullptr);
    virtual ~ToolConvertorPlugin();

    QIcon getIcon() const override;
    QString getText() const override;
    QWidget* getWidget() override;

private:
    CloudConvertorWidget* _widget = nullptr;
};

}
