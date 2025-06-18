#pragma once
#include <QIcon>
#include <QString>
#include <QAction>
#include <QObject>
#include <QWidget>
#include "bamboo/plugin/iplugin.h"
namespace welkin::bamboo {
class BAMBOO_EXPORT ToolPlugin :
        public QObject, public IPlugin {
    Q_OBJECT
public:
    ToolPlugin(QObject* parent = nullptr);
    virtual ~ToolPlugin();

    virtual bool init() override;
    virtual void release() override;

    virtual QIcon getIcon() const = 0;
    virtual QString getText() const = 0;
    virtual QWidget* getWidget() = 0;
};

}
