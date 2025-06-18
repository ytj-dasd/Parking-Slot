#pragma once
#include <QtPlugin>
#include "bamboo/base/macro.h"
namespace welkin::bamboo {
class BAMBOO_EXPORT IPlugin {
public:
    virtual ~IPlugin() {}

    virtual bool init() = 0;
    virtual void release() = 0;
};
}
Q_DECLARE_INTERFACE(welkin::bamboo::IPlugin, "bamboo.iplugin")
