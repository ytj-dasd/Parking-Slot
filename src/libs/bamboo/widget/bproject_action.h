#pragma once
#include <uface/widget/uproject_action.h>
#include <bamboo/widget/bproject_widget.h>
#include "bamboo/file_system/project.h"

namespace welkin::bamboo {
class BAMBOO_EXPORT BProjectAction : public UProjectAction {
    Q_OBJECT
public:
    explicit BProjectAction(BProjectWidget* project_widget = nullptr, bool auto_checked = false);
    virtual ~BProjectAction();

    Project* getProject();

    BProjectWidget* getBProjectWidget();
    const BProjectWidget* getBProjectWidget() const;
};
}
