#include "bproject_action.h"

namespace welkin::bamboo {
BProjectAction::BProjectAction(BProjectWidget* project_widget, bool auto_checked)
    : UProjectAction(project_widget, auto_checked) {

}
BProjectAction::~BProjectAction() {

}

Project* BProjectAction::getProject() {
    return this->getUProject()->as<Project>();
}

BProjectWidget* BProjectAction::getBProjectWidget() {
    return qobject_cast<BProjectWidget*>(_project_widget);
}
const BProjectWidget* BProjectAction::getBProjectWidget() const {
    return qobject_cast<BProjectWidget*>(_project_widget);
}
}