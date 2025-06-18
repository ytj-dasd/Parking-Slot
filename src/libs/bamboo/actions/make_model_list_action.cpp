#include "make_model_list_action.h"
#include <common/base/uuid.h>
#include <common/base/dir.h>
#include <common/geometry/box.h>

#include "model_param_dialog.h"
namespace welkin::bamboo {
MakeModelListAction::MakeModelListAction(BProjectWidget* project_widget)
        : BProjectAction(project_widget, true) {
    this->setText(tr("Make Model List"));
    this->setToolTip(tr("Make Model List"));
    this->setIcon(UIcon("item_icon/osg_models.png"));
    _enable_progress = true;
}
MakeModelListAction::~MakeModelListAction() {

}
bool MakeModelListAction::isDataPrepared() {
    auto cloud_list_folder = this->getProject()->getCloudListFolder();
    return cloud_list_folder->isValid();
}
bool MakeModelListAction::prepareParameter() {
    ModelParamDialog dlg;
    RETURN_FALSE_IF(dlg.exec() != QDialog::Accepted)

    _color_map = dlg.getColorMode();
    _min_ratio = dlg.getMinRatio();
    _max_ratio = dlg.getMaxRatio();
    return true;
}
void MakeModelListAction::process() {
    auto origin_data_folder = this->getProject()->getOriginDataFolder();
    origin_data_folder->makeCloudModel(_color_map, _min_ratio, _max_ratio);
    auto model_folder = origin_data_folder->getModelListFolder();
    this->refreshItem(model_folder);
}
}