#include "make_dom_action.h"
#include <common/base/uuid.h>
#include <common/base/dir.h>
#include <common/geometry/box.h>

#include "dom_param_dialog.h"

namespace welkin::bamboo {
MakeDOMAction::MakeDOMAction(BProjectWidget* project_widget)
        : BProjectAction(project_widget, true) {
    this->setText(tr("Make DOM"));
    this->setToolTip(tr("Make DOM"));
    this->setIcon(UIcon("item_icon/raster.png"));
    _enable_progress = true;
}
MakeDOMAction::~MakeDOMAction() {}

bool MakeDOMAction::isDataPrepared() {
    auto front_view_folder = this->getProject()->getFrontViewFolder();
    auto cloud_list_folder = this->getProject()->getCloudListFolder();
    return front_view_folder->isValid() && cloud_list_folder->isValid();
}
bool MakeDOMAction::prepareParameter() {
    auto project = this->getProject();
    auto front_view_folder = project->getFrontViewFolder();
    if (!front_view_folder->isValid()) {
        UError(tr("Front view is not exist!"));
        return false;
    }
    DOMParamDialog dlg(IMainWindow::GetMainWindow());
    dlg.setInverseY(true);
    dlg.setRectImage(front_view_folder);
    dlg.exec();
    if (!dlg.isOK()) {
        UError(tr("Cancel rect select!"));
        return false;
    }
    auto rect = dlg.getRect();
    if (!rect.isValid()) {
        UError(tr("Select rect is unvalid!"));
        return false;
    }
    _min_z = rect.range_y.min_v;
    _max_z = rect.range_y.max_v;
    UInfo(tr("min_z: %1").arg(_min_z));
    UInfo(tr("max_z: %1").arg(_max_z));

    _type = dlg.getType(); 
    _colormap = dlg.getColorMap();  
    _res = dlg.getRes(); 
    _min_value = dlg.getMinValue(); 
    _max_value = dlg.getMaxValue(); 
    return true;
}
void MakeDOMAction::process() {
    ///////////////
    auto project = this->getProject();
    auto origin_data_folder = project->getOriginDataFolder();
    origin_data_folder->projectToDOM(_min_z, _max_z,
        _res, _res, ProjectType(_type), _min_value, _max_value, _colormap);
    auto dom_folder = project->getDOMFolder();
    this->refreshItem(dom_folder);
}
}