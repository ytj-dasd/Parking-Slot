#include "project.h"
#include <uface/base/uconverter.h>

namespace welkin::bamboo {

Project::Project() : BaseProject() {
    this->initFolders(
        UFQ("原始数据"), UFQ("源数据"), UFQ("建筑物"), UFQ("特征图层"));
}

Project::~Project() {

}

}
