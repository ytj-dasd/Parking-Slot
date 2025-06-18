#include "osg_model_layer.h"
#include <common/base/dir.h>
#include <common/base/file.h>

#include <osgDB/ReadFile>

namespace welkin::bamboo {
OSGModelLayer::OSGModelLayer(QObject* parent) : OSGGroupLayer(parent) {
    
}
OSGModelLayer::~OSGModelLayer() {

}
bool OSGModelLayer::loadModel(const std::string& filename, const std::string& id) {
    osg::ref_ptr<osg::Node> loaded_model = osgDB::readRefNodeFile(filename);
    RETURN_FALSE_IF(!loaded_model)

    this->addChild(id, loaded_model.get());
    return true;
}
bool OSGModelLayer::loadModels(const std::vector<std::string>& file_list, const std::string& id) {
    UStateProcessBar([&, file_list, id](){
        osg::ref_ptr<osg::Group> model_node = new osg::Group();
        for (int i = 0; i < file_list.size(); ++i) {
            UProgressTextValue("", i, file_list.size());
            auto filename = file_list[i];
            auto sub_model = osgDB::readRefNodeFile(filename);
            if (!sub_model) {
                UWarn(QObject::tr("Error load model: %1").arg(filename.c_str()));
                continue;
            }
            // UInfo(tr("Load: %1").arg(filename.c_str()));
            model_node->addChild(sub_model);
        }
        this->removeChildToBuffer(id);
        this->addChildToBuffer(id, model_node);
        emit this->childChanged();
    });
    return true;
}
bool OSGModelLayer::loadDescModel(const std::string& filename, const std::string& id) {
    std::ifstream infile(filename);
    if (!infile.is_open()) {return false;}
    auto model_path = common::Dir(filename).getFilePath();
    std::vector<std::string> file_list = common::file::GetAsLineList(filename, true);
    for (auto& filename : file_list) {
        filename = common::Dir(model_path, filename).getPath();
    }
    return loadModels(file_list, id);
}
}