#pragma once
#include "bamboo/osg_viewer/layer/osg_group_layer.h"

namespace welkin::bamboo {
class BAMBOO_EXPORT OSGModelLayer : public OSGGroupLayer {
    Q_OBJECT
public:
    OSGModelLayer(QObject* parent = nullptr);
    virtual ~OSGModelLayer();

    bool loadModel(const std::string& filename, const std::string& id = "model");
    bool loadModels(const std::vector<std::string>& file_list, const std::string& id = "models");
    bool loadDescModel(const std::string& filename, const std::string& id = "models");
};
}