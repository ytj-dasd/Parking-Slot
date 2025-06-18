#pragma once

#include <osg/Geode>
#include "bamboo/osg_viewer/osg_layer.h"
namespace welkin::bamboo {
class BAMBOO_EXPORT OSGGeodeLayer : public OSGLayer {
    Q_OBJECT
public:
    OSGGeodeLayer(QObject* parent = nullptr);
    virtual ~OSGGeodeLayer();

    osg::Node* getRootNode() override {return _root_node.get();}
    const osg::Node* getRootNode() const override {return _root_node.get();}
    
    bool hasChild(const std::string& name) const override;
    bool removeAllChildren() override;
    bool removeChild(const std::string& name) override;

    osg::Drawable* findDrawable(const std::string& name);

    void addDrawable(const std::string& name, osg::Drawable* child);

protected:
   virtual void clearLayerData() {}

protected:
    int _last_find_index = -1;
    mutable std::mutex _mutex;
    osg::ref_ptr<osg::Geode> _root_node = nullptr;
};
}
