#include "osg_geode_layer.h"

namespace welkin::bamboo {
OSGGeodeLayer::OSGGeodeLayer(QObject* parent) : OSGLayer(parent) {
    _root_node = new osg::Geode();
}
OSGGeodeLayer::~OSGGeodeLayer() {
    
}
bool OSGGeodeLayer::hasChild(const std::string& name) const {
    std::lock_guard<std::mutex> lock(_mutex);
    int child_count = _root_node->getNumDrawables();
    for (int i = 0; i < child_count; ++i) {
        auto child = _root_node->getDrawable(i);
        CONTINUE_IF(child->getName() != name)
        return true;
    }
    return false;
}
bool OSGGeodeLayer::removeAllChildren() {
    std::lock_guard<std::mutex> lock(_mutex);
    int child_count = _root_node->getNumDrawables();
    RETURN_FALSE_IF(child_count < 1)
    _root_node->removeDrawables(0, child_count);
    this->clearLayerData();
    return true;
}
bool OSGGeodeLayer::removeChild(const std::string& name) {
    std::lock_guard<std::mutex> lock(_mutex);
    RETURN_FALSE_IF(name.empty());
    int child_count = _root_node->getNumDrawables();
    for (int i = 0; i < child_count; ++i) {
        auto child = _root_node->getDrawable(i);
        if (child->getName() == name) {
            return _root_node->removeDrawable(child);
        }
    }
    return false;
}
osg::Drawable* OSGGeodeLayer::findDrawable(const std::string& name) {
    if (name.empty()) {return nullptr;}
    std::lock_guard<std::mutex> lock(_mutex);
    int child_num = _root_node->getNumDrawables();
    if (_last_find_index >= 0 || _last_find_index < child_num) {
        auto child = _root_node->getDrawable(_last_find_index);
        if (child->getName() == name) {return child;}
    }
    for (int i = 0; i < child_num; ++i) {
        auto child = _root_node->getDrawable(i);
        if (child->getName() == name) {
            _last_find_index = i;
            return child;
        }
    }
    _last_find_index = -1;
    return nullptr;
}
void OSGGeodeLayer::addDrawable(const std::string& name, osg::Drawable* child) {
    this->removeChild(name);
    std::lock_guard<std::mutex> lock(_mutex);
    child->setName(name);
    _root_node->addDrawable(child);
}
}