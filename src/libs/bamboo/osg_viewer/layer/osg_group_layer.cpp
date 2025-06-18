#include "osg_group_layer.h"
#include <common/time/tic_toc.h>
#include <bamboo/osg_viewer/osg_viewer.h>
namespace welkin::bamboo {
OSGGroupLayer::OSGGroupLayer(QObject* parent) : OSGLayer(parent) {
    _root_node = new osg::Group();
    connect(this, SIGNAL(childChanged()), this, SLOT(onUpdateChild()));
}
OSGGroupLayer::~OSGGroupLayer() {

}
bool OSGGroupLayer::hasChild(const std::string& name) const {
    std::lock_guard<std::mutex> lock(_mutex);
    int child_num = _root_node->getNumChildren();
    for (int i = 0; i < child_num; ++i) {
        auto child = _root_node->getChild(i);
        CONTINUE_IF(child->getName() != name)
        return true;
    }
    return false;
}
bool OSGGroupLayer::removeAllChildren() {
    std::lock_guard<std::mutex> lock(_mutex);
    for (int i = _root_node->getNumChildren(); i > 0; --i) {
        auto child = _root_node->getChild(i - 1);
        std::cout << "remove: " << child->getName() << std::endl;
        _root_node->removeChild(child);
    }
    std::cout << "remove all children: " << _root_node->getNumChildren() << std::endl;
    return true;
    // int child_count = _root_node->getNumChildren();
    // RETURN_FALSE_IF(child_count < 1)
    // return _root_node->removeChildren(0, child_count);
}
bool OSGGroupLayer::removeChild(const std::string& name) {
    RETURN_FALSE_IF(name.empty());
    std::lock_guard<std::mutex> lock(_mutex);
    for (int i = _root_node->getNumChildren(); i > 0; --i) {
        auto child = _root_node->getChild(i - 1);
        if (child->getName() == name) {
            return _root_node->removeChild(child);
        }
    }
    return false;
}
osg::Node* OSGGroupLayer::findChild(const std::string& name) {
    if (name.empty()) {return nullptr;}
    std::lock_guard<std::mutex> lock(_mutex);
    int child_num = _root_node->getNumChildren();
    if (_last_find_index >= 0 || _last_find_index < child_num) {
        auto child = _root_node->getChild(_last_find_index);
        if (child->getName() == name) {return child;}
    }
    for (int i = 0; i < child_num; ++i) {
        auto child = _root_node->getChild(i);
        if (child->getName() == name) {
            _last_find_index = i;
            return child;
        }
    }
    _last_find_index = -1;
    return nullptr;
}
void OSGGroupLayer::addChild(const std::string& name, osg::Node* child) {
    std::lock_guard<std::mutex> lock(_mutex);
    child->setName(name);
    _root_node->addChild(child);

    // osgUtil::Optimizer optimizer;
    // optimizer.optimize(_root_node);
}
void OSGGroupLayer::removeChildToBuffer(const std::string& name) {
    std::lock_guard<std::mutex> lock(_rm_node_mutex);
    _rm_node_list.push_back(name);
}
void OSGGroupLayer::addChildToBuffer(const std::string& name, osg::ref_ptr<osg::Node> child) {
    std::lock_guard<std::mutex> lock(_add_node_mutex);
    child->setName(name);
    _add_node_list.push_back(child);
}
void OSGGroupLayer::onUpdateChild() {
    {
        std::lock_guard<std::mutex> lock(_rm_node_mutex);
        for (auto id : _rm_node_list) {
            this->removeChild(id);
        }
        _rm_node_list.clear();
    }
    {
        std::lock_guard<std::mutex> lock(_add_node_mutex);
        while (!_add_node_list.empty()) {
            auto node = _add_node_list.back();
            _add_node_list.pop_back();
            std::lock_guard<std::mutex> lock(_mutex);
            _root_node->addChild(node.release());
        }
    }

    // osgUtil::Optimizer optimizer;
    // optimizer.optimize(_root_node);
    // if (_databasePager.valid()) {
    //     // register any PagedLOD that need to be tracked in the scene graph
    //     _databasePager->registerPagedLODs(_root_node);
    // }
    //tt.print();
}
}