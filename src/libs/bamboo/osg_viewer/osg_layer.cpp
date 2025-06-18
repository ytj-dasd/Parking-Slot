#include "osg_layer.h"
#include <uface/base/uconverter.h>

namespace welkin::bamboo {
OSGLayer::OSGLayer(QObject* parent) : QObject(parent) {

}
OSGLayer::~OSGLayer() {

}

QString OSGLayer::name() const {
    return UTQ(getName());
}
const std::string& OSGLayer::getName() const {
    return this->getRootNode()->getName();
}
void OSGLayer::setName(const std::string& name) {
    this->getRootNode()->setName(name);
}

bool OSGLayer::isVisible() const {
    auto root_node = getRootNode();
    return root_node && root_node->getNodeMask() == 1;
}
void OSGLayer::setViewer(OSGViewer* viewer) {
    _viewer = viewer;
}

void OSGLayer::setVisible(bool visible) {
    if (auto root_node = getRootNode()) {
        root_node->setNodeMask(visible ? 1 : 0);
    }
}
}
