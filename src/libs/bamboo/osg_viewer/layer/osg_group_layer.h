#pragma once
#include <QObject>
#include <osg/Geode>
#include <osg/Group>
#include "bamboo/osg_viewer/osg_layer.h"

namespace welkin::bamboo {
class BAMBOO_EXPORT OSGGroupLayer : public OSGLayer {
    Q_OBJECT
public:
    OSGGroupLayer(QObject* parent = nullptr);
    virtual ~OSGGroupLayer();

    osg::Node* getRootNode() override {return _root_node.get();}
    const osg::Node* getRootNode() const override {return _root_node.get();}
    
    bool hasChild(const std::string& name) const override;
    bool removeAllChildren() override;
    bool removeChild(const std::string& name) override;

    osg::Node* findChild(const std::string& name);

    void addChild(const std::string& name, osg::Node* child);

    // 缓存，用于多线程
    void addChildToBuffer(const std::string& name, osg::ref_ptr<osg::Node> child);
    void removeChildToBuffer(const std::string& name);
    
signals:
    void childChanged();
protected slots:
    void onUpdateChild();
protected:
    int _last_find_index = -1;
    mutable std::mutex _add_node_mutex;
    std::list<osg::ref_ptr<osg::Node>> _add_node_list;

    mutable std::mutex _rm_node_mutex;
    std::list<std::string> _rm_node_list;

    mutable std::mutex _mutex;
    osg::ref_ptr<osg::Group> _root_node = nullptr;
};
}
