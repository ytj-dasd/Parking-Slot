#pragma once
#include <QColor>
#include <QObject>
#include <osg/Node>
#include <osgUtil/Optimizer>
#include <uface/state/ustate.h>
#include <uface/logger/ulogger.h>
#include <bamboo/base/macro.h>
namespace welkin::bamboo {

class OSGViewer;
class BAMBOO_EXPORT OSGLayer : public QObject {
    Q_OBJECT
public:
    OSGLayer(QObject* parent = nullptr);
    virtual ~OSGLayer();
    
    virtual bool init() {return true;}

    QString name() const;
    const std::string& getName() const;
    void setName(const std::string& name);

    virtual osg::Node* getRootNode() = 0;
    virtual const osg::Node* getRootNode() const = 0;

    bool isVisible() const;
    void setVisible(bool visible);

    virtual bool hasChild(const std::string& name) const = 0;
    virtual bool removeAllChildren() = 0;
    virtual bool removeChild(const std::string& name) = 0;

    void setViewer(OSGViewer* viewer);
protected:
    OSGViewer* _viewer = nullptr;
};

}
