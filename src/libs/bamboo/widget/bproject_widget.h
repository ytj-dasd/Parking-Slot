#pragma once
#include <uface/interface/imainwindow.h>
#include <uface/widget/uproject_widget.h>
#include <uface/base/uconverter.h>
#include <uface/widget/uwidget.h>
#include <uface/canvas2d/layer.hpp>

#include "bamboo/cloud_viewer/cloud_viewer.h"
#include "bamboo/file_system/project.h"
#include "bamboo/osg_viewer/layer/osg_model_layer.h"
#include "bamboo/osg_viewer/layer/osg_cloud_layer.h"
namespace welkin::bamboo {
class BAMBOO_EXPORT BProjectWidget : public UProjectWidget {
    Q_OBJECT
public:
    explicit BProjectWidget(QWidget* parent = nullptr);
    virtual ~BProjectWidget();

    Project* getProject();

public:
    void showRectImage(uface::canvas2d::RectImageLayer* layer, 
        QTreeWidgetItem *tree_item, UItem *uface_item, bool show = true);
    void showRasterImage(uface::canvas2d::RasterImageLayer* layer, 
        QTreeWidgetItem *tree_item, UItem *uface_item, bool show = true);

    void showBoxCloud(CloudViewer* viewer,
        QTreeWidgetItem *tree_item, UItem *uface_item, 
        bool show = true);
    void showBoxCloud(OSGCloudLayer* layer,
        QTreeWidgetItem *tree_item, UItem *uface_item, 
        bool show = true);
    
    void showModelList(OSGModelLayer* layer,
        QTreeWidgetItem *tree_item, UItem *uface_item, 
        bool show = true);
protected:
    void initProject() override;
private:
    std::atomic<int> _running_thread_num{0};
};

}
