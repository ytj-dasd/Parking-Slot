#include "bproject_widget.h"
#include <common/time/duration.h>
namespace welkin::bamboo {

BProjectWidget::BProjectWidget(QWidget *parent)
    : UProjectWidget(parent) {

}

BProjectWidget::~BProjectWidget() {

}

Project* BProjectWidget::getProject() {
    if (!_project) {return nullptr;}
    return _project->as<Project>();
}

void BProjectWidget::initProject() {
    _project.reset(new Project());
}

void BProjectWidget::showRectImage(uface::canvas2d::RectImageLayer* layer,
        QTreeWidgetItem *tree_item, UItem *uface_item, bool show) {
    if (show) {
        auto image_folder = uface_item->as<common::file_system::RectImageFolder>();
        RETURN_IF(!image_folder || !image_folder->isValid())
        layer->setRectImage(image_folder);
        layer->setVisible(true);
    } else {
        layer->setVisible(false);
    }
}
void BProjectWidget::showRasterImage(uface::canvas2d::RasterImageLayer* layer,
        QTreeWidgetItem *tree_item, UItem *uface_item, bool show) {
    if (show) {
        auto image_folder = uface_item->as<common::file_system::RasterImageFolder>();
        RETURN_IF(!image_folder || !image_folder->isValid())
        layer->setRasterImage(image_folder);
        layer->setVisible(true);
    } else {
        layer->setVisible(false);
    }
}
void BProjectWidget::showBoxCloud(CloudViewer* viewer,
        QTreeWidgetItem *tree_item, UItem *uface_item, bool show) {
    RETURN_IF(!viewer)
    // auto item_id = uface_item->getID();
    // if (show) {
    //     auto cloud_folder = uface_item->as<
    //         common::file_system::BoxCloudFolder<PointX>>();
    //     auto scan_cloud = cloud_folder->getSourceCloud();
    //     viewer->updateCloud(item_id, scan_cloud);
    //     uface_item->reset(); //< 释放
    // } else {
    //     viewer->removeCloud(item_id);
    //     uface_item->reset(); //< 释放
    // }
}
void BProjectWidget::showBoxCloud(OSGCloudLayer* layer,
        QTreeWidgetItem *tree_item, UItem *uface_item, bool show) {
    auto item_id = uface_item->getID();
    if (show) {
        UStateProcessBar([&, item_id, uface_item, layer](){
            while (_running_thread_num.load() > 4) {
                common::Duration(0.2).sleep();
            }
            ++_running_thread_num;
            auto cloud_folder = uface_item->as<
                common::file_system::BoxCloudFolder<PointX>>();
            auto scan_cloud = cloud_folder->getSourceCloud();
            auto bbox = cloud_folder->getBoundingBox();
            uface_item->reset(); //< 释放
            layer->updateCloud(item_id, scan_cloud, bbox.cast<float>());
            --_running_thread_num;
            // std::cout << "thread num: " << _running_thread_num.load() << std::endl;
        });
    } else {
        layer->removeChild(item_id);
    }
}
void BProjectWidget::showModelList(OSGModelLayer* layer,
        QTreeWidgetItem *tree_item, UItem *uface_item, bool show) {
    auto item_id = uface_item->getID();
    if (show) {
        auto model_list_folder = uface_item->as<OriginDataModelListFolder>();
        auto file_list = model_list_folder->getOsgFileList();
        layer->loadModels(file_list, item_id);
    } else {
        layer->removeChild(item_id);
    }
}
}
